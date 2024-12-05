/**
 * @file cmd_controller.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-07
 *
 * @copyright Copyright (c) 2024
 *
 */
// app
#include "robot_def.h"
#include "cmd_controller.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "rm_referee.h"

#include "ramp.h"
// bsp
#include "bsp_log.h"
#include <math.h>

static Publisher_t *chassis_cmd_pub;             // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub;           // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Publisher_t *ui_cmd_pub;        // UI控制消息发布者
static Subscriber_t *ui_feed_sub;      // UI反馈信息订阅者
static UI_Cmd_s ui_cmd_send;           // 传递给UI的控制信息
static UI_Upload_Data_s ui_fetch_data; // 从UI获取的反馈信息

static Publisher_t *master_cmd_pub;            // 上位机控制消息发布者
static Subscriber_t *master_feed_sub;          // 上位机反馈消息订阅者
static Master_Cmd_s master_cmd_send;           // 传递给上位机的控制信息
static Master_Upload_Data_s master_fetch_data; // 从上位机获得的反馈信息

static RC_ctrl_t *rc_data;           // 遥控器数据,初始化时返回
static referee_info_t *referee_data; // 用于获取裁判系统的数据
static CmdInstance cmd_media_param;  // 控制中介变量

void CmdDeviceInit()
{
    rc_data      = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    referee_data = RefereeHardwareInit(&huart6); // 裁判系统初始化,会同时初始化UI
}

void CmdParamInit()
{
#if PITCH_FEED_TYPE
    gimbal_cmd_send.pitch = 0;
#else
    gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif
    shoot_cmd_send.shoot_mode = SHOOT_OFF; // 初始化后发射机构失能

    ramp_init(&cmd_media_param.fb_ramp, RAMP_TIME);
    ramp_init(&cmd_media_param.lr_ramp, RAMP_TIME);

    cmd_media_param.ui_refresh_flag = 1;
    cmd_media_param.auto_rune       = 0;
}

void CmdMsgInit()
{
    gimbal_cmd_pub   = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub  = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub    = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub   = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    master_cmd_pub   = PubRegister("master_cmd", sizeof(Master_Cmd_s));
    master_feed_sub  = SubRegister("master_feed", sizeof(Master_Upload_Data_s));
    ui_cmd_pub       = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    ui_feed_sub      = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));
}

void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_data->referee_id.Robot_Color       = referee_data->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->GameRobotState.robot_id; // 计算客户端ID
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;          // 计算机器人ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}

void CalcOffsetAngle()
{
    // 从云台获取的当前yaw电机单圈角度
    float angle = gimbal_fetch_data.yaw_motor_single_round_angle;
    // 云台yaw轴当前角度
    float gimbal_yaw_current_angle = gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_YAW_ADDRESS_OFFSET];
    // 云台yaw轴目标角度
    float gimbal_yaw_set_angle = cmd_media_param.yaw_control;
    // 云台误差角
    float gimbal_error_angle = (gimbal_yaw_set_angle - gimbal_yaw_current_angle) * RAD_2_DEGREE;

#if YAW_ECD_GREATER_THAN_4096 // 如果大于180度
    float offset_angle = angle < 180.0f + YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f ? angle - YAW_ALIGN_ANGLE
                                                                                               : angle - YAW_ALIGN_ANGLE + 360.0f;
#else // 小于180度
    float offset_angle = angle >= YAW_ALIGN_ANGLE - 180.0f && angle <= YAW_ALIGN_ANGLE + 180.0f ? angle - YAW_ALIGN_ANGLE
                                                                                                : angle - YAW_ALIGN_ANGLE - 360.0f;
#endif
    chassis_cmd_send.gimbal_error_angle = offset_angle + gimbal_error_angle;
}

void GimbalModeSwitch()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    if (cmd_media_param.auto_aim || (cmd_media_param.rec_pitch != 0 && cmd_media_param.rec_yaw != 0)) {
        cmd_media_param.yaw_control   = master_fetch_data.rec_yaw;
        cmd_media_param.pitch_control = master_fetch_data.rec_pitch;
    } else {
        if (MOUSEKEYCONTROL) {
            cmd_media_param.yaw_control += rc_data[TEMP].mouse.x / 350.0f;
            cmd_media_param.pitch_control += -rc_data[TEMP].mouse.y / 15500.0f;
        } else if (ENTIREDISABLE) {
            cmd_media_param.pitch_control = cmd_media_param.pitch_control;
            cmd_media_param.yaw_control   = cmd_media_param.yaw_control;
        } else {
            cmd_media_param.yaw_control += -0.0000080f * rc_data[TEMP].rc.rocker_l_;
            cmd_media_param.pitch_control += -0.0000080f * rc_data[TEMP].rc.rocker_l1;
        }
    }

    if (cmd_media_param.yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_YAW_ADDRESS_OFFSET] >= PI) {
        cmd_media_param.yaw_control -= PI2;
    } else if (cmd_media_param.yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_YAW_ADDRESS_OFFSET] <= -PI) {
        cmd_media_param.yaw_control += PI2;
    }

    float current    = cmd_media_param.pitch_control;
    float limit_down = 25.0 * DEGREE_2_RAD;
    float limit_up   = -16.0 * DEGREE_2_RAD;

    if (current > limit_down)
        current = limit_down;
    if (current < limit_up)
        current = limit_up;

    cmd_media_param.pitch_control = current;
}

void ShootControl()
{
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.shoot_rate = 30;

    shoot_cmd_send.loader_rate = shoot_cmd_send.shoot_rate *
                                 360 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE;

    float rate_coef = 0;
    if (cmd_media_param.heat_coef == 1)
        rate_coef = 1;
    else if (cmd_media_param.heat_coef >= 0.8 && cmd_media_param.heat_coef < 1)
        rate_coef = 0.8;
    else if (cmd_media_param.heat_coef >= 0.6 && cmd_media_param.heat_coef < 0.8)
        rate_coef = 0.6;
    else if (cmd_media_param.heat_coef < 0.6)
        rate_coef = 0.4;
    cmd_media_param.heat_coef = ((referee_data->GameRobotState.shooter_id1_17mm_cooling_limit -
                                  referee_data->PowerHeatData.shooter_17mm_heat0 +
                                  rate_coef * referee_data->GameRobotState.shooter_id1_17mm_cooling_rate) *
                                 1.0f) /
                                (1.0f * referee_data->GameRobotState.shooter_id1_17mm_cooling_limit);
}

/**
 * @brief  紧急停止,双下
 *
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void emergencyhandler()
{
    gimbal_cmd_send.gimbal_mode              = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode            = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.SuperCap_flag_from_user = SUPER_USER_CLOSE;
    shoot_cmd_send.friction_mode             = FRICTION_OFF;
    shoot_cmd_send.load_mode                 = LOAD_STOP;
    shoot_cmd_send.shoot_mode                = SHOOT_OFF;

    LOGERROR("[CMD] emergency stop!");
}

static void remotecontrolset()
{
    static chassis_mode_e last_chassis_mode_ = CHASSIS_ZERO_FORCE;
    static friction_mode_e last_fric_mode_   = FRICTION_OFF;
    static loader_mode_e last_load_mode_     = LOAD_STOP;

    switch (rc_data[TEMP].rc.switch_right) {
        case RC_SW_MID:
            last_chassis_mode_ = chassis_cmd_send.chassis_mode;
            break;
        case RC_SW_DOWN:
            chassis_cmd_send.chassis_mode = (last_chassis_mode_ = CHASSIS_NO_FOLLOW) ? ((last_chassis_mode_ = CHASSIS_ROTATE) ? CHASSIS_REVERSE
                                                                                                                              : CHASSIS_ROTATE)
                                                                                     : CHASSIS_NO_FOLLOW;
            break;
        case RC_SW_UP:
            chassis_cmd_send.chassis_mode = (last_chassis_mode_ = CHASSIS_NO_FOLLOW) ? CHASSIS_FOLLOW_GIMBAL_YAW : CHASSIS_NO_FOLLOW;
            break;
        default:
            break;
    }

    switch (rc_data[TEMP].rc.switch_left) {
        case RC_SW_MID:
            last_fric_mode_ = shoot_cmd_send.friction_mode;
            last_load_mode_ = shoot_cmd_send.load_mode;
            break;
        case RC_SW_UP:
            shoot_cmd_send.friction_mode = (last_fric_mode_ = FRICTION_OFF) ? FRICTION_ON : FRICTION_OFF;
            break;
        case RC_SW_DOWN:
            if (shoot_cmd_send.friction_mode == FRICTION_ON) {
                shoot_cmd_send.load_mode = (last_load_mode_ == LOAD_BURSTFIRE) ? LOAD_STOP : last_load_mode_ + 1;
            }

        default:
            break;
    }

    // 底盘参数
    chassis_cmd_send.vx = 20000 / 660.0f * (float)rc_data[TEMP].rc.rocker_r_; // 水平方向
    chassis_cmd_send.vy = 20000 / 660.0f * (float)rc_data[TEMP].rc.rocker_r1; // 竖直方向
    chassis_cmd_send.wz = 5000;

    (rc_data[TEMP].rc.dial > 400)
        ? (chassis_cmd_send.SuperCap_flag_from_user = SUPER_USER_OPEN)
        : (chassis_cmd_send.SuperCap_flag_from_user = SUPER_USER_CLOSE);

    // 云台参数
    gimbal_cmd_send.yaw   = cmd_media_param.yaw_control;
    gimbal_cmd_send.pitch = cmd_media_param.pitch_control;

    // 新热量管理
    if (referee_data->GameRobotState.shooter_id1_17mm_cooling_limit - shoot_fetch_data.shooter_local_heat <= shoot_fetch_data.shooter_heat_control) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

static void chassisset()
{
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    chassis_cmd_send.wz           = 5000;

    static float current_speed_x = 0;
    static float current_speed_y = 0;

    if (rc_data[TEMP].key[KEY_PRESS].w) {
        chassis_cmd_send.vy = (current_speed_y + (40000 - current_speed_y) * ramp_calc(&cmd_media_param.fb_ramp));
    } else if (rc_data[TEMP].key[KEY_PRESS].s) {
        chassis_cmd_send.vy = (current_speed_y + (-40000 - current_speed_y) * ramp_calc(&cmd_media_param.fb_ramp));
    } else {
        chassis_cmd_send.vy = 0;
        ramp_init(&cmd_media_param.fb_ramp, RAMP_TIME);
    }

    if (rc_data[TEMP].key[KEY_PRESS].a) {
        chassis_cmd_send.vx = (current_speed_x + (40000 - current_speed_x) * ramp_calc(&cmd_media_param.lr_ramp));
    } else if (rc_data[TEMP].key[KEY_PRESS].d) {
        chassis_cmd_send.vx = (current_speed_x + (-40000 - current_speed_x) * ramp_calc(&cmd_media_param.lr_ramp));
    } else {
        chassis_cmd_send.vx = 0;
        ramp_init(&cmd_media_param.lr_ramp, RAMP_TIME);
    }

    current_speed_x = chassis_cmd_send.vx;
    current_speed_y = chassis_cmd_send.vy;
}

static void gimbalset()
{
    gimbal_cmd_send.yaw   = cmd_media_param.yaw_control;
    gimbal_cmd_send.pitch = cmd_media_param.pitch_control;
}

static void shootset()
{
    if (shoot_cmd_send.friction_mode == FRICTION_ON) {

        if (rc_data[TEMP].mouse.press_l)
            shoot_cmd_send.load_mode = cmd_media_param.auto_rune ? LOAD_1_BULLET : LOAD_BURSTFIRE;
        else
            shoot_cmd_send.load_mode = LOAD_STOP;

        if (referee_data->GameRobotState.shooter_id1_17mm_cooling_limit - shoot_fetch_data.shooter_local_heat <= shoot_fetch_data.shooter_heat_control)
            shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

static void keymodeset()
{

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) {
        case 1:
            if (chassis_cmd_send.chassis_mode != CHASSIS_ROTATE)
                chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            break;
        case 0:
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 2) {
        case 1:
            if (shoot_cmd_send.friction_mode != FRICTION_ON)
                shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
    }

    cmd_media_param.ui_refresh_flag = rc_data[TEMP].key[KEY_PRESS].r ? 1 : 0;

    cmd_media_param.auto_rune = rc_data[TEMP].key[KEY_PRESS].ctrl ? 1 : 0;

    chassis_cmd_send.SuperCap_flag_from_user = rc_data[TEMP].key[KEY_PRESS].shift ? SUPER_USER_OPEN : SUPER_USER_CLOSE;

    cmd_media_param.auto_aim = rc_data[TEMP].mouse.press_r ? 1 : 0;
}

/**
 * @brief 机器人复位函数，按下Ctrl+Shift+r
 *
 *
 */
static void robotreset()
{
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].r) {
        osDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset(); // 软件复位
    }
}

static void mousekeyset()
{
    chassisset();
    gimbalset();
    shootset();
    keymodeset();
    robotreset();
}

void CmdModeSet()
{
    if (MOUSEKEYCONTROL)
        mousekeyset();
    else if (RC_LOST || ENTIREDISABLE) {
        emergencyhandler();
    } else {
        remotecontrolset();
    }
}

void CmdMsgComm()
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(ui_feed_sub, &ui_fetch_data);
    SubGetMessage(master_feed_sub, &master_fetch_data);

    // chassis
    memcpy(&chassis_cmd_send.chassis_power, &referee_data->PowerHeatData.chassis_power, sizeof(float));
    memcpy(&chassis_cmd_send.power_buffer, &referee_data->PowerHeatData.chassis_power_buffer, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.level, &referee_data->GameRobotState.robot_level, sizeof(uint8_t));
    memcpy(&chassis_cmd_send.power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));

    // shoot
    memcpy(&shoot_cmd_send.shooter_heat_cooling_rate, &referee_data->GameRobotState.shooter_id1_17mm_cooling_rate, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_referee_heat, &referee_data->PowerHeatData.shooter_17mm_heat0, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_cooling_limit, &referee_data->GameRobotState.shooter_id1_17mm_cooling_limit, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.bullet_speed, &referee_data->ShootData.bullet_speed, sizeof(float));

    // UI
    memcpy(&ui_cmd_send.init_flag, &referee_data->init_flag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.robot_id_for_ui, &referee_data->referee_id, sizeof(referee_id_t));
    memcpy(&ui_cmd_send.ui_refresh_flag, &cmd_media_param.ui_refresh_flag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.chassis_mode, &chassis_cmd_send.chassis_mode, sizeof(chassis_mode_e));
    memcpy(&ui_cmd_send.chassis_attitude_angle, &gimbal_fetch_data.yaw_motor_single_round_angle, sizeof(uint16_t));
    memcpy(&ui_cmd_send.friction_mode, &shoot_cmd_send.friction_mode, sizeof(friction_mode_e));
    memcpy(&ui_cmd_send.rune_mode, &cmd_media_param.auto_rune, sizeof(uint8_t));
    memcpy(&ui_cmd_send.SuperCap_mode, &chassis_fetch_data.CapFlag_open_from_real, sizeof(uint8_t));
    memcpy(&ui_cmd_send.SuperCap_voltage, &chassis_fetch_data.cap_voltage, sizeof(float));

    // master
    static uint8_t frame_head[] = {0xAF, 0x32, 0x00, 0x12};
    memcpy(master_cmd_send.frame_head, frame_head, 4);
    memcpy(master_cmd_send.ins_quat, gimbal_fetch_data.gimbal_imu_data->INS_data.INS_quat, sizeof(float) * 4);
    memcpy(&master_cmd_send.robot_id, &referee_data->GameRobotState.robot_id, sizeof(uint8_t));
    memcpy(&master_cmd_send.rune_mode, &cmd_media_param.auto_rune, sizeof(uint8_t));

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
    PubPushMessage(master_cmd_pub, (void *)&master_cmd_send);
}
