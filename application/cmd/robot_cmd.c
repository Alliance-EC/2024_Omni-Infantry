// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "referee_UI.h"
#include "tool.h"
#include "super_cap.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#if PITCH_FEED_TYPE                                                  // Pitch电机反馈数据源为陀螺仪
#define PTICH_HORIZON_ANGLE 0                                        // PITCH水平时电机的角度
#if PITCH_ECD_UP_ADD
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#else
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
#else                                                                   // PITCH电机反馈数据源为编码器
#define PTICH_HORIZON_ANGLE    (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // PITCH水平时电机的角度,0-360
#define PITCH_LIMIT_ANGLE_UP   (PITCH_POS_MAX_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (PITCH_POS_MIN_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回

static HostInstance *host_instance; // 上位机接口
// 这里的四元数以wxyz的顺序
static uint8_t vision_recv_data[9];  // 从视觉上位机接收的数据-绝对角度，第9个字节作为识别到目标的标志位
static uint8_t vision_send_data[21]; // 给视觉上位机发送的数据-四元数

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

Referee_Interactive_info_t Referee_Interactive_info; // 发送给UI绘制的数据
extern char Send_Once_Flag;                          // 初始化UI标志

int remote_work_condition = 0; // 遥控器是否离线判断

uint8_t Super_flag            = 0; // 超电标志位
void HOST_RECV_CALLBACK()
{
    memcpy(vision_recv_data, host_instance->comm_instance, host_instance->RECV_SIZE);
    vision_recv_data[8] = 1;
}
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    HostInstanceConf host_conf = {
        .callback  = HOST_RECV_CALLBACK,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = 8,
    };
    host_instance = HostInit(&host_conf); // 视觉通信串口

    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0x312,
            .rx_id      = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD

#if PITCH_FEED_TYPE
    gimbal_cmd_send.pitch = 0;
#else
    gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
    else

        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#endif
}

/**
 * @brief 对Pitch轴角度变化进行限位
 *
 */
static void PitchAngleLimit()
{
    float current, limit_min, limit_max;
    current = gimbal_cmd_send.pitch;
#if PITCH_INS_FEED_TYPE
    limit_min = PITCH_LIMIT_ANGLE_DOWN * DEGREE_2_RAD;
    limit_max = PITCH_LIMIT_ANGLE_UP * DEGREE_2_RAD;
#else
    limit_min = PITCH_LIMIT_ANGLE_DOWN;
    limit_max = PITCH_LIMIT_ANGLE_UP;
#endif

#if PITCH_ECD_UP_ADD // 云台抬升,反馈值增
    if (current > limit_max)
        current = limit_max;
    if (current < limit_min)
        current = limit_min;
#else
    if (current < limit_max)
        current = limit_max;
    if (current > limit_min)
        current = limit_min;
#endif

    gimbal_cmd_send.pitch = current;
}

float yaw_control; // 遥控器YAW自由度输入值
/**
 * @brief 云台Yaw轴反馈值改单圈角度后过圈处理
 *
 */
static void YawControlProcess()
{
    if (yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] > 180) {
        yaw_control -= 360;
    } else if (yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] < -180) {
        yaw_control += 360;
    }
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?

    if (switch_is_up(rc_data[TEMP].rc.switch_right) && switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[中],右侧开关状态[上],小陀螺
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
        shoot_cmd_send.friction_mode  = FRICTION_REVERSE;
        shoot_cmd_send.load_mode      = LOAD_STOP;
    } else if (switch_is_mid(rc_data[TEMP].rc.switch_right) && switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[中],右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
        shoot_cmd_send.friction_mode  = FRICTION_REVERSE;
        shoot_cmd_send.load_mode      = LOAD_STOP;
    } else if (switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[中],右侧开关状态[下],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_FREE_MODE;
        shoot_cmd_send.friction_mode  = FRICTION_REVERSE;
        shoot_cmd_send.load_mode      = LOAD_STOP;
    } else if (switch_is_mid(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[下],右侧开关状态[中],底盘和云台分离,底盘保持不转动，打开摩擦轮
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_FREE_MODE;
        shoot_cmd_send.friction_mode  = FRICTION_ON;
        shoot_cmd_send.load_mode      = LOAD_STOP;
    } else if (switch_is_up(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_FREE_MODE;
        shoot_cmd_send.friction_mode  = FRICTION_ON;

        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        if (referee_info.GameRobotState.shooter_id1_17mm_cooling_limit - local_heat <= heat_control) // 剩余热量小于留出的热量
        {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    } else {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode     = LOAD_STOP;
    }
    shoot_cmd_send.shoot_rate = 25;

    if (switch_is_mid(rc_data[TEMP].rc.switch_left) && vision_recv_data[8] == 1) // 左侧开关状态为[中],视觉模式
    {
        // 使用绝对角度控制
        // // 将视觉传来的向量转为绝对角度

        static float rec_yaw, rec_pitch;
        memcpy(&rec_yaw, vision_recv_data, sizeof(float));
        memcpy(&rec_pitch, vision_recv_data + 4, sizeof(float));

        gimbal_cmd_send.yaw   = rec_yaw;
        gimbal_cmd_send.pitch = rec_pitch;

        vision_recv_data[8] = 0;

        // float x, y, z;
        // memcpy(&x, vision_recv_data, sizeof(float));
        // memcpy(&y, vision_recv_data + 4, sizeof(float));
        // memcpy(&z, vision_recv_data + 8, sizeof(float));

        // // 对向量进行解析，化为欧拉角
        // yaw   = atan2f(x, z) * 180.0 / M_PI;
        // pitch = asinf(y); // 很离谱，看了下云台PID里pitch用弧度yaw用角度……//对不起。。。。

        // 由于云台是使用多圈角度进行闭环，对于视觉的绝对角度需做些处理
        // float yaw_total_angle = gimbal_fetch_data.gimbal_imu_data->output.Yaw_total_angle_deg;
        // float yaw_total_round = yaw_total_angle / 360.0;
        // yaw                   = yaw + yaw_total_round * 360.0;
        // if (yaw_total_angle - yaw > 180) yaw += 180;
    }
    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data[8] == 0) { // 按照摇杆的输出大小进行角度增量,增益系数需调整
        yaw_control -= 0.0007f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch -= 0.00001f * (float)rc_data[TEMP].rc.rocker_l1;
    }
    YawControlProcess();
    gimbal_cmd_send.yaw = yaw_control;
    // 底盘参数
    chassis_cmd_send.vx = 20.0f * (float)rc_data[TEMP].rc.rocker_r_; // 水平方向
    chassis_cmd_send.vy = 20.0f * (float)rc_data[TEMP].rc.rocker_r1; // 竖直方向

// 云台参数,确定云台控制数据
//     if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式
//     {
#pragma message "Vision AUTOAIM"
    // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
    //     }

    // 云台软件限位
    PitchAngleLimit(); // PITCH限位
}

int Cover_Open_Flag     = 0; // 弹舱打开标志位
int Chassis_Rotate_Flag = 0; // 底盘陀螺标志位
int Shoot_Mode_Flag     = 0; // 发射模式标志位
int Shoot_Mode          = 0; // 发射模式标志位
int Shoot_Run_Flag      = 0; // 摩擦轮标志位
#pragma message "TODO"
int Enable_buff_mode_Flag = 0; // 自瞄开启标志位

/**
 * @brief 键盘设定速度
 *
 */
static void ChassisSpeedSet()
{
    // 底盘移动
    static float current_speed_x = 0;
    static float current_speed_y = 0;
    // 前后移动
    // 防止逃跑时关小陀螺按Ctrl进入慢速模式
    if (((rc_data[TEMP].key[KEY_PRESS].w) && !(rc_data[TEMP].key[KEY_PRESS].ctrl)) || ((rc_data[TEMP].key[KEY_PRESS].w) && (rc_data[TEMP].key[KEY_PRESS].ctrl) && (rc_data[TEMP].key[KEY_PRESS].c))) {
        chassis_cmd_send.vy = (current_speed_y + (CHASSIS_SPEED - current_speed_y) * ramp_calc(&fb_ramp)); // vx方向待测
        ramp_init(&slow_ramp, RAMP_TIME);                                                                  // 2000
    } else if (((rc_data[TEMP].key[KEY_PRESS].s) && !(rc_data[TEMP].key[KEY_PRESS].ctrl)) || ((rc_data[TEMP].key[KEY_PRESS].s) && (rc_data[TEMP].key[KEY_PRESS].ctrl) && (rc_data[TEMP].key[KEY_PRESS].c))) {
        chassis_cmd_send.vy = (current_speed_y + (-CHASSIS_SPEED - current_speed_y) * ramp_calc(&fb_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if ((rc_data[TEMP].key[KEY_PRESS].w) && (rc_data[TEMP].key[KEY_PRESS].ctrl) && !(rc_data[TEMP].key[KEY_PRESS].c)) { // 防止逃跑关小陀螺进入慢速移动
        chassis_cmd_send.vy = (current_speed_y + (1000 - current_speed_y) * ramp_calc(&slow_ramp));
        ramp_init(&fb_ramp, RAMP_TIME);
    } else if ((rc_data[TEMP].key[KEY_PRESS].s) && (rc_data[TEMP].key[KEY_PRESS].ctrl) && !(rc_data[TEMP].key[KEY_PRESS].c)) {
        chassis_cmd_send.vy = (current_speed_y + (-1000 - current_speed_y) * ramp_calc(&slow_ramp));
        ramp_init(&fb_ramp, RAMP_TIME);
    } else {
        chassis_cmd_send.vy = 0;
        ramp_init(&fb_ramp, RAMP_TIME);
    }

    // 左右移动
    if (((rc_data[TEMP].key[KEY_PRESS].a) && !(rc_data[TEMP].key[KEY_PRESS].ctrl)) || ((rc_data[TEMP].key[KEY_PRESS].a) && (rc_data[TEMP].key[KEY_PRESS].ctrl) && (rc_data[TEMP].key[KEY_PRESS].c))) {
        chassis_cmd_send.vx = (current_speed_x + (CHASSIS_SPEED - current_speed_x) * ramp_calc(&lr_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if ((((rc_data[TEMP].key[KEY_PRESS].d) && !(rc_data[TEMP].key[KEY_PRESS].ctrl))) || ((rc_data[TEMP].key[KEY_PRESS].d) && (rc_data[TEMP].key[KEY_PRESS].ctrl) && (rc_data[TEMP].key[KEY_PRESS].c))) {
        chassis_cmd_send.vx = (current_speed_x + (-CHASSIS_SPEED - current_speed_x) * ramp_calc(&lr_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if ((rc_data[TEMP].key[KEY_PRESS].a) && (rc_data[TEMP].key[KEY_PRESS].ctrl) && !(rc_data[TEMP].key[KEY_PRESS].c)) {
        chassis_cmd_send.vx = (current_speed_x + (+1000 - current_speed_x) * ramp_calc(&fb_ramp));
        ramp_init(&lr_ramp, RAMP_TIME);
    } else if ((rc_data[TEMP].key[KEY_PRESS].d) && (rc_data[TEMP].key[KEY_PRESS].ctrl) && !(rc_data[TEMP].key[KEY_PRESS].c)) {
        chassis_cmd_send.vx = (current_speed_x + (-1000 - current_speed_x) * ramp_calc(&fb_ramp));
        ramp_init(&lr_ramp, RAMP_TIME);
    } else {
        chassis_cmd_send.vx = 0;
        ramp_init(&lr_ramp, RAMP_TIME);
    }

    current_speed_x = chassis_cmd_send.vx;
    current_speed_y = chassis_cmd_send.vy;
}

/**
 * @brief 鼠标移动云台
 *
 */
static void GimbalSet()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    if ((rc_data[TEMP].key[KEY_PRESS].ctrl) && !(rc_data[TEMP].key[KEY_PRESS].c) && !(rc_data[TEMP].key[KEY_PRESS].v) && !(rc_data[TEMP].key[KEY_PRESS].x)) {
        yaw_control -= rc_data[TEMP].mouse.x / 3500.0f;
        gimbal_cmd_send.pitch -= -rc_data[TEMP].mouse.y / 75000.0f;
    } else {
        yaw_control -= rc_data[TEMP].mouse.x / 200.0f;
        gimbal_cmd_send.pitch -= -rc_data[TEMP].mouse.y / 15000.0f;
    }
    YawControlProcess();
    gimbal_cmd_send.yaw = yaw_control;
}

// 底盘状态标志位
typedef enum {
    CHASSIS_STATUS_ROTATE = 0,
    CHASSIS_STATUS_FOLLOW,

} Chassis_Status_Enum;
// 底盘状态（按键用）
Chassis_Status_Enum Chassis_Status = CHASSIS_STATUS_FOLLOW;

static void SetChassisMode()
{
    if (Chassis_Rotate_Flag > 20) {
        Chassis_Status      = CHASSIS_STATUS_ROTATE;
        Chassis_Rotate_Flag = 0;
    } else if (Chassis_Rotate_Flag < -20) {
        Chassis_Status      = CHASSIS_STATUS_FOLLOW;
        Chassis_Rotate_Flag = 0;
    }

    if (Chassis_Status == CHASSIS_STATUS_ROTATE) {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    } else if (Chassis_Status == CHASSIS_STATUS_FOLLOW) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
}

/**
 * @brief 机器人复位函数，按下Ctrl+Shift+r
 *
 *
 */

static void RobotReset()
{
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].r) {
        osDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset(); // 软件复位
    }
}

/**
 * @brief 键鼠设定机器人发射模式
 *
 */
static void SetShootMode()
{
    // V按下超过100ms，开启摩擦轮，清空标志位
    // 其他同理
    if (Shoot_Run_Flag > 10) { // 20*5 = 100ms
        shoot_cmd_send.friction_mode = FRICTION_ON;
        Shoot_Run_Flag               = 0;
    } else if (Shoot_Run_Flag < -10) {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        Shoot_Run_Flag               = 0;
    }
    // 按H键更改发射模式，目前为1:单发  2:连发
    if (rc_data[TEMP].key[KEY_PRESS].h) {
        Shoot_Mode_Flag++;
    } else {
        Shoot_Mode_Flag = 0;
    }
    if (Shoot_Mode_Flag > 10) {
        if (Shoot_Mode == 0) {
            Shoot_Mode = 1;
        } else {
            Shoot_Mode = 0;
        }
    }

    // 仅在摩擦轮开启时有效
    if (shoot_cmd_send.friction_mode == FRICTION_ON) {
        // 打弹，单击左键单发，长按连发
        shoot_cmd_send.shoot_rate = 25;
        if (rc_data[TEMP].mouse.press_l) {

            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        } else {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    }
    // 新热量管理
    if (referee_info.GameRobotState.shooter_id1_17mm_cooling_limit - local_heat <= heat_control) // 剩余热量小于留出的热量
    {
        if (!(rc_data[TEMP].key->r)) // 按着R则不限制热量
        {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    }
}

/**
 * @brief 键盘处理模式标志位
 *
 */

static void KeyGetMode()
{
    // V键开启摩擦轮，Ctrl+V关闭摩擦轮
    if (rc_data[TEMP].key[KEY_PRESS].v && !(rc_data[TEMP].key[KEY_PRESS].ctrl)) {
#pragma message "TODO"

        Shoot_Run_Flag++;
    } else if (rc_data[TEMP].key[KEY_PRESS].v && (rc_data[TEMP].key[KEY_PRESS].ctrl)) {
        Shoot_Run_Flag--;
    } else {
        Shoot_Run_Flag = 0;
    }
    if (rc_data[TEMP].key[KEY_PRESS].ctrl) {
        Send_Once_Flag = 0; // UI重新发送
    }
    // C键开启小陀螺，Ctrl+C停止
    if ((rc_data[TEMP].key[KEY_PRESS].c) && !(rc_data[TEMP].key[KEY_PRESS].ctrl)) {
        Chassis_Rotate_Flag++;
    } else if (rc_data[TEMP].key[KEY_PRESS].c && (rc_data[TEMP].key[KEY_PRESS].ctrl)) {

        Chassis_Rotate_Flag--;
    } else {
        Chassis_Rotate_Flag = 0;
    }
}

static void SuperCapMode()
{
    if (rc_data[TEMP].key[KEY_PRESS].shift) {
        Super_flag = SUPER_OPEN;
    } 
    else{
        Super_flag = SUPER_CLOSE;
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 * @todo 缺少打符模式（是否必要？）
 *
 */
static void MouseKeySet()
{
    ChassisSpeedSet();
    GimbalSet();
    KeyGetMode();
    SetShootMode();
    SetChassisMode();
    SuperCapMode();
    // SetGimbalMode();
    PitchAngleLimit();
    RobotReset(); // 机器人复位处理
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state                   = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode     = SHOOT_OFF;
        shoot_cmd_send.friction_mode  = FRICTION_OFF;
        shoot_cmd_send.load_mode      = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[中],恢复正常运行
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
        robot_state               = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/**
 * @brief 更新UI数据
 */
void UpDateUI()
{
    // 更新UI数据
    Referee_Interactive_info.chassis_mode  = chassis_cmd_send.chassis_mode;
    Referee_Interactive_info.gimbal_mode   = gimbal_cmd_send.gimbal_mode;
    Referee_Interactive_info.friction_mode    = shoot_cmd_send.friction_mode;
    Referee_Interactive_info.shoot_mode     = shoot_cmd_send.shoot_mode;
    Referee_Interactive_info.lid_mode      = shoot_cmd_send.lid_mode;
    // Referee_Interactive_info.Chassis_Power_Data = ; 暂时没有，等待移植

    // 保存上一次的UI数据
    Referee_Interactive_info.chassis_last_mode       = Referee_Interactive_info.chassis_mode;
    Referee_Interactive_info.gimbal_last_mode        = Referee_Interactive_info.gimbal_mode;
    Referee_Interactive_info.shoot_last_mode         = Referee_Interactive_info.shoot_mode;
    Referee_Interactive_info.friction_last_mode      = Referee_Interactive_info.friction_mode;
    Referee_Interactive_info.lid_last_mode           = Referee_Interactive_info.lid_mode;
    Referee_Interactive_info.Chassis_last_Power_Data = Referee_Interactive_info.Chassis_Power_Data;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠

    if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();
    else if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right)) {
        RemoteControlSet();
        // 之后删除_急停
        robot_state                   = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        // shoot_cmd_send.shoot_mode     = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode     = LOAD_STOP;
    } else {
        RemoteControlSet();
    }

    UpDateUI();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
    remote_work_condition = RemoteControlIsOnline();

    if (remote_work_condition == 0) {
        robot_state                   = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode     = SHOOT_OFF;
        shoot_cmd_send.friction_mode  = FRICTION_OFF;
        shoot_cmd_send.load_mode      = LOAD_STOP;
    }
    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    static uint8_t frame_head[] = {0xAF, 0x32, 0x00, 0x10};
    memcpy(vision_send_data, frame_head, 4);

    memcpy(vision_send_data + 4, gimbal_fetch_data.gimbal_imu_data->INS_data.INS_quat, sizeof(float) * 4);
    vision_send_data[20] = 0;
    for (size_t i = 0; i < 20; i++)
        vision_send_data[20] += vision_send_data[i];
    HostSend(host_instance, vision_send_data, 21);
}
