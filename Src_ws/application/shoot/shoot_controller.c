/**
 * @file shoot_controller.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "shoot_controller.h"
#include "robot_def.h"

#include "servo_motor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "ramp.h"
#include <stdint.h>

static Publisher_t *shoot_pub;
static Subscriber_t *shoot_sub;

static Shoot_Ctrl_Cmd_s shoot_cmd_recv;         // 来自cmd的发射控制信息
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

static DJIMotorInstance *friction_l, *friction_r, *loader;
static ServoInstance *bullet_bay_cover; // 弹舱盖舵机

static ShootInstance shoot_media_param; // 发射中介变量

void ShootDeviceInit()
{
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut        = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id                             = 3;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id                             = 4;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r                                                        = DJIMotorInit(&friction_config);

    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut        = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006};
    loader = DJIMotorInit(&loader_config);

    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
    DJIMotorStop(loader);

    Servo_Init_Config_s servo_config_init = {
        .htim             = &htim1,
        .Channel          = TIM_CHANNEL_1,
        .Servo_Angle_Type = init_mode,
        .Servo_type       = Servo270,
    };
    bullet_bay_cover = ServoInit(&servo_config_init);
}

void ShootMsgInit()
{
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

void ShootParamInit()
{
    int16_t init_angle  = 100;
    int16_t final_angle = 0;
    servo_init_end_angle_set(bullet_bay_cover, init_angle, final_angle);

    shoot_media_param.heat_control    = 25; // 热量控制
    shoot_media_param.local_heat      = 0;  // 本地热量
    shoot_media_param.One_bullet_heat = 10; // 打一发消耗热量
    shoot_media_param.shoot_count     = 0;  // 已发弹量

    ramp_init(&shoot_media_param.fric_on_ramp, 200);
    ramp_init(&shoot_media_param.fric_off_ramp, 200);
}

void loader_status_update(void)
{
    static float load_starting_count_ = 0;
    static float load_jaming_count_   = 0;
    static float load_rollback_count_ = 0;
    // 获取拨弹盘转速
    shoot_media_param.loader_velocity = loader->measure.speed_aps * NUM_PER_CIRCLE /
                                        REDUCTION_RATIO_LOADER / 360.0;

    switch (shoot_media_param.loader_status) {
        case LOADER_START:
            load_rollback_count_ = 0;
            if (shoot_media_param.loader_velocity >= 20)
                load_starting_count_++;
            else if (abs(shoot_media_param.loader_current) > 8000)
                load_jaming_count_++;

            if (load_starting_count_ >= 500) {
                shoot_media_param.loader_status = LOADER_IDLE;
                load_starting_count_            = 0;
            }
            if (load_jaming_count_ >= 500) {
                shoot_media_param.loader_status = LOADER_ROLLBACK;
                load_jaming_count_              = 0;
            }
            break;
        case LOADER_IDLE:
            if (shoot_media_param.loader_velocity <= 10 && shoot_media_param.loader_current > 8000)
                shoot_media_param.loader_status = LOADER_ROLLBACK;
            break;
        case LOADER_ROLLBACK:
            shoot_cmd_recv.load_mode = LOAD_REVERSE;
            if (load_rollback_count_++ > 200) {
                shoot_media_param.loader_status = LOADER_START;
            }
            break;
        default:
            shoot_media_param.loader_status = LOADER_START;
            break;
    }
}

void ShootModeSet()
{
    switch (shoot_cmd_recv.shoot_mode) {
        case SHOOT_OFF:
            DJIMotorStop(friction_l);
            DJIMotorStop(friction_r);
            DJIMotorStop(loader);
            break;
        case SHOOT_ON:
            DJIMotorEnable(friction_l);
            DJIMotorEnable(friction_r);
            DJIMotorEnable(loader);

            break;
    }
    Servo_Motor_Type_Select(bullet_bay_cover, free_mode);
    switch (shoot_cmd_recv.bay_mode) {
        case BAY_CLOSE:
            Servo_Motor_Type_Select(bullet_bay_cover, init_mode);

            break;
        case BAY_OPEN:
            Servo_Motor_Type_Select(bullet_bay_cover, end_mode);

            break;
    }

    switch (shoot_cmd_recv.load_mode) {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorSetRef(loader, 0);
            shoot_media_param.shoot_heat_count[0] = shoot_media_param.shoot_count;
            shoot_media_param.shoot_heat_count[1] = shoot_media_param.shoot_heat_count[0];
            shoot_media_param.one_bullet          = 0;
            break;
        // 激活能量机关
        case LOAD_SINGLE:
            shoot_media_param.shoot_heat_count[1] = shoot_media_param.shoot_count;
            if (shoot_media_param.shoot_heat_count[1] - shoot_media_param.shoot_heat_count[0] >= 1) {
                shoot_media_param.one_bullet = 1;
            }
            switch (shoot_media_param.one_bullet) {
                case 1:
                    DJIMotorSetRef(loader, 0);
                    break;
                case 0:
                    DJIMotorSetRef(loader, shoot_cmd_recv.loader_rate / 3);
                    break;
            }
            break;
        // 连发模式
        case LOAD_BURSTFIRE:
            DJIMotorSetRef(loader, shoot_cmd_recv.loader_rate);
            break;
        case LOAD_REVERSE:
            DJIMotorSetRef(loader, -shoot_cmd_recv.loader_rate / 2);
            break;
        default:
            while (1);
    }

    switch (shoot_cmd_recv.friction_mode) {
        case FRICTION_OFF:
            shoot_media_param.fric_speed_ref = (shoot_media_param.current_fric_speed + (0 - shoot_media_param.current_fric_speed) * ramp_calc(&shoot_media_param.fric_off_ramp));
            ramp_init(&shoot_media_param.fric_on_ramp, 300);
            break;
        case FRICTION_ON:
            shoot_media_param.fric_speed_ref = (shoot_media_param.current_fric_speed + (37700 - shoot_media_param.current_fric_speed) * ramp_calc(&shoot_media_param.fric_on_ramp));
            ramp_init(&shoot_media_param.fric_off_ramp, 300);
            break;
    }

    shoot_media_param.current_fric_speed = shoot_media_param.fric_speed_ref;

    DJIMotorSetRef(friction_l, shoot_media_param.fric_speed_ref);
    DJIMotorSetRef(friction_r, shoot_media_param.fric_speed_ref);
}

void ShootMsgComm()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 反馈数据
    memcpy(&shoot_feedback_data.shooter_local_heat, &shoot_media_param.local_heat, sizeof(float));
    memcpy(&shoot_feedback_data.shooter_heat_control, &shoot_media_param.heat_control, sizeof(int));

    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

/**
 * @brief 拨弹盘电流均值滤波
 *
 *
 */
static float loader_cunrrent_mean_filter(void)
{
    static float current_data[MAX_HISTROY] = {0};           // 初始化为0
    static uint8_t head = 0, rear = 0;                      // 队列下标
    static float window_sum = 0;                            // 窗口中所有元素的和
    uint8_t data_count      = 0;                            // 数据量
    float motor_current     = loader->measure.real_current; // 获取拨弹盘电流
    float filter_current    = 0;                            // 滤波后的电流

    current_data[head] = motor_current;
    head               = (head + 1) % MAX_HISTROY;
    data_count         = (head - rear) % MAX_HISTROY;
    if (data_count >= Fliter_windowSize) {
        window_sum = 0;
        for (uint8_t i = rear, index = 0; index < Fliter_windowSize; i++, index++) {
            i %= MAX_HISTROY;
            window_sum += current_data[i];
        }
        filter_current = window_sum / Fliter_windowSize;
        rear++;
        rear %= MAX_HISTROY;
    }

    return filter_current;
}

/**
 * @brief 摩擦轮检测记录发弹量
 *
 */
float watch;
static void shoot_Fric_data_process(void)
{
    /*----------------------------------变量常量------------------------------------------*/
    static bool bullet_waiting_confirm = false;                         // 等待比较器确认
    float data                         = friction_l->measure.speed_aps; // 获取摩擦轮转速
    static uint16_t data_histroy[MAX_HISTROY];                          // 做循环队列
    static uint8_t head = 0, rear = 0;                                  // 队列下标
    float moving_average[2];                                            // 移动平均滤波
    uint8_t data_num;                                                   // 循环队列元素个数
    float derivative;                                                   // 微分
    /*-----------------------------------逻辑控制-----------------------------------------*/
    data = abs(data);
    /*入队*/
    data_histroy[head] = data;
    head++;
    head %= MAX_HISTROY;
    /*判断队列数据量*/
    data_num = (head - rear + MAX_HISTROY) % MAX_HISTROY;
    if (data_num >= Fliter_windowSize + 1) // 队列数据量满足要求
    {
        moving_average[0] = 0;
        moving_average[1] = 0;
        /*同时计算两个滤波*/
        for (uint8_t i = rear, j = rear + 1, index = rear; index < rear + Fliter_windowSize; i++, j++, index++) {
            i %= MAX_HISTROY;
            j %= MAX_HISTROY;
            moving_average[0] += data_histroy[i];
            moving_average[1] += data_histroy[j];
        }
        moving_average[0] /= Fliter_windowSize;
        moving_average[1] /= Fliter_windowSize;
        /*滤波求导*/
        derivative = moving_average[1] - moving_average[0];
        watch      = derivative;
        /*导数比较*/
        if (derivative < -220) {
            bullet_waiting_confirm = true;
        } else if (derivative > 30) {
            if (bullet_waiting_confirm == true) {
                shoot_media_param.local_heat += shoot_media_param.One_bullet_heat; // 确认打出
                shoot_media_param.shoot_count++;
                bullet_waiting_confirm = false;
            }
        }
        rear++;
        rear %= MAX_HISTROY;
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM6) {
        /*-------------------------------------------热量控制部分---------------------------------------------*/
        shoot_media_param.local_heat -= (shoot_cmd_recv.shooter_heat_cooling_rate / 1000.0f); // 1000Hz冷却
        if (shoot_media_param.local_heat < 0) {
            shoot_media_param.local_heat = 0;
        }
        if (shoot_cmd_recv.shooter_referee_heat - shoot_cmd_recv.shooter_cooling_limit >= 15) // 裁判系统判断已经超了热量
        {
            shoot_media_param.local_heat = shoot_cmd_recv.shooter_referee_heat;
        }
        shoot_Fric_data_process();
        shoot_media_param.loader_current = loader_cunrrent_mean_filter();
    }
    /* USER CODE END Callback 1 */
}