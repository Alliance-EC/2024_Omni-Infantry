#pragma once

#include <stdint.h>
// #include "motor_def.h"
// #include "controller.h"
#include "ramp.h"

#define LF        0
#define RF        1
#define RB        2
#define LB        3

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

typedef struct {
    uint8_t center_gimbal_offset_x; // 云台旋转中心距底盘几何中心的距离
    uint8_t center_gimbal_offset_y;

    /* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
    float chassis_vx, chassis_vy, chassis_vw; // 将云台系的速度投影到底盘
    float vt_lf, vt_rf, vt_lb, vt_rb;         // 底盘速度解算后的临时输出,待进行限幅

    /*模式参数 */
    float offset_angle;
    float sin_theta, cos_theta;
    ramp_t *rotate_ramp;
    float current_speed_vw;
} ChassisInstance;

/**
 * @brief 底盘设备初始化
 *
 */
void ChassisDeviceInit();

/**
 * @brief 底盘中间参数初始化
 *
 */
void ChassisParamInit();
/**
 * @brief 底盘消息初始化,注册订阅者和发布者
 *
 */
void ChassisMsgInit();

/**
 * @brief 底盘模式设置
 *
 */
void ChassisModeSet();

/**
 * @brief 计算每个轮毂电机的输出,全向轮正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
void OmniCalculate();

/**
 * @brief 底盘消息收发
 *
 */
void ChassisMsgComm();
