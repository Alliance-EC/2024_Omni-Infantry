#pragma once

#include "robot_def.h"
#include "ramp.h"
#include <stdint.h>

#define RC_LOST         (rc_data[TEMP].rc.switch_left == 0 && rc_data[TEMP].rc.switch_right == 0)
#define MOUSEKEYCONTROL switch_is_up(rc_data[TEMP].rc.switch_left) && (switch_is_down(rc_data[TEMP].rc.switch_right))
#define ENTIREDISABLE   (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))
// 底盘模式
#define CHASSIS_FREE     0
#define CHASSIS_ROTATION 1
#define CHASSIS_FOLLOW   2
#define SHOOT_FRICTION   3
#define SHOOT_LOAD       4

typedef struct {
    /*控制值*/
    uint8_t ui_refresh_flag; // UI发送标志位

    uint8_t auto_aim;
    uint8_t auto_rune; // 自瞄打符标志位
    float rec_yaw, rec_pitch;

    float yaw_control;   // 遥控器YAW自由度输入值
    float pitch_control; // 遥控器PITCH自由度输入值
    float heat_coef;
    ramp_t fb_ramp;
    ramp_t lr_ramp;

    chassis_mode_e last_chassis_mode_;
    friction_mode_e last_fric_mode_;
    loader_mode_e last_load_mode_;
    bullet_bay_mode_e last_bay_mode_;

    uint8_t emerg_handle_log_cnt;
} CmdInstance;

/* 初始化 */
/**
 * @brief cmd设备初始化，遥控器，裁判系统
 *
 */
void CmdDeviceInit();

/**
 * @brief cmd中介变量初始化
 *
 */
void CmdParamInit();

/**
 * @brief cmd消息收发初始化
 *
 */
void CmdMsgInit();

/*cmd任务*/
/**
 * @brief  裁判系统判断各种ID，选择客户端ID
 * @retval none
 * @attention
 */
void DeterminRobotID();

/**
 * @brief 自瞄手瞄切换
 *
 */
void GimbalModeSwitch();

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
void CalcOffsetAngle();

/**
 * @brief 发射启动、热量控制
 *
 */
void ShootControl();

/**
 * @brief cmd模式切换
 *
 */
void CmdModeSet();

/**
 * @brief cmd消息收发
 *
 */
void CmdMsgComm();
