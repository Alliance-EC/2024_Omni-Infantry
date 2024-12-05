#pragma once

#include <stdint.h>
#include "ramp.h"

#define LOADER_2006       1

#define MAX_HISTROY       10
#define Fliter_windowSize 5.0f
#define BLOCK_CURRENT     10000
#define REVERSE_CURRENT   4000

/* 拨弹盘工作状态 */
typedef enum {
    LOADER_START = 0,
    LOADER_IDLE,
    LOADER_JAM,
    LOADER_ROLLBACK,
} loader_status_e;

typedef struct {

    int heat_control;     // 热量控制
    float local_heat;     // 本地热量
    int One_bullet_heat;  // 打一发消耗热量
    uint32_t shoot_count; // 已发弹量

    float loader_velocity;         // 当前电机转速
    float loader_current;          // 电机电流值
    loader_status_e loader_status; // 拨弹盘状态
    float loader_rate;             // 拨弹盘转速

    uint8_t one_bullet;
    ramp_t fric_on_ramp, fric_off_ramp;
    float fric_speed_ref; // 摩擦轮转速参考值
    uint32_t shoot_heat_count[2];
    float current_fric_speed;
} ShootInstance;

/**
 * @brief 发射设备初始化
 *
 */
void ShootDeviceInit();

/**
 * @brief 发射消息收发初始化
 *
 */
void ShootMsgInit();

/**
 * @brief 发射参数初始化
 *
 */
void ShootParamInit();

/**
 * @brief 堵转检测
 *
 */
void loader_status_update();

/**
 * @brief 发射状态切换
 *
 */
void ShootModeSet();

/**
 * @brief 发射消息收发
 *
 */
void ShootMsgComm();
