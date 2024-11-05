/**
 * @file chassis.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "chassis.h"
#include "chassis_controller.h"

void ChassisInit()
{
    ChassisDeviceInit();
    ChassisParamInit();
    ChassisMsgInit();
}

void ChassisTask()
{
    ChassisModeSet();
    OmniCalculate();
    ChassisMsgComm();
}