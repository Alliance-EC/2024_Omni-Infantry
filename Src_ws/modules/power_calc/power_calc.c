/**
 * @file power_calc.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief Refer to DynamicX open source solution
 * @version 0.1
 * @date 2024-11-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "power_calc.h"

#include <math.h>

static PowerCalcInstance powercalcinstance;

static float limit_output(float *val, float min, float max)
{
    if (*val > max) {
        *val = max;
    } else if (*val < min) {
        *val = min;
    }
    return *val;
}

void power_calc_params_init(float reduction_ratio_init, bool output_direction_init)
{
    powercalcinstance.current_coef  = 1.502f;
    powercalcinstance.velocity_coef = 0.005f;

    powercalcinstance.output_direction = output_direction_init;

    powercalcinstance.reduction_ratio = reduction_ratio_init;

    powercalcinstance.torque_current_coefficient = CMD_2_CURRENT * TORQUE_COEFFICIENT * (REDUCTION_RATIO_OF_DJI / reduction_ratio_init);
    powercalcinstance.static_consumption         = 1.0f;
}

void max_power_update(uint16_t max_power_init)
{
    powercalcinstance.max_power = max_power_init;
}

// p=t*w(b)+k1*w2(c)+k2*t2(a)+P_s_c
float current_output_calc(volatile Power_Data_s *motors_data)
{
    float a = 0, b = 0, c = 0;
    for (uint8_t motor_id = 0; motor_id < 4; motor_id++) {
        motors_data->cmd_torque[motor_id] = motors_data->cmd_current[motor_id] * powercalcinstance.torque_current_coefficient;
        a += powercalcinstance.current_coef * powf(motors_data->cmd_torque[motor_id], 2.);
        b += motors_data->cmd_torque[motor_id] * motors_data->wheel_velocity[motor_id];
        c += powercalcinstance.velocity_coef * powf(motors_data->wheel_velocity[motor_id], 2.) + powercalcinstance.static_consumption;
    }

    powercalcinstance.calc_power = a + b + c;

    c -= powercalcinstance.max_power;

    powercalcinstance.zoom_coef = powercalcinstance.calc_power < powercalcinstance.max_power ? 1.0 : (b * b - 4 * a * c) > 0 ? (-b + sqrtf(b * b - 4 * a * c)) / (2 * a)
                                                                                                                             : 0.;
    return powercalcinstance.zoom_coef;
    // return limit_output(&powercalcinstance.zoom_coef, 0.0, 1.0);
}
