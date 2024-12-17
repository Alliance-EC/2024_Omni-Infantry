#pragma once

#include <stdint.h>

#define RAMP_TIME 1500

typedef struct ramp_t {
    int32_t count; // 计数值
    int32_t scale; // 规模
    float out;     // 输出
} ramp_t;

void ramp_init(ramp_t *ramp, int32_t scale);

float ramp_calc(ramp_t *ramp);
