#pragma once
#include <stdint.h>

typedef struct {
    // raw
    int32_t raw_abs;
    int32_t raw_zero;
    int32_t raw_rel;

    // state
    float q;      // rad
    float q_prev;
    float qd;     // rad/s (filtered)
    float qd_f;

    // cmd
    float q_des;
    float qd_des;
    float kp;
    float kd;
    float tau_ff;

    // out
    float tau;
    float tau_max;
    uint8_t zero_valid;

    // config (soft limits)
    int32_t raw_min;
    int32_t raw_max;
    int32_t margin_raw;
    float   k_soft;
    float   q_max_rad;      // map scale
    float   raw_half_range; // half range for mapping
    int8_t  torque_sign;    // +1 or -1 (用来一键翻转方向)
} JointMIT;

void joint_mit_init(JointMIT *j);
void joint_mit_zero_here(JointMIT *j, int32_t raw_now);

// update state and compute tau at 1kHz
void joint_mit_step_1khz(JointMIT *j, int32_t raw_abs);
