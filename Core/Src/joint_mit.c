#include "joint_mit.h"

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float raw_rel_to_q(const JointMIT *j, int32_t raw_rel) {
    float norm = (float)raw_rel / j->raw_half_range; // roughly [-1..1]
    norm = clampf(norm, -1.2f, +1.2f);
    return norm * j->q_max_rad;
}

void joint_mit_init(JointMIT *j)
{
    // defaults safe
    j->zero_valid = 0;
    j->q = j->q_prev = 0.0f;
    j->qd = j->qd_f = 0.0f;
    j->q_des = 0.0f;
    j->qd_des = 0.0f;
    j->kp = 2.0f;
    j->kd = 0.05f;
    j->tau_ff = 0.0f;
    j->tau = 0.0f;
    j->tau_max = 0.30f;

    // mapping / soft limits (你按你项目填)
    j->raw_min = 5500;
    j->raw_max = 10500;
    j->margin_raw = 200;
    j->k_soft = 0.002f;
    j->q_max_rad = 1.0f;
    j->raw_half_range = (float)((j->raw_max - j->raw_min) / 2); // 2500
    j->torque_sign = +1; // 这里就是“方向开关”
}

void joint_mit_zero_here(JointMIT *j, int32_t raw_now)
{
    j->raw_zero = raw_now;
    j->zero_valid = 1;
    j->q = j->q_prev = 0.0f;
    j->qd = j->qd_f = 0.0f;
    j->q_des = 0.0f;   // 上电保持
    j->qd_des = 0.0f;
}

void joint_mit_step_1khz(JointMIT *j, int32_t raw_abs)
{
    j->raw_abs = raw_abs;

    if (!j->zero_valid) {
        j->tau = 0.0f;
        return;
    }

    // 1) raw rel
    j->raw_rel = j->raw_abs - j->raw_zero;

    // 2) q / qd
    j->q = raw_rel_to_q(j, j->raw_rel);

    const float dt = 0.001f;
    float qd_raw = (j->q - j->q_prev) / dt;
    j->q_prev = j->q;

    // low-pass on qd
    const float alpha = 0.1f;
    j->qd_f = j->qd_f + alpha * (qd_raw - j->qd_f);
    j->qd = j->qd_f;

    // 3) MIT torque
    float tau = j->kp * (j->q_des - j->q)
              + j->kd * (j->qd_des - j->qd)
              + j->tau_ff;

    // 4) soft limit (用 raw_abs 做边界判断，和 q 方向无关)
    if (j->raw_abs < (j->raw_min + j->margin_raw)) {
        float x = (float)((j->raw_min + j->margin_raw) - j->raw_abs);
        tau += j->k_soft * x;
    } else if (j->raw_abs > (j->raw_max - j->margin_raw)) {
        float x = (float)(j->raw_abs - (j->raw_max - j->margin_raw));
        tau -= j->k_soft * x;
    }

    // 5) clamp + direction
    tau = clampf(tau, -j->tau_max, +j->tau_max);
    j->tau = (float)j->torque_sign * tau;
}
