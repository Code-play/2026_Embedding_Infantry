#include "gimbal.h"
#include "Serialplot.h"
#include "chassis.h"
#include "trajectory_follower.h"

extern Serialplot serialplot;

//调试参数
float yaw_torque_scale = 0.001f;
float vision_yaw_torque_scale = 0.001f;
float yaw_startup_negative_bias_nm = -0.46105f;
float yaw_startup_positive_bias_nm = 0.4690f;
const fp32 yaw_output_torque_limit_nm = 4.0f;
const fp32 yaw_mit_manual_damping_kd = 0.08f;
const fp32 yaw_mit_vision_far_damping_kd = 0.04f;
const fp32 yaw_mit_vision_near_damping_kd = 0.30f;
const fp32 yaw_mit_vision_damping_near_error_rad = 0.005f;
const fp32 yaw_mit_vision_damping_far_error_rad = 0.050f;
const fp32 yaw_startup_speed_deadband = 0.05f;
const fp32 yaw_startup_torque_deadband = 0.01f;
const fp32 yaw_bias_fade_error_zero_rad = 0.0002f;
const fp32 yaw_bias_fade_error_hold_rad = 0.001f;
const fp32 yaw_bias_fade_error_knee_rad = 0.01f;
const fp32 yaw_bias_fade_error_full_rad = 0.02f;
const fp32 yaw_bias_fade_min_scale = 0.25f;
const fp32 yaw_bias_fade_knee_scale = 0.80f;
const fp32 yaw_bias_hold_enter_error_rad = 0.001f;
const fp32 yaw_bias_hold_exit_error_rad = 0.003f;
const fp32 yaw_bias_hold_speed_radps = 0.03f;
const fp32 vision_yaw_static_bias_nm = 0.0f;
const fp32 vision_yaw_static_enter_error_rad = 0.010f;
const fp32 vision_yaw_static_full_error_rad = 0.030f;
const fp32 vision_yaw_static_exit_speed_radps = 1.0f;
const fp32 vision_yaw_settled_enter_error_rad = 0.004f;
const fp32 vision_yaw_settled_exit_error_rad = 0.010f;
const uint8_t vision_yaw_settled_enter_cycles = 6U;
const fp32 vision_yaw_delta_deadband_rad = 0.0015f;
const fp32 vision_yaw_delta_filter_tau_s = 0.006f;
const fp32 vision_yaw_torque_boost_enter_error_rad = 0.006f;
const fp32 vision_yaw_torque_boost_full_error_rad = 0.030f;
const fp32 vision_yaw_breakaway_enter_error_rad = 0.006f;
const fp32 vision_yaw_breakaway_full_error_rad = 0.040f;
const fp32 vision_yaw_breakaway_min_torque_nm = 0.30f;
const fp32 vision_yaw_breakaway_max_torque_nm = 0.65f;
const fp32 vision_yaw_breakaway_exit_speed_radps = 2.5f;
const fp32 vision_yaw_direction_deadband_rad = 0.001f;
const fp32 top_yaw_ff_anchor_omega_radps = 10.0f;
const fp32 top_yaw_ff_anchor_torque_nm = 0.65f;
const fp32 top_yaw_ff_omega_gain = 0.0125f;
const fp32 top_yaw_ff_enable_omega_radps = 1.0f;
const fp32 top_yaw_ff_max_torque_nm = 1.0f;
const fp32 top_yaw_ff_filter_tau_s = 0.03f;
const fp32 top_yaw_ff_error_hold_rad = 0.004f;
const fp32 top_yaw_ff_error_backoff_rad = 0.02f;
const fp32 top_yaw_ff_min_scale = 1.0f;   /* 折扣按误差以 1x 调制前馈 0.2~0.45 N·m，与「确定性前馈」前提冲突 */

/* ---- 周期前馈：小陀螺每转一次的扰动抑制（docs/periodic-disturbance-fix-idea.md 方案 A）
 * 相位参考用**云台相对角** theta_rel（不是时间）—— 自旋变速时频率自动跟随。
 * tau_ff(theta) = a1*sin(t) + b1*cos(t) + a2*sin(2t) + b2*cos(2t) + a3*sin(3t) + b3*cos(3t)
 * 这些文件级 const fp32 已被 fw/params_whitelist.json 的 file_scope_consts 纳入可调参数，
 * 仿真里 fw_param_set 可在线标定；真机验证通过后把标定值写成本文件默认值再烧录。
 * 判据：小陀螺稳态 yaw 误差 1x 谐波幅值下降 >70%（真机 FFT 复测）。
 */
const fp32 top_periodic_ff_enable = 1.0f;
const fp32 top_periodic_ff_max_torque_nm = 1.0f;
const fp32 top_periodic_ff_fade_tau_s = 0.0f;   /* 标定值按「系数=施加力矩」反解，0.1 s 低通会让高阶偏差 40%+ */
const fp32 top_periodic_ff_phase_offset_rad = 0.0f;
/* 2026-10-25 现场标定（exp_2026-10-25_ff_coef_calib）：
 *   1x D=0.3274 N·m∠-34.7° -> a1,b1；3x D=0.0456∠-130.7° -> a3,b3
 *   2x 标定值实测把误差放大到 0.393 deg（基线 0.069），故归零，等在工作点上重新辨识。
 *   v4 的 1x 相位偏 42°，2x/3x 原本无前馈。 */
/* v7 候选：四阶周期前馈全部在线自适应；初值取 v6 实测收敛值 */
const fp32 top_periodic_ff_a1 = -0.2613f;
const fp32 top_periodic_ff_b1 = +0.1626f;
const fp32 top_periodic_ff_a2 = -0.0176f;
const fp32 top_periodic_ff_b2 = +0.0231f;
const fp32 top_periodic_ff_a3 = +0.0095f;
const fp32 top_periodic_ff_b3 = +0.0296f;

/* ==== 四阶在线自适应 + 软启动斜坡（test-only）==== */
const fp32 top_ff_adapt_gamma = 0.15f;
const fp32 top_ff_adapt_g_mag[4] = {3.34f, 4.36f, 3.22f, 2.80f};
const fp32 top_ff_adapt_g_phase[4] = {-2.857f, 2.373f, 1.410f, 0.366f};
const fp32 top_ff_adapt_limit_nm[4] = {0.6f, 0.6f, 0.6f, 0.15f};
const fp32 top_ff_adapt_min_err_deg = 0.02f;
const fp32 top_ff_adapt_guard_err_rad = 0.05f;
const fp32 top_ff_ramp_s = 0.15f;
fp32 top_ff_dyn_a[4] = {-0.2613f, -0.0176f, +0.0095f, 0.0f};
fp32 top_ff_dyn_b[4] = {+0.1626f, +0.0231f, +0.0296f, 0.0f};
fp32 top_ff_ramp_gain = 0.0f;
static fp32 top_ff_adapt_acc_th = 0.0f;
static fp32 top_ff_adapt_acc_s[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static fp32 top_ff_adapt_acc_c[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static fp32 top_ff_adapt_prev_th = 0.0f;
static uint8_t top_ff_adapt_init = 0U;
static uint8_t yaw_bias_hold_latched = 0U;
static uint8_t vision_yaw_settled_latched = 0U;
static uint8_t vision_yaw_settled_cycles = 0U;
static fp32 vision_yaw_control_delta_filtered_rad = 0.0f;
static uint8_t vision_yaw_delta_filter_initialized = 0U;
static fp32 top_yaw_ff_filtered_relative_omega = 0.0f;
static uint8_t top_yaw_ff_filter_initialized = 0U;
static fp32 top_periodic_ff_filtered_torque = 0.0f;
static uint8_t top_periodic_ff_initialized = 0U;
static const fp32 vision_yaw_release_weights[VISION_YAW_RELEASE_SLOTS] = {0.25f, 0.25f, 0.25f, 0.25f};
static const fp32 vision_yaw_ff_torque_max = 0.0f;
static const fp32 vision_pitch_ff_torque_max = 0.18f;
static const fp32 vision_pitch_vel_ff_max = 8.0f;

const fp32 pitch_friction_comp_nm = 0.15f;   // 摩擦补偿幅值，从 0.1 起调
const fp32 pitch_friction_speed_deadband = 0.05f; // speed_set 死区，静止时不补偿

static fp32 vision_pitch_abs_target_filtered_rad = 0.0f;
static uint8_t vision_pitch_abs_filter_initialized = 0U;

//扫敌模式角度积累
static fp32 sweep_360_yaw_accumulated = 0.0f;

static fp32 abs_fp32(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}

fp32 top_yaw_sector_weight = 0.0f;

static void update_top_yaw_sector_weight(void)
{
#if TOP_YAW_SECTOR_BOOST_ENABLE
    uint8_t active = (gimbal.gimbal_mode == GIMBAL_TOP &&
                      chassis.chassis_behaviour_mode == CHASSIS_TOP) ? 1U : 0U;
#if YAW_SYSID_ENABLE
    if (serialplot.is_yaw_sysid_mode_active() != 0U)
    {
        active = 0U;
    }
#endif
    if (active != 0U)
    {
        // 圆周中心距离：1.2 -> +pi -> -pi -> -2.8 是同一个连续区间。
        // 编码器只用于调度增益，不作差分，不代替 IMU 姿态反馈。
        const fp32 half_width = 0.5f * rad_format(
            TOP_YAW_SECTOR_END_RAD - TOP_YAW_SECTOR_START_RAD);
        const fp32 center = TOP_YAW_SECTOR_START_RAD + half_width;
        const fp32 distance = abs_fp32(rad_format(gimbal.gimbal_yaw_motor.encode_angle - center));
        fp32 blend = fp32_constrain(
            (half_width + TOP_YAW_SECTOR_BLEND_RAD - distance) / TOP_YAW_SECTOR_BLEND_RAD,
            0.0f, 1.0f);
        blend = blend * blend * (3.0f - 2.0f * blend);
        // 时间斜率限制，避免进入 TOP 时恰好位于增强区而瞬间跳增益。
        const fp32 step = GIMBAL_CONTROL_TIME / TOP_YAW_SECTOR_RAMP_TIME_S;
        top_yaw_sector_weight += fp32_constrain(blend - top_yaw_sector_weight, -step, step);
        return;
    }
#endif
    top_yaw_sector_weight = 0.0f;
}

static fp32 get_yaw_bias_fade_scale(fp32 control_error_rad)
{
    const fp32 error_abs = abs_fp32(control_error_rad);

    if (error_abs <= yaw_bias_fade_error_zero_rad)
    {
        return 0.0f;
    }

    if (error_abs >= yaw_bias_fade_error_full_rad)
    {
        return 1.0f;
    }

    if (error_abs <= yaw_bias_fade_error_hold_rad)
    {
        return yaw_bias_fade_min_scale * (error_abs - yaw_bias_fade_error_zero_rad) / (yaw_bias_fade_error_hold_rad - yaw_bias_fade_error_zero_rad);
    }

    if (error_abs >= yaw_bias_fade_error_knee_rad)
    {
        const fp32 upper_scale = (error_abs - yaw_bias_fade_error_knee_rad) / (yaw_bias_fade_error_full_rad - yaw_bias_fade_error_knee_rad);
        return yaw_bias_fade_knee_scale + (1.0f - yaw_bias_fade_knee_scale) * upper_scale;
    }

    return yaw_bias_fade_min_scale + (yaw_bias_fade_knee_scale - yaw_bias_fade_min_scale) * (error_abs - yaw_bias_fade_error_hold_rad) / (yaw_bias_fade_error_knee_rad - yaw_bias_fade_error_hold_rad);
}

static void reset_top_yaw_ff_filter_state(void)
{
    top_yaw_ff_filtered_relative_omega = 0.0f;
    top_yaw_ff_filter_initialized = 0U;
}

static void reset_top_periodic_ff_state(void)
{
    top_periodic_ff_filtered_torque = 0.0f;
    top_periodic_ff_initialized = 0U;
}

static fp32 get_top_yaw_ff_error_scale(fp32 relative_omega_radps, fp32 gyro_error_rad)
{
    const fp32 error_abs = abs_fp32(gyro_error_rad);

    if (relative_omega_radps * gyro_error_rad >= 0.0f)
    {
        return 1.0f;
    }

    if (error_abs <= top_yaw_ff_error_hold_rad)
    {
        return 1.0f;
    }

    if (error_abs >= top_yaw_ff_error_backoff_rad)
    {
        return top_yaw_ff_min_scale;
    }

    return 1.0f -
           (1.0f - top_yaw_ff_min_scale) *
               (error_abs - top_yaw_ff_error_hold_rad) /
               (top_yaw_ff_error_backoff_rad - top_yaw_ff_error_hold_rad);
}

/* 四阶残差归零：每累计一个整转，按各阶复残差修正 (a_k,b_k) */
static void update_top_ff_adapt(fp32 th, fp32 err_rad, fp32 rel_omega)
{
    const fp32 two_pi = 6.28318530718f;
    if (abs_fp32(err_rad) > top_ff_adapt_guard_err_rad || rel_omega < 4.0f)
    {
        top_ff_adapt_acc_th = 0.0f;
        for (uint8_t k = 0U; k < 4U; ++k)
        {
            top_ff_adapt_acc_s[k] = 0.0f;
            top_ff_adapt_acc_c[k] = 0.0f;
        }
        top_ff_adapt_init = 0U;
        return;
    }
    if (top_ff_adapt_init == 0U)
    {
        top_ff_adapt_prev_th = th;
        top_ff_adapt_init = 1U;
        return;
    }
    const fp32 dth = rad_format(th - top_ff_adapt_prev_th);
    top_ff_adapt_prev_th = th;
    const fp32 w = abs_fp32(dth);
    if (w < 1e-6f)
    {
        return;
    }
    const fp32 s1 = sin(th);
    const fp32 c1 = cos(th);
    const fp32 s2 = 2.0f * s1 * c1;
    const fp32 c2 = c1 * c1 - s1 * s1;
    const fp32 s3 = s2 * c1 + c2 * s1;
    const fp32 c3 = c2 * c1 - s2 * s1;
    const fp32 s4 = 2.0f * s2 * c2;
    const fp32 c4 = c2 * c2 - s2 * s2;
    const fp32 sk[4] = {s1, s2, s3, s4};
    const fp32 ck[4] = {c1, c2, c3, c4};

    top_ff_adapt_acc_th += w;
    for (uint8_t k = 0U; k < 4U; ++k)
    {
        top_ff_adapt_acc_s[k] += err_rad * sk[k] * w;
        top_ff_adapt_acc_c[k] += err_rad * ck[k] * w;
    }
    if (top_ff_adapt_acc_th < two_pi)
    {
        return;
    }
    const fp32 inv_th = 2.0f * 57.2957795f / top_ff_adapt_acc_th;
    for (uint8_t k = 0U; k < 4U; ++k)
    {
        const fp32 es_deg = top_ff_adapt_acc_s[k] * inv_th;
        const fp32 ec_deg = top_ff_adapt_acc_c[k] * inv_th;
        top_ff_adapt_acc_s[k] = 0.0f;
        top_ff_adapt_acc_c[k] = 0.0f;
        if ((es_deg * es_deg + ec_deg * ec_deg) <
            (top_ff_adapt_min_err_deg * top_ff_adapt_min_err_deg))
        {
            continue;
        }
        const fp32 inv_g = top_ff_adapt_gamma / top_ff_adapt_g_mag[k];
        const fp32 kr = -inv_g * cos(top_ff_adapt_g_phase[k]);
        const fp32 ki = inv_g * sin(top_ff_adapt_g_phase[k]);
        top_ff_dyn_a[k] += kr * es_deg - ki * ec_deg;
        top_ff_dyn_b[k] += kr * ec_deg + ki * es_deg;
        top_ff_dyn_a[k] = fp32_constrain(top_ff_dyn_a[k], -top_ff_adapt_limit_nm[k],
                                         top_ff_adapt_limit_nm[k]);
        top_ff_dyn_b[k] = fp32_constrain(top_ff_dyn_b[k], -top_ff_adapt_limit_nm[k],
                                         top_ff_adapt_limit_nm[k]);
    }
    top_ff_adapt_acc_th = 0.0f;
}

static fp32 get_top_yaw_ff_torque(fp32 relative_omega_radps, fp32 gyro_error_rad)
{
    fp32 omega_ff = relative_omega_radps;
    fp32 top_command_omega = chassis.z.speed_set;
    fp32 ff_reference_omega = 0.0f;

    if (omega_ff < 0.0f)
    {
        omega_ff = 0.0f;
    }

    if (top_command_omega < 0.0f)
    {
        top_command_omega = 0.0f;
    }

    if (top_yaw_ff_filter_initialized == 0U)
    {
        top_yaw_ff_filtered_relative_omega = omega_ff;
        top_yaw_ff_filter_initialized = 1U;
    }
    else
    {
        const fp32 filter_alpha = GIMBAL_CONTROL_TIME / (top_yaw_ff_filter_tau_s + GIMBAL_CONTROL_TIME);
        top_yaw_ff_filtered_relative_omega +=
            filter_alpha * (omega_ff - top_yaw_ff_filtered_relative_omega);
    }

    ff_reference_omega = top_yaw_ff_filtered_relative_omega;
    if (ff_reference_omega < top_command_omega)
    {
        ff_reference_omega = top_command_omega;
    }

    if (ff_reference_omega < top_yaw_ff_enable_omega_radps)
    {
        return 0.0f;
    }

    fp32 top_yaw_ff_torque = top_yaw_ff_anchor_torque_nm +
                             top_yaw_ff_omega_gain *
                                 (ff_reference_omega - top_yaw_ff_anchor_omega_radps);

    top_yaw_ff_torque = fp32_constrain(top_yaw_ff_torque, 0.0f, top_yaw_ff_max_torque_nm);

    return top_yaw_ff_torque * get_top_yaw_ff_error_scale(relative_omega_radps, gyro_error_rad);
}

static fp32 get_top_periodic_ff_torque(fp32 relative_angle_rad, fp32 relative_omega_radps)
{
    fp32 target = 0.0f;

    if (top_periodic_ff_enable > 0.0f &&
        relative_omega_radps > top_yaw_ff_enable_omega_radps)
    {
        const fp32 theta = relative_angle_rad + top_periodic_ff_phase_offset_rad;
        const fp32 s1 = sin(theta);
        const fp32 c1 = cos(theta);
        /* 倍角/三倍角用三角恒等式展开，省 sin/cos 调用（1 kHz 控制周期里只有 2 次三角函数） */
        const fp32 s2 = 2.0f * s1 * c1;
        const fp32 c2 = c1 * c1 - s1 * s1;
        const fp32 s3 = s2 * c1 + c2 * s1;
        const fp32 c3 = c2 * c1 - s2 * s1;

        const fp32 s4 = 2.0f * s2 * c2;
        const fp32 c4 = c2 * c2 - s2 * s2;
        target = top_ff_dyn_a[0] * s1 + top_ff_dyn_b[0] * c1 +
                 top_ff_dyn_a[1] * s2 + top_ff_dyn_b[1] * c2 +
                 top_ff_dyn_a[2] * s3 + top_ff_dyn_b[2] * c3 +
                 top_ff_dyn_a[3] * s4 + top_ff_dyn_b[3] * c4;
        target = fp32_constrain(target, -top_periodic_ff_max_torque_nm,
                                top_periodic_ff_max_torque_nm);
    }

    if (top_periodic_ff_initialized == 0U)
    {
        top_periodic_ff_filtered_torque = target;
        top_periodic_ff_initialized = 1U;
    }
    else if (top_periodic_ff_fade_tau_s > 0.0f)
    {
        const fp32 alpha = GIMBAL_CONTROL_TIME / (top_periodic_ff_fade_tau_s + GIMBAL_CONTROL_TIME);
        top_periodic_ff_filtered_torque += alpha * (target - top_periodic_ff_filtered_torque);
    }
    else
    {
        top_periodic_ff_filtered_torque = target;
    }

    return top_periodic_ff_filtered_torque;
}

static uint8_t get_yaw_bias_hold_disable(fp32 control_error_rad, fp32 actual_speed_radps, uint8_t bias_fade_enable)
{
    if (bias_fade_enable == 0U)
    {
        yaw_bias_hold_latched = 0U;
        return 0U;
    }

    const fp32 error_abs = abs_fp32(control_error_rad);
    const fp32 speed_abs = abs_fp32(actual_speed_radps);

    if (error_abs <= yaw_bias_hold_enter_error_rad && speed_abs <= yaw_bias_hold_speed_radps)
    {
        yaw_bias_hold_latched = 1U;
    }
    else if (error_abs >= yaw_bias_hold_exit_error_rad || speed_abs >= (yaw_bias_hold_speed_radps * 2.0f))
    {
        yaw_bias_hold_latched = 0U;
    }

    return yaw_bias_hold_latched;
}

static fp32 get_yaw_startup_bias_nm(fp32 speed_set, fp32 torque_cmd_nm, fp32 control_error_rad, fp32 actual_speed_radps, uint8_t fade_enable)
{
    fp32 base_bias = 0.0f;

    if (speed_set > yaw_startup_speed_deadband || torque_cmd_nm > yaw_startup_torque_deadband)
    {
        base_bias = yaw_startup_positive_bias_nm;
    }
    else if (speed_set < -yaw_startup_speed_deadband || torque_cmd_nm < -yaw_startup_torque_deadband)
    {
        base_bias = yaw_startup_negative_bias_nm;
    }

    if (fade_enable == 0U)
    {
        return base_bias;
    }

    if (get_yaw_bias_hold_disable(control_error_rad, actual_speed_radps, fade_enable) != 0U)
    {
        return 0.0f;
    }

    return base_bias * get_yaw_bias_fade_scale(control_error_rad);
}

static fp32 get_vision_yaw_static_bias_nm(fp32 control_error_rad, fp32 actual_speed_radps)
{
    const fp32 error_abs = abs_fp32(control_error_rad);
    const fp32 speed_abs = abs_fp32(actual_speed_radps);

    if (error_abs <= vision_yaw_static_enter_error_rad ||
        speed_abs >= vision_yaw_static_exit_speed_radps)
    {
        return 0.0f;
    }

    fp32 direction = control_error_rad;

    if (direction == 0.0f)
    {
        return 0.0f;
    }

    const fp32 scale = fp32_constrain(
        (error_abs - vision_yaw_static_enter_error_rad) /
            (vision_yaw_static_full_error_rad - vision_yaw_static_enter_error_rad),
        0.0f,
        1.0f);

    return ((direction > 0.0f) ? 1.0f : -1.0f) * vision_yaw_static_bias_nm * scale;
}

static fp32 get_vision_yaw_error_metric(fp32 control_error_rad, fp32 vision_delta_rad)
{
    const fp32 control_error_abs = abs_fp32(control_error_rad);
    const fp32 vision_delta_abs = abs_fp32(vision_delta_rad);

    return (vision_delta_abs > control_error_abs) ? vision_delta_abs : control_error_abs;
}

static uint8_t update_vision_yaw_settled_state(uint8_t active, fp32 control_error_rad, fp32 vision_delta_rad)
{
    if (active == 0U)
    {
        vision_yaw_settled_latched = 0U;
        vision_yaw_settled_cycles = 0U;
        return 0U;
    }

    const fp32 error_abs = get_vision_yaw_error_metric(control_error_rad, vision_delta_rad);

    if (vision_yaw_settled_latched != 0U)
    {
        if (error_abs >= vision_yaw_settled_exit_error_rad)
        {
            vision_yaw_settled_latched = 0U;
            vision_yaw_settled_cycles = 0U;
        }

        return vision_yaw_settled_latched;
    }

    if (error_abs <= vision_yaw_settled_enter_error_rad)
    {
        if (vision_yaw_settled_cycles < vision_yaw_settled_enter_cycles)
        {
            vision_yaw_settled_cycles++;
        }

        if (vision_yaw_settled_cycles >= vision_yaw_settled_enter_cycles)
        {
            vision_yaw_settled_latched = 1U;
        }
    }
    else
    {
        vision_yaw_settled_cycles = 0U;
    }

    return vision_yaw_settled_latched;
}

static fp32 get_vision_yaw_dynamic_torque_scale(fp32 control_error_rad, fp32 vision_delta_rad, uint8_t settled)
{
    if (settled != 0U)
    {
        return yaw_torque_scale;
    }

    fp32 boost_scale =
        (get_vision_yaw_error_metric(control_error_rad, vision_delta_rad) - vision_yaw_torque_boost_enter_error_rad) /
        (vision_yaw_torque_boost_full_error_rad - vision_yaw_torque_boost_enter_error_rad);

    boost_scale = fp32_constrain(boost_scale, 0.0f, 1.0f);

    return yaw_torque_scale + (vision_yaw_torque_scale - yaw_torque_scale) * boost_scale;
}

static fp32 get_yaw_mit_damping_kd(uint8_t vision_yaw_takeover_active, fp32 control_error_rad, fp32 vision_delta_rad)
{
#if YAW_SYSID_ENABLE
    if (serialplot.is_yaw_sysid_mode_active() != 0U)
    {
        return 0.0f;
    }
#endif

    if (gimbal.gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        return 0.0f;
    }

    if (vision_yaw_takeover_active == 0U)
    {
        return yaw_mit_manual_damping_kd;
    }

    const fp32 error_abs = get_vision_yaw_error_metric(control_error_rad, vision_delta_rad);

    if (error_abs <= yaw_mit_vision_damping_near_error_rad)
    {
        return yaw_mit_vision_near_damping_kd;
    }

    if (error_abs >= yaw_mit_vision_damping_far_error_rad)
    {
        return yaw_mit_vision_far_damping_kd;
    }

    const fp32 blend =
        (error_abs - yaw_mit_vision_damping_near_error_rad) /
        (yaw_mit_vision_damping_far_error_rad - yaw_mit_vision_damping_near_error_rad);

    return yaw_mit_vision_near_damping_kd +
           (yaw_mit_vision_far_damping_kd - yaw_mit_vision_near_damping_kd) * blend;
}

static fp32 apply_vision_yaw_breakaway_torque(fp32 torque_cmd_nm,
                                              uint8_t vision_yaw_takeover_active,
                                              uint8_t vision_yaw_settled,
                                              fp32 control_error_rad,
                                              fp32 vision_delta_rad,
                                              fp32 actual_speed_radps)
{
    if (vision_yaw_takeover_active == 0U || vision_yaw_settled != 0U)
    {
        return torque_cmd_nm;
    }

    const fp32 error_abs = get_vision_yaw_error_metric(control_error_rad, vision_delta_rad);
    const fp32 speed_abs = abs_fp32(actual_speed_radps);

    if (error_abs <= vision_yaw_breakaway_enter_error_rad ||
        speed_abs >= vision_yaw_breakaway_exit_speed_radps)
    {
        return torque_cmd_nm;
    }

    fp32 direction = vision_delta_rad;
    if (abs_fp32(direction) <= vision_yaw_direction_deadband_rad)
    {
        direction = control_error_rad;
    }

    if (abs_fp32(direction) <= vision_yaw_direction_deadband_rad)
    {
        return torque_cmd_nm;
    }

    fp32 scale =
        (error_abs - vision_yaw_breakaway_enter_error_rad) /
        (vision_yaw_breakaway_full_error_rad - vision_yaw_breakaway_enter_error_rad);
    scale = fp32_constrain(scale, 0.0f, 1.0f);

    const fp32 min_torque =
        vision_yaw_breakaway_min_torque_nm +
        (vision_yaw_breakaway_max_torque_nm - vision_yaw_breakaway_min_torque_nm) * scale;
    const fp32 torque_abs = abs_fp32(torque_cmd_nm);

    if (torque_cmd_nm * direction > 0.0f && torque_abs >= min_torque)
    {
        return torque_cmd_nm;
    }

    if (torque_cmd_nm * direction < 0.0f && torque_abs > min_torque)
    {
        return torque_cmd_nm;
    }

    return (direction > 0.0f) ? min_torque : -min_torque;
}
//重力补偿参数


double test_tor=-1.55;
double initial_angle=-0.266;
//改动
double pitch_horizontal_imu_angle=0.0;
double deviation_angle=0; 
double final_tor=0;

fp32 final_yaw_tor = 0.0f;


// bool_t auto_switch = false; //自瞄开关放在Communicate中声明了

//云台模块 对象
Gimbal gimbal;

static fp32 constrain_symmetric(fp32 value, fp32 limit)
{
    return fp32_constrain(value, -limit, limit);
}

static fp32 apply_center_deadband(fp32 value, fp32 deadband)
{
    if (value > deadband)
    {
        return value - deadband;
    }
    if (value < -deadband)
    {
        return value + deadband;
    }

    return 0.0f;
}

static void reset_vision_yaw_delta_filter(void)
{
    vision_yaw_control_delta_filtered_rad = 0.0f;
    vision_yaw_delta_filter_initialized = 0U;
}

static const protocol_info_t *get_gimbal_imu_info(void)
{
    return imu.get_imu_output_info_point();
}

static fp32 get_imu_yaw_angle_rad(void)
{
    const protocol_info_t *imu_info = get_gimbal_imu_info();
    return rad_format(GIMBAL_IMU_YAW_SIGN * ANGLE_TO_RAD * imu_info->yaw + GIMBAL_IMU_YAW_OFFSET_RAD);
}

// H30 用户手册 2.8：FLU 载体系，ZYX (321) 欧拉角顺序。
// 协议 0x20 的 angle_y/angle_z 是机体系角速度 deg/s，不是欧拉角速度。
// IMU 随 pitch 运动时：yaw_dot = (wy*sin(roll) + wz*cos(roll)) / cos(pitch)。
static fp32 get_imu_yaw_rate_radps(void)
{
    const protocol_info_t *imu_info = get_gimbal_imu_info();
    // 必须使用 IMU 原始姿态；不能套用 pitch 电机的方向符号或机械零位。
    const fp32 roll_rad = ANGLE_TO_RAD * imu_info->roll;
    const fp32 pitch_rad = ANGLE_TO_RAD * imu_info->pitch;
    fp32 cos_pitch = cosf(pitch_rad);

    // 欧拉 yaw 在 pitch 接近 +/-90 度时奇异。限分母仅避免数值发散，
    // 不代表该姿态下 yaw 速度仍准确；正常云台俯仰范围不触发此限制。
    if (abs_fp32(cos_pitch) < GIMBAL_IMU_YAW_RATE_MIN_COS_PITCH)
    {
        cos_pitch = (cos_pitch < 0.0f) ?
            -GIMBAL_IMU_YAW_RATE_MIN_COS_PITCH : GIMBAL_IMU_YAW_RATE_MIN_COS_PITCH;
    }

    return GIMBAL_IMU_YAW_SIGN * ANGLE_TO_RAD *
        (imu_info->angle_y * sinf(roll_rad) + imu_info->angle_z * cosf(roll_rad)) / cos_pitch;
}

//改动
static fp32 get_imu_pitch_angle_rad(void)
{
    const protocol_info_t *imu_info = get_gimbal_imu_info();
    return GIMBAL_IMU_PITCH_CTRL_SIGN*(GIMBAL_IMU_PITCH_SIGN * ANGLE_TO_RAD * imu_info->pitch + GIMBAL_IMU_PITCH_OFFSET_RAD);
}

static fp32 constrain_pitch_encoder_angle(fp32 angle)
{
    return fp32_constrain(angle, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
}

static fp32 safe_pitch_set_from_feedback(void)
{
    const fp32 pitch_feedback = gimbal.gimbal_pitch_motor.encode_angle;

    if (pitch_feedback >= PITCH_MIN_ANGLE && pitch_feedback <= PITCH_MAX_ANGLE)
    {
        return pitch_feedback;
    }

    return constrain_pitch_encoder_angle(gimbal.gimbal_pitch_motor.encode_angle_set);
}
//改动
static fp32 constrain_pitch_imu_angle(fp32 angle)          // 原 constrain_pitch_encoder_angle
{
    return fp32_constrain(angle, PITCH_IMU_MIN_ANGLE, PITCH_IMU_MAX_ANGLE);
}

static fp32 safe_pitch_imu_set_from_feedback(void)         // 原 safe_pitch_set_from_feedback，逻辑可简化
{
    return constrain_pitch_imu_angle(gimbal.gimbal_pitch_motor.gyro_angle);
}

static fp32 vision_yaw_delta_to_control_angle(void)
{
    fp32 raw_delta = constrain_symmetric(
        GIMBAL_IMU_YAW_SIGN * vision.vision_recv_data.yaw * vision.delta_param.k_yaw,
        vision.delta_param.yaw_step_max);

    raw_delta = apply_center_deadband(raw_delta, vision_yaw_delta_deadband_rad);

    if (vision_yaw_delta_filter_initialized == 0U)
    {
        vision_yaw_control_delta_filtered_rad = raw_delta;
        vision_yaw_delta_filter_initialized = 1U;
    }
    else
    {
        const fp32 alpha = GIMBAL_CONTROL_TIME / (vision_yaw_delta_filter_tau_s + GIMBAL_CONTROL_TIME);
        vision_yaw_control_delta_filtered_rad += alpha * (raw_delta - vision_yaw_control_delta_filtered_rad);
    }

    if (abs_fp32(vision_yaw_control_delta_filtered_rad) <= vision_yaw_delta_deadband_rad)
    {
        vision_yaw_control_delta_filtered_rad = 0.0f;
    }

    return constrain_symmetric(vision_yaw_control_delta_filtered_rad, vision.delta_param.yaw_step_max);
}

static fp32 vision_yaw_vel_to_control_rate(void)
{
    return GIMBAL_IMU_YAW_SIGN * vision.vision_recv_data.yaw_vel;
}

static fp32 vision_yaw_acc_to_control_acc(void)
{
    return GIMBAL_IMU_YAW_SIGN * vision.vision_recv_data.yaw_acc;
}

static fp32 vision_pitch_delta_to_control_angle(void)
{
    return constrain_symmetric(
        GIMBAL_IMU_PITCH_CTRL_SIGN * vision.vision_recv_data.pitch * vision.delta_param.k_pitch,
        vision.delta_param.pitch_step_max);
}

static fp32 vision_pitch_vel_to_control_rate(void)
{
    return GIMBAL_IMU_PITCH_CTRL_SIGN * vision.vision_recv_data.pitch_vel;
}

static fp32 vision_pitch_acc_to_control_acc(void)
{
    return GIMBAL_IMU_PITCH_CTRL_SIGN * vision.vision_recv_data.pitch_acc;
}

static uint8_t vision_delta_mode_allowed(void)
{
    // TOP 视觉跟随的模式门槛：状态机允许运行，但实际输出仍由目标识别/track 状态 gated。
    return (gimbal.gimbal_mode == GIMBAL_TOP && chassis.chassis_behaviour_mode == CHASSIS_TOP) ? 1U : 0U;
}

static uint8_t top_mode_under_vision_takeover(void)
{
    if (vision_delta_mode_allowed() == 0U)
    {
        return 0U;
    }

    return (vision.rt.state == VISION_TRACK_UNLOCKED || vision.rt.state == VISION_TRACK_LOCKED) ? 1U : 0U;
}

static uint8_t vision_mpc_feedforward_active(void)
{
    return (top_mode_under_vision_takeover() != 0U && vision.vision_if_find_target() == TRUE) ? 1U : 0U;
}

static uint8_t quintic_yaw_drive_active(void)
{
    return (vision_mpc_feedforward_active() != 0U && quintic_yaw.active != 0U) ? 1U : 0U;
}

static uint8_t quintic_pitch_drive_active(void)
{
    return (vision_mpc_feedforward_active() != 0U && quintic_pitch.active != 0U) ? 1U : 0U;
}

static fp32 pitch_quintic_velocity_ff(void)
{
    if (quintic_pitch_drive_active() == 0U)
    {
        return 0.0f;
    }

    return constrain_symmetric(
        quintic_pitch_vel * quintic_pitch_vel_ff_gain,
        vision_pitch_vel_ff_max);
}

static fp32 pitch_quintic_torque_ff(void)
{
    if (quintic_pitch_drive_active() == 0U)
    {
        return 0.0f;
    }

    return constrain_symmetric(
        quintic_pitch_acc * quintic_pitch_acc_ff_gain,
        vision_pitch_ff_torque_max);
}

static void vision_delta_clear_yaw_release(void)
{
    memset(&vision.yaw_release_rt, 0, sizeof(vision.yaw_release_rt));
}

static void vision_delta_release_yaw_step(void)
{
    const fp32 release_delta = vision.yaw_release_rt.slots[vision.yaw_release_rt.index];

    if (release_delta != 0.0f)
    {
        gimbal.gimbal_yaw_motor.gyro_angle_set = rad_format(gimbal.gimbal_yaw_motor.gyro_angle_set + release_delta);
        vision.yaw_release_rt.slots[vision.yaw_release_rt.index] = 0.0f;
    }

    vision.yaw_release_rt.index++;
    if (vision.yaw_release_rt.index >= VISION_YAW_RELEASE_SLOTS)
    {
        vision.yaw_release_rt.index = 0U;
    }
}

static void vision_delta_sync_setpoint_to_feedback(void)
{
    vision_delta_clear_yaw_release();
    // 视觉接管中断/重新进入 acquire 或 lost_hold：不续接旧轨迹，直接清空重新起步。
    quintic_reset(&quintic_yaw);
    quintic_reset(&quintic_pitch);
    gimbal.gimbal_yaw_motor.gyro_angle_set = gimbal.gimbal_yaw_motor.gyro_angle;
    gimbal.gimbal_yaw_motor.speed_set = 0.0f;
    //gimbal.gimbal_pitch_motor.encode_angle_set = safe_pitch_set_from_feedback();
    gimbal.gimbal_pitch_motor.gyro_angle_set = safe_pitch_imu_set_from_feedback();

    gimbal.gimbal_pitch_motor.speed_set = 0.0f;

    gimbal.yaw_imu_rate_pid.align_state_to_current();
    gimbal.gimbal_yaw_motor.gyro_angle_pid.align_state_to_current();
    gimbal.gimbal_pitch_motor.speed_pid.align_state_to_current();
    //gimbal.gimbal_pitch_motor.encode_angle_pid.align_state_to_current();
    gimbal.gimbal_pitch_motor.gyro_angle_pid.align_state_to_current();
}

static void vision_delta_enter_acquire(void)
{
    reset_vision_yaw_delta_filter();
    vision_delta_sync_setpoint_to_feedback();
    vision.clear_lock_qualification();
    vision.rt.fresh = 0U;
    vision.rt.consumed = 1U;
    vision.rt.timeout = 0U;
    vision.rt.target_lost_count = 0U;
    vision.rt.state = VISION_ACQUIRE;
}

static void vision_delta_enter_lost_hold(void)
{
    reset_vision_yaw_delta_filter();
    vision_delta_sync_setpoint_to_feedback();
    vision.clear_lock_qualification();
    vision.rt.fresh = 0U;
    vision.rt.consumed = 1U;
    vision.rt.timeout = 1U;
    vision.rt.target_lost_count = 0U;
    vision.rt.state = VISION_LOST_HOLD;
}

static void vision_delta_disable(void)
{
    // 离开 TOP / 离开 chassis TOP / 视觉不再接管：quintic 直接清空，不保留给以后续接。
    quintic_reset(&quintic_yaw);
    quintic_reset(&quintic_pitch);
    reset_vision_yaw_delta_filter();
    vision.reset_runtime_state(0U);
}

static void vision_delta_update_lock_qualification(void)
{
    if (vision.vision_fire_requested() != 0U)
    {
        if (vision.rt.lock_enter_count < vision.delta_param.lock_enter_threshold)
        {
            vision.rt.lock_enter_count++;
        }
        vision.rt.lock_exit_count = 0U;

        if (vision.rt.lock_enter_count >= vision.delta_param.lock_enter_threshold)
        {
            vision.rt.state = VISION_TRACK_LOCKED;
            vision.rt.fire_qualified = 1U;
        }
        else
        {
            vision.rt.state = VISION_TRACK_UNLOCKED;
            vision.rt.fire_qualified = 0U;
        }
    }
    else
    {
        vision.rt.lock_enter_count = 0U;
        if (vision.rt.lock_exit_count < vision.delta_param.lock_exit_threshold)
        {
            vision.rt.lock_exit_count++;
        }
        vision.rt.fire_qualified = 0U;
        if (vision.rt.lock_exit_count >= vision.delta_param.lock_exit_threshold)
        {
            vision.rt.state = VISION_TRACK_UNLOCKED;
        }
    }
}

// 处理视觉 delta 控制帧
static void vision_delta_process_frame(void)
{   
    if (vision.vision_recv_data.mode > 2U)
    {
        vision.rt.invalid_frame = 1U;
        return;
    }
    else
    {
        vision.rt.invalid_frame = 0U;
        if (vision.rt.fresh == 0U || vision.rt.consumed != 0U)
        {
         vision_delta_release_yaw_step();
        return;
        }

        if (vision.vision_recv_data.mode == 0U)
        {
            // mode=0（视觉报告无目标）时立刻清空 quintic，不等 lost_threshold。
            // 否则 quintic_output_update() 每周期仍推进内部轨迹，只是输出被 gating
            // 抑制；若 1-2 帧后目标重新出现，链式续接会从旧轨迹的"未来点"起步，
            // 产生小跳变。立即 reset 后重新锁定时走 !active 分支从实际反馈角重同步。
            quintic_reset(&quintic_yaw);
            quintic_reset(&quintic_pitch);
            reset_vision_yaw_delta_filter();

            if (vision.rt.state == VISION_ACQUIRE)
            {
                vision.clear_lock_qualification();
                vision.rt.fresh = 0U;
                vision.rt.consumed = 1U;
                return;
            }

            if (vision.rt.target_lost_count < vision.delta_param.target_lost_threshold)
            {
                vision.rt.target_lost_count++;
            }

                vision.clear_lock_qualification();

            if (vision.rt.target_lost_count >= vision.delta_param.target_lost_threshold)
            {
            vision_delta_enter_lost_hold();
            }

            vision.rt.fresh = 0U;
            vision.rt.consumed = 1U;
            return;
    }

    vision_delta_clear_yaw_release();

    vision.rt.target_lost_count = 0U;
    vision.rt.timeout = 0U;
    if (vision.rt.state == VISION_ACQUIRE)
    {
        vision.rt.state = VISION_TRACK_UNLOCKED;
    }

    // ===== quintic 链式重规划 =====
    // 起点取自上段 quintic 在“此刻”（本控制周期）的求值，保持位置/速度/加速度连续。
    // quintic 未 active（首次锁定/刚从丢失重新锁定）时退回到实际反馈角作为起点。
    const uint32_t now = HAL_GetTick();

    fp32 p0_yaw, v0_yaw, a0_yaw;
    if (quintic_yaw.active != 0U && quintic_chain_state_yaw(now, &p0_yaw, &v0_yaw, &a0_yaw) != 0U)
    {
        // 链式起点 OK，p0_yaw/v0_yaw/a0_yaw 已由 quintic_chain_state_yaw 设置
    }
    else
    {
        // !active 或 chain state 检测到 NaN（已自动 reset）：从实际反馈角重同步
        p0_yaw = gimbal.gimbal_yaw_motor.gyro_angle;
        v0_yaw = gimbal.yaw_imu_rate_radps;
        a0_yaw = 0.0f;
    }

    fp32 p0_pit, v0_pit, a0_pit;
    if (quintic_pitch.active != 0U && quintic_chain_state_pitch(now, &p0_pit, &v0_pit, &a0_pit) != 0U)
    {
        // 链式起点 OK
    }
    else
    {
        p0_pit = gimbal.gimbal_pitch_motor.gyro_angle;
        v0_pit = gimbal.gimbal_pitch_motor.speed;
        a0_pit = 0.0f;
    }

    // 视觉 yaw/pitch 是 delta 误差角，"当前时刻"目标点 = 实际反馈 + delta。
    // yaw 仍按 IMU yaw 坐标控制；pitch 只在编码器域叠加、规划、限幅。
    // yaw 在 quintic_plan_yaw 内部相对 p0 做最短路径 unwrap，这里只需给绝对角。
    const fp32 target_now_yaw_abs = rad_format(
        gimbal.gimbal_yaw_motor.gyro_angle + vision_yaw_delta_to_control_angle());
    const fp32 target_now_pit = constrain_pitch_imu_angle(
        gimbal.gimbal_pitch_motor.gyro_angle + vision_pitch_delta_to_control_angle());

    quintic_plan_yaw(p0_yaw, v0_yaw, a0_yaw,
                      target_now_yaw_abs, vision_yaw_vel_to_control_rate(), vision_yaw_acc_to_control_acc(),
                      now);
    quintic_plan_pitch(p0_pit, v0_pit, a0_pit,
                        target_now_pit, vision_pitch_vel_to_control_rate(), vision_pitch_acc_to_control_acc(),
                        PITCH_IMU_MIN_ANGLE, PITCH_IMU_MAX_ANGLE,
                        now);

    vision_delta_update_lock_qualification();

    vision.rt.fresh = 0U;
    vision.rt.consumed = 1U;
}
}

// 更新视觉接管状态机
static void vision_delta_update_runtime(void)
{   
    const uint32_t now = HAL_GetTick();
    
    if (vision_delta_mode_allowed() == 0U)
    {
        vision_delta_disable();
        return;
    }

    if (vision.rt.state == VISION_DISABLED)
    {
        vision_delta_enter_acquire();
        return;
    }

    if (vision.rt.last_rx_tick != 0U &&
        (now - vision.rt.last_rx_tick) > vision.delta_param.vision_timeout_ms)
    {
        if (vision.rt.state == VISION_TRACK_UNLOCKED || vision.rt.state == VISION_TRACK_LOCKED)
        {
            vision_delta_enter_lost_hold();
            return;
        }

        vision.rt.timeout = 1U;
        vision.clear_lock_qualification();
        vision.rt.fresh = 0U;
        vision.rt.consumed = 1U;
        return;
    }

    vision.rt.timeout = 0U;

    if (vision.rt.state == VISION_LOST_HOLD)
    {
        vision_delta_enter_acquire();
        return;
    }

    vision_delta_process_frame();
}

// Yaw0 Pitch1
// yaw：IMU 角度外环 + IMU 航向角速度内环；pitch 保持原来的速度反馈。

float Gimbal_YAW_IMU_RATE_PID[6] = {
    YAW_IMU_RATE_PID_KP, YAW_IMU_RATE_PID_KI, YAW_IMU_RATE_PID_KD,
    YAW_IMU_RATE_PID_KF, YAW_IMU_RATE_PID_MAX_IOUT, YAW_IMU_RATE_PID_MAX_OUT};
float Gimbal_PITCH_SPEED_PID[6] = {1000.0f, 0.0f, 10.0f, 0.0f, 1.0f, 3500.0f};

float Gimbal_PITCH_IMU_PID[6] = {30.0f, 0.01f, 0.0f, 200.0f, 0.0f, 100.0f};
// yaw 外环参数统一在 gimbal.h 的 YAW_GYRO_PID_* 中调整，输出单位 rad/s。
float Gimbal_GYRO_PID[6] = {
    YAW_GYRO_PID_KP, YAW_GYRO_PID_KI, YAW_GYRO_PID_KD, YAW_GYRO_PID_KF,
    YAW_GYRO_PID_MAX_IOUT, YAW_GYRO_PID_MAX_OUT};

//原有speed PID参数 {1350.0f,0.0f,110.0f,0.0f,1.0f,3500.0f}
//原有encode PID参数 {10.0f,0.0f,25.0f,0.0f,1.0f,50.0f}
//原有gyro PID参数 {6.0f,0.0f,15.0f,0.0f,1.0f,50.0f}
//   分别是          P I D F IOUT MAX_IOUT MAX_OUT


//ENCODE P 8 I 0 D 15 IOUT 0 MAXOUT 16
/**
 * @brief          初始化云台
 * @Author         WSJ
 */
void Gimbal::init()
{
    /*---------------------------遥控器---------------------------------*/
    //遥控器数据指针获取
    gimbal_RC = vt13.get_vt13_remote_control_point();
    last_gimbal_RC = vt13.get_last_vt13_remote_control_point();

    /*----------------------------电机-----------------------------------*/
    gimbal_yaw_motor=DM_Motor(CAN_YAW_MOTOR_ID,MASTER_YAW_MOTOR_ID, can_receive.get_dm_motor_measure_point(0));
    gimbal_yaw_motor.dm_mit_init();
    yaw_imu_rate_radps = 0.0f;
    yaw_imu_rate_pid.init(PID_SPEED, Gimbal_YAW_IMU_RATE_PID, &yaw_imu_rate_radps, &gimbal_yaw_motor.speed_set, NULL);
    gimbal_yaw_motor.gyro_angle_pid.init(PID_ANGLE, Gimbal_GYRO_PID, &gimbal_yaw_motor.gyro_angle, &gimbal_yaw_motor.gyro_angle_set, NULL);

    gimbal_pitch_motor=DM_Motor(CAN_PITCH_MOTOR_ID,MASTER_PITCH_MOTOR_ID,can_receive.get_dm_motor_measure_point(1));
    gimbal_pitch_motor.dm_mit_init();
    gimbal_pitch_motor.speed_pid.init(PID_SPEED, Gimbal_PITCH_SPEED_PID, &gimbal_pitch_motor.speed, &gimbal_pitch_motor.speed_set, NULL);
    gimbal_pitch_motor.gyro_angle_pid.init(PID_ANGLE, Gimbal_PITCH_IMU_PID,          // 移到赋值之后
                                           &gimbal_pitch_motor.gyro_angle,
                                           &gimbal_pitch_motor.gyro_angle_set, NULL);

    /*------------------------云台状态机初始化----------------------------*/
    //初始化初始状态为无力模式
    gimbal_mode = GIMBAL_ZERO_FORCE;
    last_gimbal_mode = gimbal_mode;

    /*--------------------------滤波值初始化-----------------------------*/
    // const static fp32 gimbal_yaw_high_pass_filter_para[1] = {GIMBAL_ACCEL_YAW_NUM};
    // const static fp32 gimbal_pitch_high_pass_filter_para[1] = {GIMBAL_ACCEL_PITCH_NUM};
    //一阶高通滤波初始化
    //gimbal_yaw_high_pass_filter.init(GIMBAL_CONTROL_TIME, gimbal_yaw_high_pass_filter_para);
    //gimbal_pitch_high_pass_filter.init(GIMBAL_CONTROL_TIME, gimbal_pitch_high_pass_filter_para);

    feedback_update();
}

void Gimbal::feedback_update()
{
    //按键数据更新
    //key_state_update();
    gimbal_data_update();
    //模式切换数据保存
    mode_change_save();
    last_gimbal_mode = gimbal_mode;
}

void Gimbal::mode_change_save()
{
    //切入底盘跟随云台模式
    if (last_gimbal_mode != GIMBAL_TOP && gimbal_mode == GIMBAL_TOP)
    {   
        gimbal_yaw_motor.gyro_angle_set = gimbal_yaw_motor.gyro_angle;
        gimbal_yaw_motor.speed_set = 0.0f;
        yaw_imu_rate_pid.align_state_to_current();
        gimbal_yaw_motor.gyro_angle_pid.align_state_to_current();
        //gimbal_pitch_motor.encode_angle_set = safe_pitch_set_from_feedback();
        gimbal_pitch_motor.gyro_angle_set = safe_pitch_imu_set_from_feedback();

        gimbal_pitch_motor.speed_set = 0.0f;
        gimbal_pitch_motor.speed_pid.align_state_to_current();
        //gimbal_pitch_motor.encode_angle_pid.align_state_to_current();
        gimbal_pitch_motor.gyro_angle_pid.align_state_to_current();
    }
    //切入自由控制模式
    else if (last_gimbal_mode != GIMBAL_FREE && gimbal_mode == GIMBAL_FREE)
    {   
        gimbal_yaw_motor.gyro_angle_set = gimbal_yaw_motor.gyro_angle;
        gimbal_yaw_motor.speed_set = 0.0f;
        yaw_imu_rate_pid.align_state_to_current();
        gimbal_yaw_motor.gyro_angle_pid.align_state_to_current();
        gimbal_pitch_motor.gyro_angle_set =  constrain_pitch_imu_angle((fp32)pitch_horizontal_imu_angle);
        gimbal_pitch_motor.speed_set = 0.0f;
        gimbal_pitch_motor.speed_pid.align_state_to_current();
        gimbal_pitch_motor.gyro_angle_pid.align_state_to_current();
    }

}

/**
 * @brief          云台数据计算更新
 * @Author         WSJ
 */
void Gimbal::gimbal_data_update()
{
    
    gimbal_pitch_motor.update();
    gimbal_yaw_motor.update();
    gimbal_yaw_motor.gyro_angle = get_imu_yaw_angle_rad();
    //改动
    gimbal_pitch_motor.gyro_angle = get_imu_pitch_angle_rad(); 
    // yaw 速度内环使用 IMU 航向角速度（rad/s），与 gyro_angle 外环同坐标系。
    yaw_imu_rate_radps = get_imu_yaw_rate_radps();
    gimbal_yaw_motor.encode_angle = uint_to_float(gimbal_yaw_motor.motor_measure->p_int, -gimbal_yaw_motor.tmp.pmax, gimbal_yaw_motor.tmp.pmax, 16);//gimbal_pitch_motor.motor_measure->p_int
    gimbal_pitch_motor.encode_angle = uint_to_float(gimbal_pitch_motor.motor_measure->p_int, -gimbal_pitch_motor.tmp.pmax, gimbal_pitch_motor.tmp.pmax, 16);
}

void Gimbal::set_mode()
{
    //记录上一次的模式
    last_gimbal_mode = gimbal_mode;

#if YAW_SYSID_ENABLE
    gimbal_mode = GIMBAL_FREE;
    serialplot.update_yaw_sysid_state(gimbal_mode);
#else
    //拨杆改变模式(在这里修改拨杆值对应的模式)
    switch_control();
#endif

#if !YAW_SYSID_ENABLE
    /* [安全补丁 P2] 遥控失联 -> 云台无力（DM 失能，是丢控时最安全的云台状态） */
    if (vt13.is_rc_lost() != 0U)
    {
        gimbal_mode = GIMBAL_ZERO_FORCE;
    }
#endif

}


/**
 * @brief 云台拨杆控制模式切换函数
 * 
 * 根据遥控器模式拨杆的位置切换云台的工作模式：
 * - 拨杆向上：切换到GIMBAL_TOP模式（跟随底盘模式）
 * - 拨杆中位：切换到GIMBAL_FREE模式（自由模式）
 * - 拨杆向下：切换到GIMBAL_ZERO_FORCE模式（无力模式）
 * 
 * 该函数通过读取遥控器mode_sw开关的状态，设置相应的云台工作模式。
 */
void Gimbal::switch_control()
{
    if (switch_is_up(gimbal_RC->rc.mode_sw))
    {
        gimbal_mode = GIMBAL_TOP;
    }
    else if (switch_is_mid(gimbal_RC->rc.mode_sw))
    {
        gimbal_mode = GIMBAL_FREE;
    }
    else if (switch_is_down(gimbal_RC->rc.mode_sw))
    {
        gimbal_mode = GIMBAL_ZERO_FORCE;
    }
}


void Gimbal::set_control()
{
    
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

#if YAW_SYSID_ENABLE
    if (serialplot.is_yaw_sysid_mode_active() != 0U)
    {
        auto_switch=false;
        return;
    }
    else
#endif
    if (gimbal_mode == GIMBAL_TOP)
    {
       gimbal_top_control(&add_yaw_angle, &add_pitch_angle);
      
    }
    else if (gimbal_mode == GIMBAL_FREE)
    {   
        auto_switch=false;//关闭自瞄
        gimbal_free_control(&add_yaw_angle, &add_pitch_angle);//遥控器决定云台怎么动
        
    }
}




#define rc_deadband_limit(input, output, dealine) {      \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                 \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



void Gimbal::gimbal_top_control(fp32 *yaw, fp32 *pitch)
{
    //sweep_360_yaw_accumulated = 0.0f;
    if(is_sweeping_360){
        fp32 sweeping_speed = 30.0f;
        fp32 sweep_step = sweeping_speed * GIMBAL_CONTROL_TIME;

        if (sweep_360_yaw_accumulated < 2.0f * PI)
        {
            *yaw += sweep_step;
            sweep_360_yaw_accumulated += sweep_step;
        }
        else
        {
            is_sweeping_360 = false;
        }
    }
    gimbal_rc_to_control_angle(yaw, pitch);

    gimbal_yaw_motor.gyro_angle_set = rad_format(gimbal_yaw_motor.gyro_angle_set + *yaw);

    //gimbal_pitch_motor.set(constrain_pitch_encoder_angle(gimbal_pitch_motor.encode_angle_set + *pitch), ENCODE_ANGLE);
    gimbal_pitch_motor.set(constrain_pitch_imu_angle(gimbal_pitch_motor.gyro_angle_set + *pitch), GYRO_ANGLE);

    gimbal_yaw_motor.set(gimbal_yaw_motor.gyro_angle_set , GYRO_ANGLE);
 
}

/**
 * @brief          云台自由模式控制，yaw 使用 IMU 角度目标，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
void Gimbal::gimbal_free_control(fp32 *yaw, fp32 *pitch)
{
    gimbal_rc_to_control_angle(yaw, pitch);

    //gimbal_pitch_motor.set(constrain_pitch_encoder_angle(gimbal_pitch_motor.encode_angle_set + *pitch), ENCODE_ANGLE);
    gimbal_pitch_motor.set(constrain_pitch_imu_angle(gimbal_pitch_motor.gyro_angle_set + *pitch), GYRO_ANGLE);

    gimbal_yaw_motor.set(rad_format(gimbal_yaw_motor.gyro_angle_set + *yaw), GYRO_ANGLE);

}

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
// void Gimbal::gimbal_motionless_control(fp32 *yaw, fp32 *pitch)
// {
//     if (yaw == NULL || pitch == NULL)
//     {
//         return;
//     }
//     *yaw = 0.0f;
//     *pitch = 0.0f;
// }


// 每个控制周期（500Hz）调用一次，且必须在 solve()/output() 之前调用：
// 先按当前时刻求值 quintic_yaw/quintic_pitch，再在视觉真正接管时（与
// vision_mpc_feedforward_active() 同口径 gated）把求值结果写入
// yaw 写 IMU yaw 姿态 setpoint；pitch 直接写编码器位置 setpoint。
void Gimbal::quintic_output_update()
{
    const uint32_t now = HAL_GetTick();
    quintic_step(now);

    if (quintic_yaw_drive_active() != 0U)
    {
        gimbal_yaw_motor.gyro_angle_set = rad_format(quintic_yaw_pos);
    }

    if (quintic_pitch_drive_active() != 0U)
    {
        gimbal_pitch_motor.gyro_angle_set = constrain_pitch_imu_angle(quintic_pitch_pos);
    }

}

void Gimbal::solve()
{
    update_top_yaw_sector_weight();
#if YAW_SYSID_ENABLE
    if (serialplot.is_yaw_sysid_mode_active() != 0U)
    {
        gimbal_yaw_motor.speed_set = 0.0f;
        gimbal_yaw_motor.current_give = serialplot.get_yaw_sysid_torque_raw_eq();
        //gimbal_pitch_motor.solve(ENCODE_ANGLE);
        gimbal_pitch_motor.solve(GYRO_ANGLE);
    }
    else
#endif
    if (gimbal_mode == GIMBAL_TOP || gimbal_mode == GIMBAL_FREE)
    {
        // IMU 角度外环直接输出速度目标，不再转换成编码器目标。
        // 将历史量映射到当前角度附近，避免跨 +/-pi 时 D/F 项跳变。
        const fp32 gyro_error = rad_format(gimbal_yaw_motor.gyro_angle_set - gimbal_yaw_motor.gyro_angle);
        /* 2026-10-26 v4：保持死区 0.008 -> 0.002 rad。
         * 依据：真机 v3 采集里 1.5~3 Hz 抖动占 41.5% 方差，而 0.008 rad 死区使环路 66% 时间
         * 不动作；扇区增强区一直用 0.0015 rad 且无自激，故把基准收紧到同一量级。 */
        const fp32 top_hold_error = 0.002f +
            top_yaw_sector_weight * (TOP_YAW_SECTOR_HOLD_ERROR_RAD - 0.002f);
        if (gimbal_mode == GIMBAL_TOP &&
        vision_mpc_feedforward_active() == 0U &&
        abs_fp32(gyro_error) <= top_hold_error)
        {
            // 增强区缩小保持死区，使轻微偏移也能由角度环纠正。
            gimbal_yaw_motor.speed_set = 0.0f;
            gimbal_yaw_motor.gyro_angle_pid.align_state_to_current();
        }
        else
        {
            gimbal_yaw_motor.gyro_angle_pid.data.error = gyro_error -
            rad_format(gyro_error - gimbal_yaw_motor.gyro_angle_pid.data.error);
            gimbal_yaw_motor.gyro_angle_pid.data.set_last = gimbal_yaw_motor.gyro_angle_set -
            rad_format(gimbal_yaw_motor.gyro_angle_set - gimbal_yaw_motor.gyro_angle_pid.data.set_last);
            const fp32 base_angle_kp = gimbal_yaw_motor.gyro_angle_pid.data.Kp;
            gimbal_yaw_motor.gyro_angle_pid.data.Kp = base_angle_kp *
                (1.0f + top_yaw_sector_weight * (TOP_YAW_SECTOR_ANGLE_KP_SCALE - 1.0f));
            gimbal_yaw_motor.speed_set = gimbal_yaw_motor.gyro_angle_pid.pid_calc();
            gimbal_yaw_motor.gyro_angle_pid.data.Kp = base_angle_kp;
            gimbal_yaw_motor.speed_set = fp32_constrain(gimbal_yaw_motor.speed_set, -gimbal_yaw_motor.tmp.vmax, gimbal_yaw_motor.tmp.vmax);
        }


        // 只增强 P；保留同一套积分状态及 I/D/F 参数，避免双 PID 切换冲击。
        const fp32 base_rate_kp = yaw_imu_rate_pid.data.Kp;
        yaw_imu_rate_pid.data.Kp = base_rate_kp *
            (1.0f + top_yaw_sector_weight * (TOP_YAW_SECTOR_RATE_KP_SCALE - 1.0f));
        gimbal_yaw_motor.current_give = yaw_imu_rate_pid.pid_calc();
        yaw_imu_rate_pid.data.Kp = base_rate_kp;

        gimbal_pitch_motor.solve(GYRO_ANGLE);
    }
    else
    {
        // GIMBAL_ZERO_FORCE：不经过 gimbal_rc_to_control_angle()/vision_delta_update_runtime()，
        // 这里兜底清空 quintic，避免长时间无力后残留旧轨迹。
        quintic_reset(&quintic_yaw);
        quintic_reset(&quintic_pitch);
    }

}


void Gimbal::output()
{   
    const uint8_t yaw_motor_ready = (gimbal_yaw_motor.motor_measure != NULL && gimbal_yaw_motor.motor_measure->state != 0) ? 1U : 0U;
    const uint8_t pitch_motor_ready = (gimbal_pitch_motor.motor_measure != NULL && gimbal_pitch_motor.motor_measure->state != 0) ? 1U : 0U;
    fp32 yaw_control_error = 0.0f;
    fp32 yaw_output_torque = 0.0f;
    uint8_t yaw_bias_fade_enable = 1U;
    const uint8_t vision_yaw_takeover_active =
        (top_mode_under_vision_takeover() != 0U && vision.vision_if_find_target() == TRUE) ? 1U : 0U;

    //const fp32 pitch_position_cmd = constrain_pitch_encoder_angle(gimbal.gimbal_pitch_motor.encode_angle_set);
    const fp32 pitch_position_cmd = constrain_pitch_imu_angle(gimbal_pitch_motor.gyro_angle_set);
    gimbal.gimbal_pitch_motor.gyro_angle_set = pitch_position_cmd;

    deviation_angle = pitch_position_cmd - pitch_horizontal_imu_angle;
    final_tor = test_tor * cos(deviation_angle);

    if (gimbal_mode == GIMBAL_TOP || gimbal_mode == GIMBAL_FREE)
    {
        yaw_control_error = rad_format(gimbal_yaw_motor.gyro_angle_set - gimbal_yaw_motor.gyro_angle);
    }

    const fp32 vision_yaw_error_for_settle = vision_yaw_control_delta_filtered_rad;
    const uint8_t vision_yaw_settled =
        update_vision_yaw_settled_state(vision_yaw_takeover_active,
                                        yaw_control_error,
                                        vision_yaw_error_for_settle);
    const fp32 active_yaw_torque_scale =
        (vision_yaw_takeover_active != 0U) ?
            get_vision_yaw_dynamic_torque_scale(yaw_control_error, vision_yaw_error_for_settle, vision_yaw_settled) :
            yaw_torque_scale;
    gimbal_yaw_motor.current_give = active_yaw_torque_scale * gimbal_yaw_motor.current_give;

    if (gimbal_mode == GIMBAL_FREE)
    {
        gimbal_yaw_motor.current_give += get_yaw_startup_bias_nm(gimbal_yaw_motor.speed_set, gimbal_yaw_motor.current_give, yaw_control_error, gimbal_yaw_motor.speed, yaw_bias_fade_enable);
    }
    else if (vision_yaw_takeover_active != 0U && vision_yaw_settled == 0U)
    {
        gimbal_yaw_motor.current_give += get_vision_yaw_static_bias_nm(yaw_control_error, gimbal_yaw_motor.speed);
    }

    final_yaw_tor = 0.0f;
    const uint8_t top_yaw_ff_allowed =
        (vision_yaw_takeover_active != 0U) ? 0U : 1U;

    if (gimbal_mode == GIMBAL_TOP &&
        chassis.chassis_behaviour_mode == CHASSIS_TOP &&
        top_yaw_ff_allowed != 0U)
    {
#if TOP_YAW_FF_TEST_ENABLE
        final_yaw_tor = get_top_yaw_ff_torque(top_tune_debug.relative_omega, yaw_control_error);
#else
        reset_top_yaw_ff_filter_state();
#endif
        /* 四阶在线自适应：每转按残差修正 (a_k,b_k) */
        update_top_ff_adapt(chassis.chassis_relative_angle, yaw_control_error,
                            top_tune_debug.relative_omega);
        final_yaw_tor += get_top_periodic_ff_torque(chassis.chassis_relative_angle,
                                                   top_tune_debug.relative_omega);
        /* 软启动斜坡：进 TOP 后 0.15 s 内把整段前馈线性拉起，消除力矩阶跃 */
        if (top_ff_ramp_gain < 1.0f)
        {
            top_ff_ramp_gain += GIMBAL_CONTROL_TIME / top_ff_ramp_s;
            if (top_ff_ramp_gain > 1.0f)
            {
                top_ff_ramp_gain = 1.0f;
            }
        }
        final_yaw_tor *= top_ff_ramp_gain;
    }
    else
    {
        top_ff_ramp_gain = 0.0f;
        reset_top_yaw_ff_filter_state();
        reset_top_periodic_ff_state();
    }

    fp32 yaw_quintic_ff = 0.0f;
    if (quintic_yaw_drive_active() != 0U && vision_yaw_settled == 0U)
    {
        yaw_quintic_ff = constrain_symmetric(
            quintic_yaw_vel * quintic_yaw_vel_ff_gain + quintic_yaw_acc * quintic_yaw_acc_ff_gain,
            vision_yaw_ff_torque_max);
    }
    yaw_output_torque =
        gimbal_yaw_motor.current_give + final_yaw_tor + yaw_quintic_ff;
    yaw_output_torque = apply_vision_yaw_breakaway_torque(
        yaw_output_torque,
        vision_yaw_takeover_active,
        vision_yaw_settled,
        yaw_control_error,
        vision_yaw_error_for_settle,
        gimbal_yaw_motor.speed);
    yaw_output_torque = constrain_symmetric(yaw_output_torque,
                                            yaw_output_torque_limit_nm);
	//这里×0.001只是用来做缩放比例的，现在正常使用dm6006电机作为yaw轴，而yaw轴电机目前是纯力矩控制的电机
    //所以如果在电机调参震动的时候，上位机发现current_give绝对值达到或超过4，就是达到额定扭矩了
    //力矩和电流的转化关系在mit模式里，达妙电机中是这样：
    //i=tot/kt_out,kt_out=kt*减速比,kt=1.5*极对数*磁链，这些参数dm上位机可以看
    //本人测试大约，4310电机：kt=1.5*14*0.00439719,减速比为10
    //6006电机kt=1.5*14*0.006071686,减速比为6（每个电机实际都不一样，相同的种类相近而已）
    if (gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_yaw_motor.current_give = 0;
        gimbal_pitch_motor.current_give = 0;
        final_yaw_tor = 0.0f;
        yaw_output_torque = 0.0f;
        reset_top_yaw_ff_filter_state();
        reset_top_periodic_ff_state();
        can_receive.DISABLE_DM_Motor_mode(gimbal_yaw_motor.id,gimbal_yaw_motor.ctrl.mode);
        can_receive.DISABLE_DM_Motor_mode(gimbal_pitch_motor.id,gimbal_yaw_motor.ctrl.mode);
    }
    else
    {
        if (pitch_motor_ready == 0U || yaw_motor_ready == 0U)
        {
            can_receive.ENABLE_DM_Motor_mode(gimbal_yaw_motor.id,gimbal_yaw_motor.ctrl.mode);
            can_receive.ENABLE_DM_Motor_mode(gimbal_pitch_motor.id,gimbal_yaw_motor.ctrl.mode);

#if YAW_SYSID_ENABLE
            serialplot.send_yaw_sysid_frame();
#endif
            return;
        }
    }
    const fp32 pitch_torque_scale = 0.001f;              // 速度环输出 → Nm 缩放（跟 yaw 对齐）
    const fp32 pitch_output_torque_limit_nm = 4.0f;
    fp32 pitch_pid_torque =
    pitch_torque_scale * gimbal_pitch_motor.current_give;


     if (gimbal.gimbal_pitch_motor.gyro_angle >= PITCH_IMU_MAX_ANGLE && pitch_pid_torque > 0.0f)
    {
        pitch_pid_torque = 0.0f;
    }
    else if (gimbal.gimbal_pitch_motor.gyro_angle <= PITCH_IMU_MIN_ANGLE && pitch_pid_torque < 0.0f)
    {
        pitch_pid_torque = 0.0f;
    }

      fp32 pitch_friction_tor = 0.0f;
    if (gimbal_pitch_motor.speed_set > pitch_friction_speed_deadband)
    {
        pitch_friction_tor = pitch_friction_comp_nm;
    }
    else if (gimbal_pitch_motor.speed_set < -pitch_friction_speed_deadband)
    {
        pitch_friction_tor = -pitch_friction_comp_nm;
    }

    fp32 pitch_output_torque = constrain_symmetric(
        pitch_pid_torque + final_tor + pitch_friction_tor + pitch_quintic_torque_ff(),
        pitch_output_torque_limit_nm);  
  
    // fp32 pitch_output_torque = constrain_symmetric(
    // pitch_pid_torque + final_tor + pitch_quintic_torque_ff(),
    // pitch_output_torque_limit_nm);

    //mit格式:pos,kp,kd,vel,tor
    //pitch轴 
    //can_receive.mit_ctrl(0,0,0,0,gimbal_pitch_motor.current_give,gimbal_pitch_motor.id,gimbal_pitch_motor.ctrl.mode,gimbal_pitch_motor.tmp);
    // can_receive.mit_ctrl(gimbal_pitch_motor.encode_angle_set,65,1.4,0,final_tor,gimbal_pitch_motor.id,gimbal_pitch_motor.ctrl.mode,gimbal_pitch_motor.tmp);
     can_receive.mit_ctrl(
         0,
         0,
         0,
         0,
         pitch_output_torque,
         gimbal_pitch_motor.id,
         gimbal_pitch_motor.ctrl.mode,
         gimbal_pitch_motor.tmp);
    can_receive.mit_ctrl(
        0,
        0.0f,
        get_yaw_mit_damping_kd(vision_yaw_takeover_active,
                               yaw_control_error,
                               vision_yaw_error_for_settle),
        0.0f,
        yaw_output_torque,
        gimbal_yaw_motor.id,
        gimbal_yaw_motor.ctrl.mode,
        gimbal_yaw_motor.tmp);
    
   

#if YAW_SYSID_ENABLE
    serialplot.send_yaw_sysid_frame();
#endif
    // can_receive.mit_ctrl(0,0,0,0,gimbal_yaw_motor.current_give,gimbal_yaw_motor.id,gimbal_yaw_motor.ctrl.mode,gimbal_yaw_motor.tmp);
    //yaw轴
    static uint8_t diag_send_divider = 0U;
    if ((diag_send_divider = (uint8_t)((diag_send_divider + 1U) & 0x03U)) == 0U)
    {
        serialplot.send_yaw_diag_frame();
    }
} 


/**
 * @brief 将遥控器输入转换为云台控制角度
 * 
 * 该函数根据当前模式（自瞄模式或手动模式）处理遥控器和鼠标输入，
 * 计算出云台的偏航角(yaw)和俯仰角(pitch)控制值。
 * 在自瞄模式下且识别到目标时，云台控制权交给mini PC；
 * 否则使用遥控器和鼠标输入进行手动控制。
 * 
 * @param[in,out] yaw 指向偏航角控制值的指针，函数会将计算结果存入该地址
 * @param[in,out] pitch 指向俯仰角控制值的指针，函数会将计算结果存入该地址
 */
void Gimbal::gimbal_rc_to_control_angle(fp32 *yaw, fp32 *pitch)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

#if VISION_CTRL_MODE == VISION_CTRL_MODE_DELTA
    vision_delta_update_runtime();

   if ((vision.rt.state == VISION_TRACK_UNLOCKED || vision.rt.state == VISION_TRACK_LOCKED) &&
       vision_delta_mode_allowed() != 0U)
   {
       *pitch = 0;
       *yaw = 0;
       BSP_Buzzer_Off();
   }
   else
    {
         fp32 yaw_channel = 0, pitch_channel = 0;
         //死区限制
         rc_deadband_limit(gimbal_RC->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
         rc_deadband_limit(gimbal_RC->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

         *yaw = yaw_channel * YAW_RC_SEN + gimbal_RC->mouse.x * YAW_MOUSE_SEN;
         *pitch = pitch_channel * PITCH_RC_SEN + gimbal_RC->mouse.y * PITCH_MOUSE_SEN;
    
         BSP_Buzzer_On();
     }
#else
	   if (vision_delta_mode_allowed() != 0U && vision.vision_if_find_target() == TRUE&& (HAL_GetTick() - vision.rt.last_rx_tick) <= VISION_ABS_TIMEOUT_MS)
	   {
	       // ABS 模式开火资格：视觉接管中，开火请求直接跟随当前帧 mode 位（2=请求开火）
	       vision.rt.fire_qualified = vision.vision_fire_requested();

	       fp32 vision_yaw_set = 0.0f;
	       fp32 vision_pitch_set = 0.0f;
	       vision.vision_get_angle(&vision_yaw_set, &vision_pitch_set);
	       gimbal_yaw_motor.gyro_angle_set =
	           rad_format(GIMBAL_IMU_YAW_SIGN * vision_yaw_set + GIMBAL_IMU_YAW_OFFSET_RAD);

               
            const fp32 pitch_target_raw =
                GIMBAL_IMU_PITCH_CTRL_SIGN * vision_pitch_set;
            // if (vision_pitch_abs_filter_initialized == 0U)
            // {
            //     vision_pitch_abs_target_filtered_rad = pitch_target_raw;
            //     vision_pitch_abs_filter_initialized = 1U;
            // }
            // else
            // {
            //     const fp32 alpha = GIMBAL_CONTROL_TIME / (0.06f + GIMBAL_CONTROL_TIME);
            //     vision_pitch_abs_target_filtered_rad +=
            //         alpha * (pitch_target_raw - vision_pitch_abs_target_filtered_rad);
            // }
            // gimbal_pitch_motor.gyro_angle_set =
            //     constrain_pitch_imu_angle(vision_pitch_abs_target_filtered_rad);
               
             gimbal_pitch_motor.gyro_angle_set =
                constrain_pitch_imu_angle(pitch_target_raw);

	       *pitch = 0;
	       *yaw = 0;
	       BSP_Buzzer_Off();
   }
   else
    {
         vision_pitch_abs_filter_initialized = 0U;   // 退出视觉接管，重置目标滤波
         vision.rt.fire_qualified = 0U;   // 视觉未接管/超时/无目标：禁止视觉开火
         fp32 yaw_channel = 0, pitch_channel = 0;
         rc_deadband_limit(gimbal_RC->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
         rc_deadband_limit(gimbal_RC->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

         *yaw = yaw_channel * YAW_RC_SEN + gimbal_RC->mouse.x * YAW_MOUSE_SEN;
         *pitch = pitch_channel * PITCH_RC_SEN + gimbal_RC->mouse.y * PITCH_MOUSE_SEN;
    
         BSP_Buzzer_On();
    }
#endif
}

int Gimbal::float_to_uint(fp32 x_float, fp32 x_min, fp32 x_max, int bits)
{
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return (int) ((x_float-offset)*((fp32)((1<<bits)-1))/span);
}


fp32 Gimbal::uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return ((fp32)x_int)*span/((fp32)((1<<bits)-1)) + offset;
}

fp32 Gimbal::angle_to_rad(fp32 angle)
{
    return angle * PI / 180.0f;
}
