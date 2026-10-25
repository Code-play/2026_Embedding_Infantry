#include "trajectory_follower.h"
#include "user_lib.h"
#include <math.h>
#include <string.h>

QuinticTrajectory quintic_yaw;
QuinticTrajectory quintic_pitch;

fp32 quintic_yaw_T_min      = 0.010f;
fp32 quintic_yaw_T_max      = 0.070f;
fp32 quintic_pitch_T_min    = 0.040f;
fp32 quintic_pitch_T_max    = 0.300f;
fp32 quintic_max_yaw_rate   = 14.0f;
fp32 quintic_max_pitch_rate = 6.0f;
fp32 quintic_max_yaw_acc    = 100.0f;
fp32 quintic_max_pitch_acc  = 100.0f;
fp32 quintic_acc_decay_T    = 0.200f;

fp32 quintic_yaw_vel_ff_gain   = 0.020f; // yaw 速度前馈扭矩增益
fp32 quintic_yaw_acc_ff_gain   = 0.003f; // yaw 加速度前馈扭矩增益
fp32 quintic_pitch_vel_ff_gain = 1.0f;   // MIT vel 直通增益
fp32 quintic_pitch_acc_ff_gain = 0.002f; // pitch 加速度前馈扭矩增益

fp32 quintic_yaw_pos = 0.0f, quintic_yaw_vel = 0.0f, quintic_yaw_acc = 0.0f;
fp32 quintic_pitch_pos = 0.0f, quintic_pitch_vel = 0.0f, quintic_pitch_acc = 0.0f;

// 自适应 T 搜索参数
#define QUINTIC_T_GROWTH 1.4f
#define QUINTIC_T_SEARCH_MAX_ITERS 8
#define QUINTIC_CHECK_SAMPLES 16 // 每条候选轨迹在 [0,T] 上采样 17 个点检查峰值 vel/acc

// ============================================================
// quintic 系数闭式解（边界条件 p(0)=p0,p'(0)=v0,p''(0)=a0；p(T)=pT,p'(T)=vT,p''(T)=aT）
// ============================================================
static void quintic_solve_coeffs(fp32 p0, fp32 v0, fp32 a0,
                                  fp32 pT, fp32 vT, fp32 aT,
                                  fp32 T, fp32 coeffs[QUINTIC_COEFFS])
{
    const fp32 T2 = T * T;
    const fp32 T3 = T2 * T;
    const fp32 T4 = T3 * T;
    const fp32 T5 = T4 * T;
    const fp32 dp = pT - p0;

    coeffs[0] = p0;
    coeffs[1] = v0;
    coeffs[2] = a0 * 0.5f;

    coeffs[3] = (20.0f * dp - (8.0f * vT + 12.0f * v0) * T
                 - (3.0f * a0 - aT) * T2) / (2.0f * T3);

    coeffs[4] = (30.0f * (p0 - pT) + (14.0f * vT + 16.0f * v0) * T
                 + (3.0f * a0 - 2.0f * aT) * T2) / (2.0f * T4);

    coeffs[5] = (12.0f * dp - 6.0f * (v0 + vT) * T
                 - (a0 - aT) * T2) / (2.0f * T5);
}

static void quintic_eval_coeffs(const fp32 coeffs[QUINTIC_COEFFS], fp32 t,
                                 fp32 *pos, fp32 *vel, fp32 *acc)
{
    // Horner 形式求值
    *pos = coeffs[0] + t * (coeffs[1] + t * (coeffs[2] + t * (coeffs[3] + t * (coeffs[4] + t * coeffs[5]))));
    *vel = coeffs[1] + t * (2.0f * coeffs[2] + t * (3.0f * coeffs[3] + t * (4.0f * coeffs[4] + t * 5.0f * coeffs[5])));
    *acc = 2.0f * coeffs[2] + t * (6.0f * coeffs[3] + t * (12.0f * coeffs[4] + t * 20.0f * coeffs[5]));
}

void quintic_eval(const QuinticTrajectory *q, fp32 t, fp32 *pos, fp32 *vel, fp32 *acc)
{
    quintic_eval_coeffs(q->a, t, pos, vel, acc);
}

static uint8_t fp32_all_finite6(fp32 a, fp32 b, fp32 c, fp32 d, fp32 e, fp32 f)
{
    return (isfinite(a) && isfinite(b) && isfinite(c) &&
            isfinite(d) && isfinite(e) && isfinite(f)) ? 1U : 0U;
}

static uint8_t quintic_coeffs_finite(const fp32 coeffs[QUINTIC_COEFFS])
{
    for (int i = 0; i < QUINTIC_COEFFS; ++i)
    {
        if (!isfinite(coeffs[i]))
        {
            return 0U;
        }
    }
    return 1U;
}

// 采样检查候选轨迹在 [0,T] 上的峰值 |vel|/|acc| 是否满足约束。
// NaN/Inf 必须显式判定为不通过：fabsf(NaN) > max 恒为 false，若不特判会被误判为"满足约束"，
// 一旦视觉包或计算中出现 NaN，会直接生成 NaN setpoint 下发给电机。
static uint8_t quintic_check_peak(const fp32 coeffs[QUINTIC_COEFFS], fp32 T, fp32 max_vel, fp32 max_acc)
{
    for (int i = 0; i <= QUINTIC_CHECK_SAMPLES; ++i)
    {
        const fp32 t = T * ((fp32)i / (fp32)QUINTIC_CHECK_SAMPLES);
        fp32 pos, vel, acc;
        quintic_eval_coeffs(coeffs, t, &pos, &vel, &acc);
        if (!isfinite(pos) || !isfinite(vel) || !isfinite(acc) ||
            fabsf(vel) > max_vel || fabsf(acc) > max_acc)
        {
            return 0U;
        }
    }
    return 1U;
}

void quintic_reset(QuinticTrajectory *q)
{
    memset(q->a, 0, sizeof(q->a));
    q->t_start_ms = 0U;
    q->T = 0.0f;
    q->p0 = q->v0 = q->a0_ = 0.0f;
    q->pT = q->vT = q->aT = 0.0f;
    q->active = 0U;
    q->degraded = 0U;
}

// 通用单轴自适应规划：候选 T -> 外推终点(含加速度衰减) -> 求解 quintic ->
// 采样检查真实轨迹峰值 vel/acc -> 超限则增大 T -> 重复。
// T_max 仍超限时执行降级：先去掉终点 vel/acc 要求，仍超限则把位移裁剪到
// T_max 下零速零加速 minimum-jerk profile 恰好达到限速/限加速的最大可行位移。
static void quintic_plan_axis(QuinticTrajectory *q,
                               fp32 p0, fp32 v0, fp32 a0,
                               fp32 target_now_pos, fp32 target_now_vel, fp32 target_now_acc,
                               fp32 T_min, fp32 T_max,
                               fp32 max_vel, fp32 max_acc,
                               uint8_t has_pos_limit, fp32 pos_min, fp32 pos_max,
                               uint32_t now_ms)
{
    // 输入 NaN/Inf 防护：视觉包损坏或上游计算异常时，绝不能拿 NaN/Inf 去解闭式系数
    // （会静默生成 NaN 系数并可能通过后续检查）。直接放弃本次重规划，reset 后由
    // 调用方在 !active 分支回退到实际反馈角。
    if (fp32_all_finite6(p0, v0, a0, target_now_pos, target_now_vel, target_now_acc) == 0U)
    {
        quintic_reset(q);
        return;
    }

    fp32 T = T_min;
    fp32 coeffs[QUINTIC_COEFFS];
    fp32 pT = target_now_pos, vT = target_now_vel, aT = target_now_acc;
    uint8_t feasible = 0U;

    for (int iter = 0; iter < QUINTIC_T_SEARCH_MAX_ITERS; ++iter)
    {
        if (T >= T_max)
        {
            T = T_max;
        }

        const fp32 raw_decay = 1.0f - T / quintic_acc_decay_T;
        const fp32 decay = (raw_decay > 0.0f) ? raw_decay : 0.0f;
        aT = target_now_acc * decay;
        pT = target_now_pos + target_now_vel * T + 0.5f * aT * T * T;
        vT = target_now_vel + aT * T;

        if (has_pos_limit != 0U)
        {
            const fp32 pT_clamped = fp32_constrain(pT, pos_min, pos_max);
            if (pT_clamped != pT)
            {
                // 目标超出机械限位：按"到达边界即静止"处理，否则 vT/aT 仍指向限位
                // 外侧，会让后续 vel/acc FF 继续向限位外推。
                vT = 0.0f;
                aT = 0.0f;
            }
            pT = pT_clamped;
        }

        quintic_solve_coeffs(p0, v0, a0, pT, vT, aT, T, coeffs);

        if (quintic_check_peak(coeffs, T, max_vel, max_acc) != 0U)
        {
            feasible = 1U;
            break;
        }

        if (T >= T_max)
        {
            break; // 已在 T_max 仍不满足约束，退出进入降级流程
        }

        T *= QUINTIC_T_GROWTH;
    }

    uint8_t degraded = 0U;

    if (feasible == 0U)
    {
        degraded = 1U;
        T = T_max;

        // 降级步骤 1：去掉终点 vel/acc 要求，只保留位置目标
        aT = 0.0f;
        vT = 0.0f;
        pT = target_now_pos;
        if (has_pos_limit != 0U)
        {
            pT = fp32_constrain(pT, pos_min, pos_max);
        }

        quintic_solve_coeffs(p0, v0, a0, pT, vT, aT, T, coeffs);

        if (quintic_check_peak(coeffs, T, max_vel, max_acc) == 0U)
        {
            // 降级步骤 2：把位移裁剪到 T_max 下零速零加速 minimum-jerk
            // profile 恰好触及限速/限加速的最大可行位移（保证可行性）。
            // 零边界 minimum-jerk profile: v_peak = 1.875*dp/T, a_peak = 5.7735*dp/T^2,
            // 均对 dp 线性，反解即为下面两式（无需开方）。
            const fp32 dp_vel_limit = max_vel * T / 1.875f;
            const fp32 dp_acc_limit = max_acc * T * T / 5.7735f;
            const fp32 dp_limit = (dp_vel_limit < dp_acc_limit) ? dp_vel_limit : dp_acc_limit;

            fp32 dp = fp32_constrain(target_now_pos - p0, -dp_limit, dp_limit);
            pT = p0 + dp;
            if (has_pos_limit != 0U)
            {
                pT = fp32_constrain(pT, pos_min, pos_max);
            }
            vT = 0.0f;
            aT = 0.0f;

            quintic_solve_coeffs(p0, v0, a0, pT, vT, aT, T, coeffs);
        }
    }

    // 最终防线：即便输入合法，极端数值组合下闭式解仍可能产生非有限系数
    // （理论上 T 恒 >= T_min > 0 不应发生，防御性保留）。宁可放弃本次
    // 规划也不能把 NaN/Inf 系数提交进 active 轨迹。
    if (quintic_coeffs_finite(coeffs) == 0U)
    {
        quintic_reset(q);
        return;
    }

    memcpy(q->a, coeffs, sizeof(coeffs));
    q->T = T;
    q->t_start_ms = now_ms;
    q->p0 = p0;
    q->v0 = v0;
    q->a0_ = a0;
    q->pT = pT;
    q->vT = vT;
    q->aT = aT;
    q->active = 1U;
    q->degraded = degraded;
}

void quintic_plan_yaw(fp32 p0, fp32 v0, fp32 a0,
                      fp32 target_now_pos_abs, fp32 target_now_vel, fp32 target_now_acc,
                      uint32_t now_ms)
{
    // 最短路径 unwrap：dp 落在 (-pi, pi]，target_now_pos 保持连续（不做模运算），
    // 从而与上一段 quintic 的链式起点 p0 保持位置连续。只在最终写出 setpoint 时才 rad_format。
    const fp32 dp = rad_format(target_now_pos_abs - p0);
    const fp32 target_now_pos = p0 + dp;

    quintic_plan_axis(&quintic_yaw, p0, v0, a0,
                       target_now_pos, target_now_vel, target_now_acc,
                       quintic_yaw_T_min, quintic_yaw_T_max,
                       quintic_max_yaw_rate, quintic_max_yaw_acc,
                       0U, 0.0f, 0.0f,
                       now_ms);
}

void quintic_plan_pitch(fp32 p0, fp32 v0, fp32 a0,
                        fp32 target_now_pos, fp32 target_now_vel, fp32 target_now_acc,
                        fp32 pos_min, fp32 pos_max,
                        uint32_t now_ms)
{
    quintic_plan_axis(&quintic_pitch, p0, v0, a0,
                       target_now_pos, target_now_vel, target_now_acc,
                       quintic_pitch_T_min, quintic_pitch_T_max,
                       quintic_max_pitch_rate, quintic_max_pitch_acc,
                       1U, pos_min, pos_max,
                       now_ms);
}

// 链式起点钳制留一点余量（而不是恰好夹到限值）：闭式解在 v0/a0 恰好处于边界时，
// 为满足边界条件本身还会引入额外曲率，导致新一段的峰值略微超出限值
// （数值测试可见~2%量级）。留 10% 余量给求解器空间，配合 quintic_step() 的
// 硬钳制兜底，双重保证最终输出不超限。
#define QUINTIC_CHAIN_STATE_MARGIN 0.9f

uint8_t quintic_chain_state_yaw(uint32_t now_ms, fp32 *p0, fp32 *v0, fp32 *a0)
{
    const fp32 t = fp32_constrain((fp32)(now_ms - quintic_yaw.t_start_ms) * 0.001f, 0.0f, quintic_yaw.T);
    quintic_eval(&quintic_yaw, t, p0, v0, a0);
    if (!isfinite(*p0) || !isfinite(*v0) || !isfinite(*a0))
    {
        quintic_reset(&quintic_yaw);
        return 0U;
    }
    const fp32 v_limit = quintic_max_yaw_rate * QUINTIC_CHAIN_STATE_MARGIN;
    const fp32 a_limit = quintic_max_yaw_acc * QUINTIC_CHAIN_STATE_MARGIN;
    *v0 = fp32_constrain(*v0, -v_limit, v_limit);
    *a0 = fp32_constrain(*a0, -a_limit, a_limit);
    return 1U;
}

uint8_t quintic_chain_state_pitch(uint32_t now_ms, fp32 *p0, fp32 *v0, fp32 *a0)
{
    const fp32 t = fp32_constrain((fp32)(now_ms - quintic_pitch.t_start_ms) * 0.001f, 0.0f, quintic_pitch.T);
    quintic_eval(&quintic_pitch, t, p0, v0, a0);
    if (!isfinite(*p0) || !isfinite(*v0) || !isfinite(*a0))
    {
        quintic_reset(&quintic_pitch);
        return 0U;
    }
    const fp32 v_limit = quintic_max_pitch_rate * QUINTIC_CHAIN_STATE_MARGIN;
    const fp32 a_limit = quintic_max_pitch_acc * QUINTIC_CHAIN_STATE_MARGIN;
    *v0 = fp32_constrain(*v0, -v_limit, v_limit);
    *a0 = fp32_constrain(*a0, -a_limit, a_limit);
    return 1U;
}

void quintic_step(uint32_t now_ms)
{
    // 最后一道防线：fp32_constrain 对 NaN 的两个比较都是 false，会原样放行 NaN，
    // 起不到钳制作用，所以 NaN/Inf 必须在这里单独判定并直接 reset+归零，不能只靠
    // 下面的 constrain 兜底。
    if (quintic_yaw.active != 0U)
    {
        fp32 t = (fp32)(now_ms - quintic_yaw.t_start_ms) * 0.001f;
        t = fp32_constrain(t, 0.0f, quintic_yaw.T);
        quintic_eval(&quintic_yaw, t, &quintic_yaw_pos, &quintic_yaw_vel, &quintic_yaw_acc);
        if (!isfinite(quintic_yaw_pos) || !isfinite(quintic_yaw_vel) || !isfinite(quintic_yaw_acc))
        {
            quintic_reset(&quintic_yaw);
            quintic_yaw_pos = 0.0f;
            quintic_yaw_vel = 0.0f;
            quintic_yaw_acc = 0.0f;
        }
        else
        {
            // 安全兜底：即使规划/降级存在边界情况，也绝不向下游输出超过配置限值的 vel/acc。
            quintic_yaw_vel = fp32_constrain(quintic_yaw_vel, -quintic_max_yaw_rate, quintic_max_yaw_rate);
            quintic_yaw_acc = fp32_constrain(quintic_yaw_acc, -quintic_max_yaw_acc, quintic_max_yaw_acc);
        }
    }

    if (quintic_pitch.active != 0U)
    {
        fp32 t = (fp32)(now_ms - quintic_pitch.t_start_ms) * 0.001f;
        t = fp32_constrain(t, 0.0f, quintic_pitch.T);
        quintic_eval(&quintic_pitch, t, &quintic_pitch_pos, &quintic_pitch_vel, &quintic_pitch_acc);
        if (!isfinite(quintic_pitch_pos) || !isfinite(quintic_pitch_vel) || !isfinite(quintic_pitch_acc))
        {
            quintic_reset(&quintic_pitch);
            quintic_pitch_pos = 0.0f;
            quintic_pitch_vel = 0.0f;
            quintic_pitch_acc = 0.0f;
        }
        else
        {
            quintic_pitch_vel = fp32_constrain(quintic_pitch_vel, -quintic_max_pitch_rate, quintic_max_pitch_rate);
            quintic_pitch_acc = fp32_constrain(quintic_pitch_acc, -quintic_max_pitch_acc, quintic_max_pitch_acc);
        }
    }
}
