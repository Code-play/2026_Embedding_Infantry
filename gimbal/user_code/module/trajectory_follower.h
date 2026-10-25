#ifndef TRAJECTORY_FOLLOWER_H
#define TRAJECTORY_FOLLOWER_H

#include "struct_typedef.h"

#define QUINTIC_COEFFS 6

// 单轴 quintic 轨迹。a[0..5] 是相对 t_start_ms 的多项式系数：
// x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5, t in [0, T]
// p0/v0/a0_/pT/vT/aT 只作诊断/调参用，实际求值只依赖 a[] 与 T。
typedef struct
{
    fp32 a[QUINTIC_COEFFS];
    uint32_t t_start_ms;
    fp32 T;
    fp32 p0, v0, a0_;
    fp32 pT, vT, aT;
    uint8_t active;
    uint8_t degraded; // 上一次规划是否触发了 T_max 降级策略（诊断用）
} QuinticTrajectory;

extern QuinticTrajectory quintic_yaw;
extern QuinticTrajectory quintic_pitch;

// --- 规划参数 ---
extern fp32 quintic_yaw_T_min;      // yaw 最小规划周期 (s)
extern fp32 quintic_yaw_T_max;      // yaw 最大规划周期 (s)
extern fp32 quintic_pitch_T_min;    // pitch 最小规划周期 (s)
extern fp32 quintic_pitch_T_max;    // pitch 最大规划周期 (s)
extern fp32 quintic_max_yaw_rate;   // rad/s
extern fp32 quintic_max_pitch_rate; // rad/s
extern fp32 quintic_max_yaw_acc;    // rad/s^2
extern fp32 quintic_max_pitch_acc;  // rad/s^2
extern fp32 quintic_acc_decay_T;    // 终点加速度衰减时间常数 (s)

// --- 前馈增益（沿用现有 torque FF 通道语义，不改变单位）---
extern fp32 quintic_yaw_vel_ff_gain;
extern fp32 quintic_yaw_acc_ff_gain;
extern fp32 quintic_pitch_vel_ff_gain;
extern fp32 quintic_pitch_acc_ff_gain;

// --- 实时输出（quintic_step() 每控制周期更新一次）---
extern fp32 quintic_yaw_pos, quintic_yaw_vel, quintic_yaw_acc;
extern fp32 quintic_pitch_pos, quintic_pitch_vel, quintic_pitch_acc;

// 清空一条轨迹（active=0），不进行续接。
void quintic_reset(QuinticTrajectory *q);

// 按当前时刻求值 quintic_yaw / quintic_pitch，更新上面的实时输出变量。
// 必须每个控制周期调用一次，且必须在 Gimbal::solve() 与 Gimbal::output() 之前调用，
// 否则两者读到的 quintic_*_pos/vel/acc 是上一次视觉帧到达时的陈旧值。
void quintic_step(uint32_t now_ms);

// 按给定时刻 t（相对轨迹起点，单位 s）直接求值，供外部诊断使用。
void quintic_eval(const QuinticTrajectory *q, fp32 t, fp32 *pos, fp32 *vel, fp32 *acc);

// 重规划前取链式起点：对 quintic_yaw/quintic_pitch 在 now_ms 时刻求值，并把
// v0/a0 钳制到配置限值内。降级轨迹在个别采样点可能仍略超限（见 quintic_plan_axis
// 的降级说明），若不钳制会被当作下一段的边界条件带入闭式解，导致超限被链式放大。
// 只应在对应 quintic_yaw.active / quintic_pitch.active 为真时调用。
// 返回 1=成功，0=求值出现 NaN/Inf（已自动 reset，调用方应回退到实际反馈角作为起点）。
uint8_t quintic_chain_state_yaw(uint32_t now_ms, fp32 *p0, fp32 *v0, fp32 *a0);
uint8_t quintic_chain_state_pitch(uint32_t now_ms, fp32 *p0, fp32 *v0, fp32 *a0);

// yaw 重规划：target_now_pos_abs 是视觉 "当前时刻" 点的绝对角（已 rad_format 过）。
// 内部做最短路径 unwrap，再自适应选 T：候选 T -> 外推终点 -> 求解 quintic ->
// 采样检查峰值 vel/acc -> 超限则增大 T -> 重复；T_max 仍超限则降级。
void quintic_plan_yaw(fp32 p0, fp32 v0, fp32 a0,
                      fp32 target_now_pos_abs, fp32 target_now_vel, fp32 target_now_acc,
                      uint32_t now_ms);

// pitch 重规划：target_now_pos 无需 unwrap，但终点会按 [pos_min, pos_max] 机械限位裁剪。
void quintic_plan_pitch(fp32 p0, fp32 v0, fp32 a0,
                        fp32 target_now_pos, fp32 target_now_vel, fp32 target_now_acc,
                        fp32 pos_min, fp32 pos_max,
                        uint32_t now_ms);

#endif
