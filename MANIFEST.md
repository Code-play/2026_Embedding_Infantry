# 固件版本归档：2026-10-26_top_dc_fix（v3）

| 项 | 值 |
|---|---|
| 版本标签 | `2026-10-26_top_dc_fix`（v3） |
| 基线 | `2026-10-24_periodic_ff_v2`（v2） |
| hex | `hex/gimbal.hex`，348323 字节 |
| hex SHA256 | `739283C62FA5921B73EA535490BD33F81FB77CC4515F1B43018BFA61929488E5` |
| Code（axf） | 121460 字节 |
| 构建方式 | Keil MDK AC6.24（`build/build_ac6.ps1`） |
| 归档时间 | 2026-10-24 |

## 本版改动

只改一处：`user_code/module/gimbal.h`

```
#define YAW_IMU_RATE_PID_MAX_IOUT 2500.0f    /* 原为 1000.0f */
```

即小陀螺 TOP 模式 yaw 角速度内环的积分限幅从 1.0 N·m 提高到 2.5 N·m。

## 机理

1. 底盘自转 7 rad/s 时，yaw 轴维持世界朝向所需的力矩约 1.2 N·m（线缆/摩擦/阻尼）；
2. 旧上限 1.0 N·m 低于该负载，内环积分长期饱和（真机采集 61% 采样 `rate_iout` 顶在上限）；
3. 力矩缺口只能由内环 P 项（Kp=800）承担 → 需要持续约 0.18 rad/s 的速率误差；
4. 外环积分又被 TOP 保持死区分支 `align_state_to_current()` 在误差足够小时反复清零，
   只能以 P 项（Kp=11）输出该速率误差 → 稳态角度误差约 0.015 rad ≈ 0.85°。

上限提到 2.5 N·m 后积分不再饱和，稳态静差消失。

## 真机验证

2026-10-24 21:12，真机 SWD 只读采集 45 s，上档小陀螺（TOP）自转约 1.1 Hz，稳态 42 s 统计。

| 指标 | v2（MAX_IOUT=1000） | v3（本版本） |
|---|---|---|
| mean | +0.851° | **-0.006°** |
| std | 0.359° | 0.220° |
| pp | 1.90° | 1.28° |
| max\|e\| | 1.77° | **0.66°** |
| \|e\|>1° 占比 | 35.6% | 0% |
| 1× 谐波幅值 | 0.219° | 0.057° |
| 内环积分中位 | 1000（= 上限，61% 饱和） | 1182（上限 2500，无饱和） |
| 外环 speed_set 中位 | +0.188 rad/s | 0 |

严格判据 `max|e| <= 1°` 由 1.77° 改善到 0.66°，通过。

## 烧录

```powershell
python -m pyocd flash -t stm32h723xx -O connect_mode=attach hex\gimbal.hex
```

烧录后只读冒烟：`uwTick` 递增、`CFSR = HFSR = 0`、yaw 内环 PID 的 `max_iout = 2.5`。

## 与上一版的差异清单

- v3 相对基线 `2026-10-24_periodic_ff_v2`：仅 `module/gimbal.h` 一行（`YAW_IMU_RATE_PID_MAX_IOUT` 1000 → 2500），
  其余（周期前馈 `top_periodic_ff_a1/b1`、安全补丁）与 v2 相同。
