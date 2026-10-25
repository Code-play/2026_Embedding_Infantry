# 2026_Embedding_Infantry · 步兵固件（实机）

浙江工业大学 DEUS 战队 2026 赛季步兵机器人固件源码与工程。

本仓库**只包含实机固件**：CubeMX/Keil 工程、用户代码（`user_code/`）与已编译的烧录 hex。
不含仿真、建模、自动调参等工具链代码。

## 当前版本：v3 = `2026-10-26_top_dc_fix`

- **改动**：`gimbal/user_code/module/gimbal.h` 中 `YAW_IMU_RATE_PID_MAX_IOUT` 由 `1000` 改为 `2500`
  （小陀螺 TOP 模式下 yaw 角速度内环的积分上限 $1.0\to2.5$ N·m）。
- **原因**：底盘自转 7 rad/s 时 yaw 轴的保持负载约 1.2 N·m，旧上限 1.0 N·m 导致内环积分长期饱和
  （真机采集里 61% 的采样顶在上限），力矩缺口只能由内环 P 项承担 → 需要约 0.18 rad/s 的持续速率误差；
  而外环积分又会被 TOP 保持死区分支 `align_state_to_current()` 反复清零，只能以 P 项输出该速率误差
  → 稳态角度误差约 $0.015$ rad（$0.85^\circ$）。
- 其余代码与上一版（周期前馈 + 安全补丁）一致。

真机验证见 `MANIFEST.md`。

## 目录结构

```
gimbal/                  Keil / CubeMX 工程
  Core/                  CubeMX 生成的初始化与中断
  Drivers/               STM32H7 HAL 驱动
  Middlewares/           FreeRTOS 等中间件
  MDK-ARM/               Keil 工程文件（gimbal.uvprojx / gimbal.uvoptx / startup）
  user_code/             用户代码（算法 / 应用任务 / 设备 / 通信协议）
  gimbal.ioc             CubeMX 工程配置
hex/gimbal.hex           已编译的 v3 固件，可直接烧录
build/                   AC6（armclang）构建工具
  build_ac6.ps1          一键构建：复制工程 → AC6 迁移补丁 → uVision 编译
  apply_ac6_patches.py   AC6 迁移补丁（weak 回调 / __packed / 结构体静态断言）
  gimbal.uvprojx         AC6 迁移后的 Keil 工程
  FreeRTOS_GCC_ARM_CM4F  FreeRTOS 的 GCC/ARM_CM4F 移植层
```

## 构建

方式一（原始工程，AC5）：Keil MDK 打开 `gimbal/MDK-ARM/gimbal.uvprojx` 直接编译。

方式二（AC6，本仓库 hex 的构建方式）：Keil MDK ≥ 5.37（AC6.24）+ Python 3.11：

```powershell
pwsh -File build\build_ac6.ps1 `
  -Tree   <仓库路径>\gimbal `
  -UserCode <仓库路径>\gimbal\user_code `
  -OutDir <输出目录>
```

## 烧录

```powershell
python -m pyocd flash -t stm32h723xx -O connect_mode=attach hex\gimbal.hex
```

- hex 大小：348323 字节
- hex SHA256：`739283C62FA5921B73EA535490BD33F81FB77CC4515F1B43018BFA61929488E5`

烧录后建议做一次只读冒烟：`uwTick` 递增、`CFSR = HFSR = 0`、`YAW_IMU_RATE_PID_MAX_IOUT = 2500`（RAM 中对应 PID 的 `max_iout` 为 2.5）。

## 历史版本

| 版本 | 标签 | 说明 |
|---|---|---|
| v2 | `2026-10-24_periodic_ff_v2` | TOP yaw 周期前馈（1 次谐波 $\cos(\theta+\varphi)$ 拟合，系数按真机标定）+ 安全补丁（无力档清空速度目标、遥控失联超时强制无力） |
| v3 | `2026-10-26_top_dc_fix` | 本次：内环积分上限 1000 → 2500，消除小陀螺稳态静差 |
