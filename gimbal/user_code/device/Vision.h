#ifndef VISION_H
#define VISION_H

#include "Referee.h"
#include "bsp_uart.h"
#include "imu.h"
#include "main.h"

#define UART_VISION USART1
#define VISION_UART (&huart1)
#define VISION_RX_BUFFER_SIZE VISION_READ_LEN_PACKED

#define VISION_SEND_BEGIN (0xA5)
#define VISION_RECV_HEAD0 ('D')
#define VISION_RECV_HEAD1 ('S')

#define VISION_SEND_LEN_PACKED 31 // 电控发给视觉：姿态 + 弹速
#define VISION_READ_LEN_PACKED 34 // 视觉发给电控：v34 delta 控制包

#define VISION_ABS_TIMEOUT_MS 150U 

#define VISION_CTRL_MODE_ABS 0
#define VISION_CTRL_MODE_DELTA 1
#define VISION_CTRL_MODE  VISION_CTRL_MODE_ABS                                                     \
 // VISION_CTRL_MODE_DELTA // 当前主链路：视觉 yaw/pitch 按误差角 delta 使用

typedef enum {
  VISION_MANU = 0,
  VISION_BUFF = 1,
  VISION_AUTO = 2,
} VisionActData_t; // 视觉模式选择

typedef enum {
  VISION_DISABLED = 0,
  VISION_ACQUIRE,
  VISION_TRACK_UNLOCKED,
  VISION_TRACK_LOCKED,
  VISION_LOST_HOLD,
} vision_ctrl_state_e;

typedef enum {
  VISION_LED_OFF = 0,
  VISION_LED_ERROR,
  VISION_LED_SEARCHING,
  VISION_LED_TRACKING,
  VISION_LED_LOCKED,
} vision_led_state_e;

// 视觉 -> 电控 34 字节控制包。yaw/pitch 是误差角 delta，不是云台绝对角。
typedef __packed struct {
  uint8_t head[2]; // 帧头，“D”“S”象征着”Deus“

  uint8_t
      mode; // 控制模式: 0=不接管云台, 1=接管云台但不开火, 2=接管云台并请求开火

  float yaw; // yaw 误差角 delta，单位 rad

  float yaw_vel; // yaw 速度前馈，单位 rad/s
  float yaw_acc; // yaw 加速度前馈，单位 rad/s^2

  float pitch;     // pitch 误差角 delta，单位 rad
  float pitch_vel; // pitch 速度前馈，单位 rad/s
  float pitch_acc; // pitch 加速度前馈，单位 rad/s^2

  float distance;        // 目标水平距离
  uint8_t identify_buff; // 打符(能量机关)目标标记
  uint16_t crc16;        // CRC16 校验
} VisionRecvData_t;

// static_assert(sizeof(VisionRecvData_t) == VISION_READ_LEN_PACKED,
//               "VisionRecvData_t must stay 34 bytes");

// 电控 -> 视觉 31 字节反馈包
typedef __packed struct {
  uint8_t BEGIN; // 帧头起始位,暂定0xA5
  uint8_t CmdID; // 颜色

  fp32 speed; // 射速

  fp32 yaw;

  fp32 pitch;

  fp32 roll;

  fp32 yaw_rate; // yaw 角速度，rad/s

  fp32 pitch_rate; // pitch 角速度，rad/s

  fp32 roll_rate; // roll 角速度，rad/s

  uint8_t END;

} VisionSendData_t;

// static_assert(sizeof(VisionSendData_t) == VISION_SEND_LEN_PACKED,
//               "VisionSendData_t must stay 31 bytes");

typedef struct {
  volatile uint32_t last_rx_tick;
  volatile uint8_t fresh;
  volatile uint8_t consumed;
  volatile uint8_t timeout;
  volatile uint8_t invalid_frame;
  uint8_t lock_enter_count;
  uint8_t lock_exit_count;
  uint8_t target_lost_count;
  volatile uint8_t fire_qualified;
  vision_ctrl_state_e state;
} VisionRuntimeState;

#define VISION_YAW_RELEASE_SLOTS 4U

typedef struct {
  fp32 slots[VISION_YAW_RELEASE_SLOTS];
  uint8_t index;
} VisionYawReleaseRuntime;

typedef struct {
  uint16_t vision_timeout_ms;
  uint8_t lock_enter_threshold;
  uint8_t lock_exit_threshold;
  uint8_t target_lost_threshold;
  fp32 k_yaw;
  fp32 k_pitch;
  fp32 yaw_step_max;
  fp32 pitch_step_max;
} VisionDeltaParam;

class Vision {
public:
  VisionRecvData_t vision_recv_data;
  VisionSendData_t vision_send_data;
  VisionRuntimeState rt;
  VisionYawReleaseRuntime yaw_release_rt;
  VisionDeltaParam delta_param;

  // 数据接收
  uint8_t Rx_Buffer[VISION_READ_LEN_PACKED];
  uint16_t Rx_Buffer_Size;

  void init(UART_HandleTypeDef *huart, uint8_t *Rx_buf, uint16_t Rx_buf_size);
  void send();
  void unpack();
  void vision_get_angle(fp32 *yaw, fp32 *pitch);
  void vision_get_feedforward(fp32 *yaw_vel, fp32 *yaw_acc, fp32 *pitch_vel,
                              fp32 *pitch_acc);
  uint8_t vision_fire_requested() const;
  fp32 vision_get_distance() const;
  uint8_t vision_get_identify_buff() const;
  void reset_runtime_state(uint8_t clear_last_rx_tick);
  void clear_lock_qualification();
  vision_led_state_e get_led_state() const;

  bool_t vision_if_find_target();
};

extern Referee referee;

#endif
