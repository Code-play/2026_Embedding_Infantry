#include "Vision.h"
#include "CRC8_CRC16.h"
#include "gimbal.h"

#include <cstring>

extern IMU imu;

void Vision::clear_lock_qualification()
{
    rt.lock_enter_count = 0U;
    rt.lock_exit_count = 0U;
    rt.fire_qualified = 0U;
}

void Vision::reset_runtime_state(uint8_t clear_last_rx_tick)
{
    if (clear_last_rx_tick != 0U)
    {
        rt.last_rx_tick = 0U;
    }

    rt.fresh = 0U;
    rt.consumed = 1U;
    rt.timeout = 0U;
    rt.invalid_frame = 0U;
    rt.target_lost_count = 0U;
    memset(&yaw_release_rt, 0, sizeof(yaw_release_rt));
    clear_lock_qualification();
    rt.state = VISION_DISABLED;
}

void Vision::init(UART_HandleTypeDef *huart, uint8_t *Rx_buf, uint16_t Rx_buf_size)
{
    this->Rx_Buffer_Size = Rx_buf_size;
    UART_Init(huart, this->Rx_Buffer, this->Rx_Buffer_Size);

    memset(&vision_recv_data, 0, sizeof(vision_recv_data));
    memset(&vision_send_data, 0, sizeof(vision_send_data));

    reset_runtime_state(1U);

    delta_param.vision_timeout_ms = 60U;
    delta_param.lock_enter_threshold = 2U;
    delta_param.lock_exit_threshold = 1U;
    delta_param.target_lost_threshold = 1U;
    // 旧参数：平移场景
    // delta_param.k_yaw = 0.1299998f;
    // delta_param.k_pitch = 0.103000f;
    // delta_param.yaw_step_max = 0.299989f;
    // delta_param.pitch_step_max = 0.132998f;
    // 当前参数：旋转目标，非小陀螺
    delta_param.k_yaw = 0.5120000f;
    delta_param.k_pitch = 0.4439999f;
    delta_param.yaw_step_max = 1.4813299f;
    delta_param.pitch_step_max = 1.432999f;
}

uint8_t vision_send_pack[50] = {0};

void Vision::send()
{
    int i; // 逐字节发送计数
    uint16_t shoot_speed_limit;
    uint16_t bullet_speed;
    const protocol_info_t *imu_info = imu.get_imu_output_info_point();
    //get_shooter_shoot_speed_limit_and_bullet_speed(&shoot_speed_limit, &bullet_speed);
  
    this->vision_send_data.BEGIN = VISION_SEND_BEGIN;
  
    this->vision_send_data.CmdID = 0x01; // 敌方颜色/模式占位

    this->vision_send_data.speed = referee.initial_speed;  
    this->vision_send_data.speed = 25.0f; // 测试用，固定射速 25 m/s

    this->vision_send_data.yaw = ANGLE_TO_RAD * imu_info->yaw;
    // this->vision_send_data.pitch =
    //     VISION_PITCH_TO_ENCODER_SIGN * gimbal.gimbal_pitch_motor.encode_angle +
    //     VISION_PITCH_ENCODER_OFFSET_RAD;
    this->vision_send_data.pitch =
    GIMBAL_IMU_PITCH_SIGN * ANGLE_TO_RAD * imu_info->pitch + GIMBAL_IMU_PITCH_OFFSET_RAD;
    this->vision_send_data.roll = ANGLE_TO_RAD * imu_info->roll;
    // IMU 原始角速度是 deg/s，这里转成视觉协议需要的 rad/s。
    this->vision_send_data.yaw_rate = ANGLE_TO_RAD * imu_info->angle_z;
    //改动
    this->vision_send_data.pitch_rate = GIMBAL_IMU_PITCH_SIGN * ANGLE_TO_RAD * imu_info->angle_y;

    // this->vision_send_data.pitch_rate =
    //     VISION_PITCH_TO_ENCODER_SIGN * gimbal.gimbal_pitch_motor.speed;
    this->vision_send_data.roll_rate = ANGLE_TO_RAD * imu_info->angle_x;
    this->vision_send_data.END = 0xFF;
  
    memcpy(vision_send_pack, &vision_send_data, VISION_SEND_LEN_PACKED);

  // TODO: 后续切回 DMA，避免阻塞控制任务。
   //HAL_UART_Transmit_DMA(VISION_UART, vision_send_pack, VISION_SEND_LEN_PACKED);
  
    // 当前先按字节阻塞发送，方便联调。
				for (i = 0; i < VISION_SEND_LEN_PACKED; i++)
      {
           HAL_UART_Transmit(VISION_UART, &vision_send_pack[i], sizeof(vision_send_pack[0]), 0xFFF);
       }
    
  
    memset(vision_send_pack, 0, 50);
    }
void Vision::unpack()
{ 
    if (Rx_Buffer[0] != VISION_RECV_HEAD0 ||
        Rx_Buffer[1] != VISION_RECV_HEAD1 ||
        verify_CRC16_check_sum(Rx_Buffer, VISION_READ_LEN_PACKED) == 0U)
    {
        rt.invalid_frame = 1U;
        return;
    }

    memcpy(&(this->vision_recv_data), Rx_Buffer, VISION_READ_LEN_PACKED);
    if (this->vision_recv_data.mode > 2U)
    {
        rt.invalid_frame = 1U;
        return;
    }

    rt.last_rx_tick = HAL_GetTick();
    rt.fresh = 1U;
    rt.consumed = 0U;
    rt.timeout = 0U;
    rt.invalid_frame = 0U;

    // 调试用：统计视觉帧间隔。
    // Vision_Time_Test[NOW] = xTaskGetTickCount();
    // Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
    // Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
}

void Vision::vision_get_angle(fp32 *yaw, fp32 *pitch)
{
    *yaw = this->vision_recv_data.yaw;
    *pitch = this->vision_recv_data.pitch;
}

void Vision::vision_get_feedforward(fp32 *yaw_vel, fp32 *yaw_acc, fp32 *pitch_vel, fp32 *pitch_acc)
{
    *yaw_vel = this->vision_recv_data.yaw_vel;
    *yaw_acc = this->vision_recv_data.yaw_acc;
    *pitch_vel = this->vision_recv_data.pitch_vel;
    *pitch_acc = this->vision_recv_data.pitch_acc;
}

uint8_t Vision::vision_fire_requested() const
{
    return (vision_recv_data.mode == 2U) ? 1U : 0U;
}

fp32 Vision::vision_get_distance() const
{
    return vision_recv_data.distance;
}

uint8_t Vision::vision_get_identify_buff() const
{
    return vision_recv_data.identify_buff;
}

vision_led_state_e Vision::get_led_state() const
{
    if (rt.invalid_frame != 0U || rt.timeout != 0U || rt.state == VISION_LOST_HOLD)
    {
        return VISION_LED_ERROR;
    }

    if (rt.state == VISION_TRACK_LOCKED)
    {
        return VISION_LED_LOCKED;
    }

    if (rt.state == VISION_TRACK_UNLOCKED)
    {
        return VISION_LED_TRACKING;
    }

    if (rt.state == VISION_ACQUIRE)
    {
        return VISION_LED_SEARCHING;
    }

    return VISION_LED_OFF;
}

bool_t Vision::vision_if_find_target()
{
	return (vision_recv_data.mode != 0U) ? TRUE : FALSE;
}
