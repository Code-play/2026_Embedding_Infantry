#ifndef GIMBAL_H
#define GIMBAL_H

//暂时未对导入的库进行分类和删减
#include "imu.h"
#include "Motor.h"
#include "Can_receive.h"
#include "gimbal_task.h"
#include "Communicate.h"
#include "First_order_filter.h"


// /*------------------------速度环pid-----------------------------*/
// yaw IMU角速度内环：反馈单位 rad/s，参数仅供 yaw_imu_rate_pid 使用。
#define YAW_IMU_RATE_PID_KP 800.0f //
#define YAW_IMU_RATE_PID_KI 12.000f
#define YAW_IMU_RATE_PID_KD 50.0f
#define YAW_IMU_RATE_PID_KF 100.0f
#define YAW_IMU_RATE_PID_MAX_IOUT 2500.0f
#define YAW_IMU_RATE_PID_MAX_OUT 15000.0f //15000

// // pitch 速度环
// #define PITCH_SPEED_PID_KP 3333.0f // 2900
// #define PITCH_SPEED_PID_KI 0.0f
// #define PITCH_SPEED_PID_KD 0.0f
// #define PITCH_SPEED_PID_KF 0.0f
// #define PITCH_SPEED_PID_MAX_IOUT 10.0f
// #define PITCH_SPEED_PID_MAX_OUT 20000.0f //15000

// yaw IMU 角度外环：目标 gyro_angle_set、反馈 gyro_angle（rad），输出 speed_set（rad/s）。
// 初值沿用原位置环的 Kp/限幅；先关闭积分、微分和目标增量前馈，再按实机调节。
#define YAW_GYRO_PID_KP 11.0f
#define YAW_GYRO_PID_KI 0.003f
#define YAW_GYRO_PID_KD 0.0f
#define YAW_GYRO_PID_KF 400.0f
#define YAW_GYRO_PID_MAX_IOUT 800.0f
#define YAW_GYRO_PID_MAX_OUT 15000.0f

#define PITCH_MAX_ANGLE 0.22f
#define PITCH_MIN_ANGLE -0.27f
//等待实测
#define PITCH_IMU_MAX_ANGLE 0.30f
#define PITCH_IMU_MIN_ANGLE -0.40f

// IMU yaw 欧拉角映射到云台控制坐标系。实车方向或安装零位不一致时，只调这里。
#define GIMBAL_IMU_YAW_SIGN 1.0f
#define GIMBAL_IMU_YAW_OFFSET_RAD 0.0f
// yaw 欧拉角速度换算的分母保护，仅用于接近俯仰 +/-90 度的奇异区。
#define GIMBAL_IMU_YAW_RATE_MIN_COS_PITCH 0.1f

#define GIMBAL_IMU_PITCH_SIGN 1.0f
#define GIMBAL_IMU_PITCH_OFFSET_RAD 0.0f
#define GIMBAL_IMU_PITCH_CTRL_SIGN (-1.0f)

// 视觉 pitch 误差映射到编码器增量。若 pitch 接管越追越偏，只翻这里。
#define VISION_PITCH_TO_ENCODER_SIGN 1.0f
// 编码器 pitch 上报视觉的零位修正。若视觉弹道需要俯仰零点，调这里。
#define VISION_PITCH_ENCODER_OFFSET_RAD 0.0f

//一阶高通滤波参数
#define GIMBAL_ACCEL_YAW_NUM 0.1666666667f
#define GIMBAL_ACCEL_PITCH_NUM 0.3333333333f
//云台控制周期
#define GIMBAL_CONTROL_TIME ((fp32)GIMBAL_CONTROL_TIME_MS * 0.001f)
//云台摩擦力补偿参数 (单位与电流/扭矩环输出一致)
#define YAW_FRICTION_COMP_MAX 0.3f // 初始给0，由用户逐步增加测试 (例如 200, 500)
#define YAW_FRICTION_SPEED_DEADBAND 0.1f // 速度死区，防止静止时高频抖动
#define TOP_YAW_FF_TEST_ENABLE 1U

// TOP 编码器区间增益调度。置 0 恢复本次改动前的控制行为。
#define TOP_YAW_SECTOR_BOOST_ENABLE 1U
#define TOP_YAW_SECTOR_START_RAD 0.6f
#define TOP_YAW_SECTOR_END_RAD (-2.9f)
#define TOP_YAW_SECTOR_BLEND_RAD 0.20f
#define TOP_YAW_SECTOR_RAMP_TIME_S 0.020f
#define TOP_YAW_SECTOR_ANGLE_KP_SCALE 1.25f  //1.25
#define TOP_YAW_SECTOR_RATE_KP_SCALE 1.60f  //1.8
#define TOP_YAW_SECTOR_HOLD_ERROR_RAD 0.0015f

// 调试器观察：0=原参数，1=区间内完整增强。
extern fp32 top_yaw_sector_weight;

//云台双轴遥控器通道定义
#define YAW_CHANNEL 0
#define PITCH_CHANNEL 1

//云台行为模式判断
#define switch_is_down(x) ((x) == 0)
#define switch_is_mid(x) ((x) == 1)
#define switch_is_up(x)  ((x) == 2)

//遥控器死区
#define RC_DEADBAND 10

//云台 遥控器速度
#define YAW_RC_SEN  -0.00001f // 右手系 z轴逆时针为正 但是遥控器通道向右为正 故加负号
#define PITCH_RC_SEN -0.000004f

//云台 鼠标速度
#define YAW_MOUSE_SEN   -0.0005f
#define PITCH_MOUSE_SEN -0.0003f

//鼠标长按-
#define PRESS_L_LONG_TIME 400
#define PRESS_R_LONG_TIME 5




//云台行为模式
typedef enum
{
    GIMBAL_ZERO_FORCE ,//无力
    GIMBAL_TOP,       //跟随底盘
    GIMBAL_FREE,          //自由
} gimbal_mode_e;



class Gimbal
{
public:
    const VT13_RC_ctrl_t *gimbal_RC; //云台使用的遥控器指针
    VT13_RC_ctrl_t *last_gimbal_RC;  //云台使用的遥控器指针

    bool_t is_shoot;

    //Imu imu;
    DM_Motor gimbal_yaw_motor;   //云台yaw电机数据，speed 始终为电机回传速度
    fp32 yaw_imu_rate_radps;     //姿态换算得到的世界 yaw 角速度，供 yaw 速度环使用
    Pid yaw_imu_rate_pid;        //IMU yaw 角速度内环，输出到 yaw 电机 speed_set
    DM_Motor gimbal_pitch_motor; //云台pitch电机数据

    //First_high_pass_filter gimbal_yaw_high_pass_filter;   //云台yaw电机一阶高通滤波
    //First_high_pass_filter gimbal_pitch_high_pass_filter; //云台pitch电机一阶高通滤波

    uint16_t gimbal_last_key_v; //上一次的按键值
    uint8_t press_r;            //鼠标右键状态（用于自瞄判断）
    uint8_t last_press_r;       //上一次鼠标右键状态
    uint16_t press_r_time;      //鼠标右键按下计时
    uint8_t press_l;            //鼠标左键状态
    uint16_t press_l_time;      //鼠标左键按下计时 
    uint16_t gimbal_mode;
    uint16_t last_gimbal_mode;

    void init();                         //云台初始化
    void set_mode();                     //设置云台控制模式
    void feedback_update();              //云台数据反馈
    void key_state_update();             //按键信息更新
    bool_t gimbal_to_mid();              //云台归中
    void switch_control();               //拨杆控制模式
    void pitch_up_control();             //摩擦轮上电抬头控制
    void mode_change_save();             //模式切换数据保存
    void gimbal_data_update();           //云台数据计算更新
    void set_control();                  //设置云台控制量
    void quintic_output_update();        //更新 quintic 轨迹状态并写入 setpoint 字段（必须在 solve()/output() 之前调用）
    void solve();                        //云台控制PID计算
    void output();                       //输出电流

    /***************************(C) GIMBAL control *******************************/
    void gimbal_top_control(fp32 *yaw, fp32 *pitch);    //陀螺仪模式
    void gimbal_free_control(fp32 *yaw, fp32 *pitch);       //自由模式（IMU 角度外环）
    fp32 gimbal_free_angle_limit(fp32 angle); //自由模式下的角度限幅
    void gimbal_rc_to_control_angle(fp32 *yaw, fp32 *pitch); //遥控器控制云台角度增量
    //常用函数
    int float_to_uint(fp32 x_float, fp32 x_min, fp32 x_max, int bits);
    fp32 uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits);
    fp32 angle_to_rad(fp32 angle);
    /***************************(C) GIMBAL control *******************************/

};




extern Gimbal gimbal;


#endif
