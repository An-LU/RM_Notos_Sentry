#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"
#include "sensor_sw.h"

//底盘模式选择 右边开关
#define CHASSIS_MODE_SW			0
//底盘控制通道 右边摇杆
#define CHASSIS_MOVE_CHANNEL	0
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define CHASSIS_RC_DEADLINE 10
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//底盘反弹时间间隔 2ms
#define CHASSIS_RETURN_TIME_MS 2
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//在速度更新使用		底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_M3508_CAN_CURRENT 16000.0f
//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例    2Π/60 * r 
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f//0.000798488132787405781442588526584f//0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_M3508_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//底盘匀速速度
#define CHASSIS_SPEED_1		-1.5f
#define CHASSIS_SPEED_2		-1.5f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_CHANNEL_RC_SEN			0.005f
//滤波系数
#define CHASSIS_ACCEL_NUM				0.1666666667f


////底盘前后左右控制按键
//#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
//#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
//#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
//#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D


typedef struct
{
	const motor_measure_t *chassis_motor_measure;	//电调反馈信息
	fp32 accel;				//加速度
	fp32 speed;				//当前速度
	fp32 speed_set;			//设定速度
	int16_t last_give_current;
	int16_t give_current;	//发送的电流值
} Chassis_Motor_t;
/******************************************************/
//typedef enum
//{
//	CHASSIS_PAUSE,				//底盘停顿 识别到目标
//	CHASSIS_PATROL,				//底盘正常巡逻
//}chassis_auto_mode;
//typedef enum
//{
//	L,		//向左运动
//	S,		//无运动
//	R,		//向右运动
//}move_dir;
typedef enum
{
	CHASSIS_RELAX,				//底盘无力
	CHASSIS_AUTO,				//自动模式
	CHASSIS_RC					//底盘遥控控制
}chassis_mode_e;
typedef struct
{
	//gimbal_vision_info;//云台信息
	const RC_ctrl_t *chassis_RC;				//遥控器指针
	chassis_mode_e chassis_mode;				//底盘模式
	chassis_mode_e last_chassis_mode;			//底盘上一次模式
	Chassis_Motor_t chassis_motor;				//底盘电机数据
	PidTypeDef chassis_speed_pid;				//底盘速度环pid
	first_order_filter_type_t chassis_first_OF;	//一阶低通滤波参数
	fp32 chassis_speed;							//底盘速度
	fp32 chassis_speed_set;						//底盘速度设定
	fp32 chassis_speed_max;						//底盘最大速度
	fp32 chassis_speed_min;						//底盘最小速度
	//move_dir last_dir_flag;						//底盘上一次运动方向标志
	move_dir dir_flag;							//底盘运动方向标志
}chassis_control;
/******************************************************/
extern void chassis_task(void *pvParameters);
//extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_control *chassis_move_rc_to_vector);
extern const chassis_control *get_motor_point(void);
#endif
