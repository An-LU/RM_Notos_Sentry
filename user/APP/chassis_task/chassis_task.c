#include "chassis_task.h"

#include "rc.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"

#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "Remote_Control.h"
#include "INS_Task.h"

#include "sensor_sw.h"

//底盘信息
static chassis_control chassis_info;

//传感器初始化
void Sensor_Init(void);
void Chassis_PID_Init(void);

void Chassis_Relax_mode(void);
void Chassis_Auto_mode(void);
void Chassis_Rc_mode(void);

static void Chassis_mode_set(void);
void Chassis_Init(void);
void Chassis_Updata(void);
static void Chassis_Control(void);
void Chassis_Send_Current(void);

void chassis_auto_patrol(void);
/***测试***/
void test_dir(void);
fp32 test_speed(void);
/**********/

//主任务
void chassis_task(void *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	//初始化底盘
	Chassis_Init();
	//判断电机和遥控是否在线
	while (toe_is_error(ChassisMotor1TOE) || toe_is_error(DBUSTOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
	while(1)
	{
		//模式  遥控模式  自动模式
		Chassis_mode_set();
		//底盘数据更新  
		Chassis_Updata();
		//底盘控制量计算
		Chassis_Control();
		//发送控制电流
		Chassis_Send_Current();
		//系统延时
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
}

void Chassis_Send_Current(void)
{
	//当底盘模式为无力状态 为了不受干扰 直接发送电流为0
	if(chassis_info.chassis_mode == CHASSIS_RELAX)
	{
		CAN_CMD_CHASSIS(0);
	}	
	else if (!(toe_is_error(ChassisMotor1TOE)))
    {
		CAN_CMD_CHASSIS(chassis_info.chassis_motor.give_current);
	}
}
void Chassis_Init(void)
{
	const static fp32 chassis_order_filter[1] = {CHASSIS_ACCEL_NUM};
	Sensor_Init();					//传感器初始化
	Chassis_PID_Init();				//底盘pid初始化
	
	//底盘低通滤波初始化
	first_order_filter_init(&chassis_info.chassis_first_OF, CHASSIS_CONTROL_TIME, chassis_order_filter);
	//获取遥控器数据
	chassis_info.chassis_RC = get_remote_control_point();	
	//底盘速度限制
	chassis_info.chassis_speed_max = MAX_WHEEL_SPEED;
	chassis_info.chassis_speed_min = -MAX_WHEEL_SPEED;
	//初始化底盘状态
	chassis_info.chassis_mode = CHASSIS_RELAX;
	chassis_info.dir_flag = R;
}

void Chassis_PID_Init(void)
{
	const fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	chassis_info.chassis_motor.chassis_motor_measure = get_Chassis_Motor_Measure_Point(0);//需要修改
	PID_Init(&chassis_info.chassis_speed_pid, PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
}

static void Chassis_mode_set(void)
{
	//遥控器设置底盘模式
	//当遥控器离线的时候，为AUTO模式
	if (toe_is_error(DBUSTOE))
	{
		chassis_info.last_chassis_mode = chassis_info.chassis_mode;
		chassis_info.chassis_mode = CHASSIS_AUTO;//自动
	}
    else if (switch_is_mid(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//中间为遥控
    {
		chassis_info.last_chassis_mode = chassis_info.chassis_mode;
        chassis_info.chassis_mode = CHASSIS_RC;//遥控
    }
    else if (switch_is_up(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))		//上拨为自动
    {
		chassis_info.last_chassis_mode = chassis_info.chassis_mode;
        chassis_info.chassis_mode = CHASSIS_AUTO;//自动
    }
    else if (switch_is_down(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//下拨为无力
    {
		chassis_info.last_chassis_mode = chassis_info.chassis_mode;
        chassis_info.chassis_mode = CHASSIS_RELAX;//底盘无力
    }
}
void Chassis_Updata(void)
{
	//更新电机速度和加速度
	chassis_info.chassis_motor.speed = chassis_info.chassis_motor.chassis_motor_measure->speed_rpm * M3508_MOTOR_RPM_TO_VECTOR;
	chassis_info.chassis_motor.accel = chassis_info.chassis_speed_pid.Dout * CHASSIS_CONTROL_FREQUENCE;
	//更新底盘速度
	chassis_info.chassis_speed = chassis_info.chassis_motor.chassis_motor_measure->speed_rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
}
static void Chassis_Control(void)
{
	//选择控制量输入
	switch(chassis_info.chassis_mode)
	{
		case CHASSIS_RELAX:
		{
			Chassis_Relax_mode();
			break;
		}
		case CHASSIS_AUTO:
		{
			Chassis_Auto_mode();
			break;
		}
		case CHASSIS_RC:
		{
			Chassis_Rc_mode();
			break;
		}
	}
	//pid计算
	PID_Calc(&chassis_info.chassis_speed_pid, chassis_info.chassis_speed, chassis_info.chassis_speed_set);
	//更新电机电流
	chassis_info.chassis_motor.last_give_current = chassis_info.chassis_motor.give_current;
	chassis_info.chassis_motor.give_current = (int16_t)(chassis_info.chassis_speed_pid.out);
}
void Chassis_Relax_mode(void)
{
	chassis_info.chassis_speed = 0.0f;
	chassis_info.chassis_speed_set = 0.0f;
	chassis_info.chassis_motor.give_current = 0.0f;
}
void Chassis_Auto_mode(void)
{
	//初始化相关变量
	uint8_t vision_flag = 0;//Vision_update_flag();			//获取视觉标志位
	//视觉是否识别到
	if( vision_flag )	//识别到目标 底盘停止 CHASSIS_PAUSE
	{
		chassis_info.chassis_speed_set = 0.0f;
	}
	else				//正常巡航 CHASSIS_PATROL
	{
		//test_dir();
		chassis_auto_patrol();
	}
}
void Chassis_Rc_mode(void)
{
	int16_t chassis_move_channel;
	fp32 chassis_move_set;
	//遥控死区限制
	if(chassis_info.chassis_RC->rc.ch[CHASSIS_MOVE_CHANNEL] > CHASSIS_RC_DEADLINE || chassis_info.chassis_RC->rc.ch[CHASSIS_MOVE_CHANNEL] < -CHASSIS_RC_DEADLINE)
		chassis_move_channel = chassis_info.chassis_RC->rc.ch[CHASSIS_MOVE_CHANNEL];
	else
		chassis_move_channel = 0;
	//摇杆转换成速度
	chassis_move_set = chassis_move_channel * CHASSIS_CHANNEL_RC_SEN;
	//一阶低通滤波代替斜坡函数输入
	first_order_filter_cali(&chassis_info.chassis_first_OF, chassis_move_set);
	//停止信号，不需要缓慢加速，直接减速到零
	if (chassis_move_set < CHASSIS_RC_DEADLINE * CHASSIS_CHANNEL_RC_SEN && chassis_move_set > -CHASSIS_RC_DEADLINE * CHASSIS_CHANNEL_RC_SEN)
		chassis_info.chassis_first_OF.out = 0.0f;
	//速度限幅赋值控制量
	chassis_info.chassis_speed_set = fp32_constrain(chassis_info.chassis_first_OF.out, chassis_info.chassis_speed_min, chassis_info.chassis_speed_max);
}
//底盘巡航
void chassis_auto_patrol(void)
{
	uint8_t sw_status = get_Switch_Status(chassis_info.dir_flag);
	//开关未触发 匀速巡航
	if( sw_status )
	{
		fp32 speed = test_speed();
		if(chassis_info.dir_flag == L)
		{
			chassis_info.chassis_speed_set = speed;
			//chassis_info.chassis_speed_set = CHASSIS_SPEED_1;
		}
		else if( chassis_info.dir_flag == R)
		{
			chassis_info.chassis_speed_set = -speed;
			//chassis_info.chassis_speed_set = -CHASSIS_SPEED_1;
		}
		else
		{
			chassis_info.chassis_speed_set = 0;
		}
	}
	//底盘回弹
	else if( !sw_status )
	{
		if(chassis_info.dir_flag == L)
		{
			chassis_info.chassis_speed_set = -CHASSIS_SPEED_2;
			chassis_info.dir_flag = R;
		}
		else if( chassis_info.dir_flag == R)
		{
			chassis_info.chassis_speed_set = CHASSIS_SPEED_2;
			chassis_info.dir_flag = L;
		}
		else
		{
			chassis_info.chassis_speed_set = 0;
		}
	}
	else
	{
		chassis_info.chassis_speed_set = 0;
	}
}

const chassis_control *get_motor_point(void)
{
	return &chassis_info;
}

//模拟开关
void test_dir(void)
{
    if (switch_is_mid(chassis_info.chassis_RC->rc.s[1]))	//中间为遥控
    {
		chassis_info.dir_flag = L;
    }
    else if (switch_is_up(chassis_info.chassis_RC->rc.s[1]))	//上拨为自动
    {
		chassis_info.dir_flag = R;
    }
    else if (switch_is_down(chassis_info.chassis_RC->rc.s[1]))	//下拨为无力
    {
		chassis_info.dir_flag = S;
    }
}
//测试速度
fp32 test_speed(void)
{
    if (switch_is_mid(chassis_info.chassis_RC->rc.s[1]))	//中间
    {
		return -2.0f;
    }
    else if (switch_is_up(chassis_info.chassis_RC->rc.s[1]))	//上拨
    {
		return -2.5f;
    }
    else if (switch_is_down(chassis_info.chassis_RC->rc.s[1]))	//下拨
    {
		return -1.5f;
    }
	else
	{
		return 0.0f;
	}
}




