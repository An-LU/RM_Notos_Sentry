#include "Gimbal_Task.h"

#include "main.h"

#include "arm_math.h"
//#include "gimbal_behaviour.h"
//#include "user_lib.h"
#include "INS_Task.h"
#include "remote_control.h"
#include "shoot.h"
#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//角度规整
#define ECD_format(ecd,ecd_del)						\
	{												\
		if( (ecd) <= 8191 && (ecd) >= (ecd_del) )	\
			(ecd) -= (ecd_del);						\
		else if( (ecd) < (ecd_del) )				\
			(ecd) = (ecd) - (ecd_del) + 8192;		\
	}

//#if INCLUDE_uxTaskGetStackHighWaterMark	//查看任务堆栈剩余容量
//uint32_t gimbal_high_water;
//#endif

//云台控制所有相关数据
static Gimbal_Control_s gimbal_info;
//发送的云台can 指令
static int16_t Yaw_Can_Set_Voltage = 0, Pitch_Can_Set_Voltage = 0, Shoot_Can_Set_Voltage = 0;

static void Gimbal_Init(void);
static void Gimbal_PID_Init(void);
static void Gimbal_mode_set(void);
static void Gimbal_Updata(void);
static void Gimbal_Control(void);
static void Gimbal_Send_Voltage(void);

static void Gimbal_Calibration_mode(void);
static void Gimbal_Relax_mode(void);
static void Gimbal_Auto_mode(fp32 *, fp32 *);
static void Gimbal_Rc_mode(fp32 *, fp32 *);
	
static void gimbal_pid_calculation(Gimbal_Mode_e gimbal_mode);
static void Angle_Limit(void);
static fp32 relative_angle_change(uint16_t last_ecd, uint16_t ecd, uint16_t offset_ecd, uint8_t *turn_table_flag, uint16_t ecd_turn);

void GIMBAL_task(void *pvParameters)
{
    //等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
    Gimbal_Init();
    //射击初始化
    shoot_init();
    //判断电机是否都上线
    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE)|| toe_is_error(TriggerMotorTOE) )
    {
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);
        Gimbal_Updata();             //云台数据反馈
    }

    while (1)
    {
		//模式 遥控 自动 无力
		Gimbal_mode_set();
		//云台数据更新
		Gimbal_Updata();
		//云台控制量计算
		Gimbal_Control();
		//更新射击电流
        Shoot_Can_Set_Voltage = shoot_control_loop();  
		//发送控制电压
		Gimbal_Send_Voltage();
		//系统延时
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);

//#if INCLUDE_uxTaskGetStackHighWaterMark
//        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif
    }
}


//发送控制电压
static void Gimbal_Send_Voltage(void)
{
	//电机是否反装
#if YAW_TURN
	Yaw_Can_Set_Voltage = -gimbal_info.yaw_motor.give_voltage;
#else
	Yaw_Can_Set_Voltage = gimbal_info.yaw_motor.give_voltage;
#endif

#if PITCH_TURN
	Pitch_Can_Set_Voltage = -gimbal_info.pitch_motor.give_voltage;
#else
	Pitch_Can_Set_Voltage = gimbal_info.pitch_motor.give_voltage;
#endif

	//云台在遥控器掉线状态即relax 状态，can指令为0，不使用Voltage设置为零的方法，是保证遥控器掉线一定使得云台停止
	if (!(toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE) && toe_is_error(TriggerMotorTOE)))
	{
		//当底盘模式为无力状态 为了不受干扰 直接发送电流为0	
		if (gimbal_info.gimbal_mode == GIMBAL_RELAX)
		{
			CAN_CMD_GIMBAL(0, 0, 0, 0);
		}
		else
		{
			//CAN_CMD_GIMBAL(0, Pitch_Can_Set_Voltage, Shoot_Can_Set_Voltage, 0);
			//CAN_CMD_GIMBAL(Yaw_Can_Set_Voltage, Pitch_Can_Set_Voltage, Shoot_Can_Set_Voltage, 0);
			CAN_CMD_GIMBAL(0, 0, 0, 0);
		}
	}
}
//云台初始化
static void Gimbal_Init()
{
	const static fp32 gimbal_pitch_order_filter[1] = {GIMBAL_PITCH_ACCEL_NUM};
	const static fp32 gimbal_yaw_order_filter[1] = {GIMBAL_YAW_ACCEL_NUM};
	//PID初始化
	Gimbal_PID_Init();
	//滤波初始化
	first_order_filter_init(&gimbal_info.pitch_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_pitch_order_filter);
	first_order_filter_init(&gimbal_info.yaw_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_yaw_order_filter);
	//获取遥控器数据
	gimbal_info.gimbal_RC = get_remote_control_point();
	//获取电机数据
	gimbal_info.yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_info.pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	//更新初始角度
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.gimbal_motor_measure->last_ecd;
	gimbal_info.pitch_motor.ecd_now = gimbal_info.pitch_motor.gimbal_motor_measure->ecd;
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.gimbal_motor_measure->last_ecd;
	gimbal_info.yaw_motor.ecd_now = gimbal_info.yaw_motor.gimbal_motor_measure->ecd;
	
	gimbal_info.pitch_motor.angle = gimbal_info.pitch_motor.angle_set = relative_angle_change( gimbal_info.pitch_motor.gimbal_motor_measure->last_ecd, 
							gimbal_info.pitch_motor.gimbal_motor_measure->ecd, PITCH_ECD_DEL, &gimbal_info.pitch_motor.turn_table_flag, PITCH_ECD_TURN);
	gimbal_info.yaw_motor.angle = gimbal_info.yaw_motor.angle_set = relative_angle_change(gimbal_info.yaw_motor.gimbal_motor_measure->last_ecd, 
							gimbal_info.yaw_motor.gimbal_motor_measure->ecd, YAW_ECD_DEL, &gimbal_info.yaw_motor.turn_table_flag, YAW_ECD_TURN);
	//初始化云台圈数
	gimbal_info.turn_circle_num = 0;
	//获取自瞄数据
	gimbal_info.gimbal_AUTO_ctrl = get_AUTO_control_point();
	//最大角度限制
	gimbal_info.yaw_motor.angle_max = (YAW_ECD_MAX - YAW_ECD_DEL) * Ecd_to_Rad;
	gimbal_info.yaw_motor.angle_min = -1.6400f;//( YAW_ECD_MIN - YAW_ECD_DEL) * Ecd_to_Rad;//
	gimbal_info.pitch_motor.angle_max = (PITCH_ECD_MAX - PITCH_ECD_DEL) * Ecd_to_Rad;
	gimbal_info.pitch_motor.angle_min = (PITCH_ECD_MIN - PITCH_ECD_DEL) * Ecd_to_Rad;
}
//云台PID初始化
static void Gimbal_PID_Init(void)
{
	//速度环 内环
	const fp32 yaw_motor_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
	const fp32 pitch_motor_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
	//角度环 外环
	const fp32 yaw_motor_angle_pid[3] = {YAW_ANGLE_PID_KP, YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD};
	const fp32 pitch_motor_angle_pid[3] = {PITCH_ANGLE_PID_KP, PITCH_ANGLE_PID_KI, PITCH_ANGLE_PID_KD};
	//自瞄角度环 外环
	const fp32 yaw_motor_auto_pid[3] = {YAW_AUTO_PID_KP, YAW_AUTO_PID_KI, YAW_AUTO_PID_KD};
	const fp32 pitch_motor_auto_pid[3] = {PITCH_AUTO_PID_KP, PITCH_AUTO_PID_KI, PITCH_AUTO_PID_KD};
	
	PID_Init(&gimbal_info.yaw_motor.motor_speed_pid, PID_POSITION, yaw_motor_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT );
	PID_Init(&gimbal_info.yaw_motor.motor_angle_pid, PID_POSITION, yaw_motor_angle_pid, YAW_ANGLE_PID_MAX_OUT, YAW_ANGLE_PID_MAX_IOUT );
	PID_Init(&gimbal_info.yaw_motor.motor_auto_pid, PID_POSITION, yaw_motor_auto_pid, YAW_AUTO_PID_MAX_OUT, YAW_AUTO_PID_MAX_IOUT );
	
	PID_Init(&gimbal_info.pitch_motor.motor_speed_pid, PID_POSITION, pitch_motor_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT );
	PID_Init(&gimbal_info.pitch_motor.motor_angle_pid, PID_POSITION, pitch_motor_angle_pid, PITCH_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT );
	PID_Init(&gimbal_info.pitch_motor.motor_auto_pid, PID_POSITION, pitch_motor_auto_pid, PITCH_AUTO_PID_MAX_OUT, PITCH_AUTO_PID_MAX_IOUT );
}
//云台模式选择
static void Gimbal_mode_set(void)
{
	//当遥控器离线的时候，为AUTO模式
	if (toe_is_error(DBUSTOE))
	{
		gimbal_info.gimbal_mode = GIMBAL_AUTO;//自动
	}
    else if (switch_is_mid(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))		//中间为遥控
    {
        gimbal_info.gimbal_mode = GIMBAL_RC;//遥控
    }
    else if (switch_is_up(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))		//上拨为自动
    {
        gimbal_info.gimbal_mode = GIMBAL_AUTO;//自动
    }
    else if (switch_is_down(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))	//下拨为无力
    {
        gimbal_info.gimbal_mode = GIMBAL_RELAX;//云台无力
    }
}
//相对角度
static fp32 relative_angle_change(uint16_t last_ecd, uint16_t ecd, uint16_t offset_ecd, uint8_t *turn_table_flag, uint16_t ecd_turn)
{
	int32_t relative_ecd = 0;
	int16_t err_ecd;
	//计算相对机械角度
	ECD_format(ecd, offset_ecd);
	ECD_format(last_ecd, offset_ecd);
	//机械角度差
	err_ecd = ecd - last_ecd;
	//计算角度码盘
	if (err_ecd > 6000 || (!gimbal_info.turn_mid_flag && ecd < 8192 && ecd > 6000))		//电机反向转过起始点 0(8191) 相对位置应该为负数
	{
		*turn_table_flag = 0;
	}
	else if (err_ecd < -6000 || (!gimbal_info.turn_mid_flag && ecd <= 6000))//&& ecd >= 0 电机正转经过起始点 8191(0) 相对位置应该为正数
	{
		*turn_table_flag = 1;
	}
	if (*turn_table_flag)		//正角度码盘 (0,2PI)
	{
		relative_ecd = ecd;
	}
	else
	{
		relative_ecd = ecd - 8192;
	}
	//电机编码值转换为rad
	return relative_ecd * Ecd_to_Rad;
}
//计算云台圈数和实际角度
static void calc_turn_angle(fp32 *angle_last, fp32 *angle_now)
{
	//正向圈数为正 逆时针
	if(*angle_now - *angle_last < -4.0f)
	{
		gimbal_info.turn_circle_num++;
		*angle_now += 2 * PI;
	}
	//反向圈数为负 顺时针
	else if(*angle_now - *angle_last > 4.0f)
	{
		gimbal_info.turn_circle_num--;
		*angle_now -= 2 * PI;
	}
}
//云台数据更新
static void Gimbal_Updata(void)
{
	//fp32 angle_temp = gimbal_info.yaw_motor.angle;
	const fp32 *gimbal_INT_angle_point = get_INS_angle_point();//获取陀螺仪姿态解算后的数据 角度
	const fp32 *gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();//获取陀螺仪原始数据 角速度
	//角度更新  外环
	gimbal_info.pitch_motor.angle_last = gimbal_info.pitch_motor.angle;//angle_pitch_temp;
	gimbal_info.yaw_motor.angle_last = gimbal_info.yaw_motor.angle;
	//更新机械角度(ecd)和相对角度(angle)
	gimbal_info.pitch_motor.ecd_now = gimbal_info.pitch_motor.gimbal_motor_measure->ecd;
	gimbal_info.yaw_motor.ecd_now = gimbal_info.yaw_motor.gimbal_motor_measure->ecd;
	gimbal_info.pitch_motor.angle = relative_angle_change(gimbal_info.pitch_motor.ecd_last, gimbal_info.pitch_motor.ecd_now, 
											PITCH_ECD_DEL, &gimbal_info.pitch_motor.turn_table_flag, PITCH_ECD_TURN);
	gimbal_info.yaw_motor.angle = relative_angle_change( gimbal_info.yaw_motor.ecd_last, gimbal_info.yaw_motor.ecd_now, 
											YAW_ECD_DEL, &gimbal_info.yaw_motor.turn_table_flag, YAW_ECD_TURN) + gimbal_info.turn_circle_num * 2 * PI;
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.ecd_now;
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.ecd_now;
	//计算yaw轴电机圈数
	calc_turn_angle(&gimbal_info.yaw_motor.angle_last, &gimbal_info.yaw_motor.angle);
	//角速度更新  内环
	//gimbal_info.pitch_motor.speed = *(gimbal_INT_gyro_point + INS_PITCH_ADDRESS_OFFSET);		//陀螺仪能够跟随云台pitch轴
	gimbal_info.pitch_motor.speed = gimbal_info.pitch_motor.gimbal_motor_measure->speed_rpm * Ecd_to_Rad;	//否则则用电机反馈的转速值rmp
	//gimbal_info.yaw_motor.speed = *(gimbal_INT_gyro_point + INS_YAW_ADDRESS_OFFSET);
	gimbal_info.yaw_motor.speed = gimbal_info.yaw_motor.gimbal_motor_measure->speed_rpm * Ecd_to_Rad;
	//电压更新
	gimbal_info.yaw_motor.last_give_voltage = gimbal_info.yaw_motor.give_voltage;
	gimbal_info.pitch_motor.last_give_voltage = gimbal_info.pitch_motor.give_voltage;
}
//控制量输入
static void Gimbal_Control(void)
{
	fp32 add_yaw_angle = 0.0f;
	fp32 add_pitch_angle = 0.0f;
	//选择控制量输入
	switch(gimbal_info.gimbal_mode)
	{
		case GIMBAL_CALI:
		{
			Gimbal_Calibration_mode();
			return;
		}
		case GIMBAL_RELAX:
		{
			Gimbal_Relax_mode();
			break;
		}
		case GIMBAL_AUTO:
		{
			Gimbal_Auto_mode(&add_pitch_angle, &add_yaw_angle);
			break;
		}
		case GIMBAL_RC:
		{
			Gimbal_Rc_mode(&add_pitch_angle, &add_yaw_angle);
			break;
		}
	}
	//回中处理
	if( !gimbal_info.turn_mid_flag )
	{
		//死区处理
		if(gimbal_info.pitch_motor.angle > 0.1f || gimbal_info.pitch_motor.angle < -0.1f)
		{
			add_pitch_angle = gimbal_info.pitch_motor.angle < 0.0f ? GIMBAL_RETURN_PITCH : -GIMBAL_RETURN_PITCH;		//先抬起pitch轴
			add_yaw_angle = 0.0f;
		}
		else if(gimbal_info.yaw_motor.angle > 0.1f || gimbal_info.yaw_motor.angle < -0.1f)
		{
			add_pitch_angle = 0.0f;
			add_yaw_angle = gimbal_info.yaw_motor.angle < 0.0f ? GIMBAL_RETURN_YAW : -GIMBAL_RETURN_YAW;
		}
		else	//已回中
		{
			gimbal_info.turn_mid_flag = 1;
		}
		//角度叠加
		gimbal_info.pitch_motor.angle_set += add_pitch_angle;
		gimbal_info.yaw_motor.angle_set += add_yaw_angle;
		if(gimbal_info.turn_mid_flag)
		{
//			gimbal_info.pitch_motor.angle_set = 0.0f;
//			gimbal_info.yaw_motor.angle_set = 0.0f;
		}
	}
	else
	{
		//角度叠加
		gimbal_info.pitch_motor.angle_set += add_pitch_angle;
		gimbal_info.yaw_motor.angle_set += add_yaw_angle;
		//角度限幅
		Angle_Limit();
	}
	//PID计算
	gimbal_pid_calculation(gimbal_info.gimbal_mode);
	//电压
	gimbal_info.yaw_motor.give_voltage = (int16_t)gimbal_info.yaw_motor.voltage_set;
	gimbal_info.pitch_motor.give_voltage = (int16_t)gimbal_info.pitch_motor.voltage_set;
}
static void Gimbal_Calibration_mode(void)
{
	return;
}
//云台无力模式
static void Gimbal_Relax_mode(void)
{
	//pitch
	gimbal_info.pitch_motor.speed_set = 0.0f;
	gimbal_info.pitch_motor.angle_set = gimbal_info.pitch_motor.angle;//0.0f;
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.ecd_now;
	gimbal_info.pitch_motor.voltage_set = 0.0f;
	gimbal_info.pitch_motor.give_voltage = 0.0f;
	//yaw
	gimbal_info.yaw_motor.speed_set = 0.0f;
	gimbal_info.yaw_motor.angle_set = gimbal_info.yaw_motor.angle;//0.0f;
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.ecd_now;
	gimbal_info.yaw_motor.voltage_set = 0.0f;
	gimbal_info.yaw_motor.give_voltage = 0.0f;
	//回中标志清0
	gimbal_info.turn_mid_flag = 0;
	gimbal_info.turn_circle_num = 0;
}
//云台自动模式
static void Gimbal_Auto_mode(fp32 *pitch_add, fp32 *yaw_add)
{
	//初始化相关变量
	static uint8_t run_flag_pitch;	//0反方向 1正反向
	static uint8_t run_flag_yaw;
	uint8_t vision_flag = 0;//Vision_update_flag();			//获取视觉标志位
	//gimbal_info.turn_mid_flag = 0;
	//打开摩擦轮
	
	//视觉是否识别到
	if( vision_flag )	//识别到目标 云台由视觉控制
	{
		*yaw_add = gimbal_info.gimbal_AUTO_ctrl->yaw_angle * YAW_AUTO_SEN;
		*pitch_add = gimbal_info.gimbal_AUTO_ctrl->pitch_angle * PITCH_AUTO_SEN;
		Vision_clean_flag();	
		//目标值与当前值误差为0.5f时开火 
		if(0)
		{
			
		}
	}
	else				//云台正常巡航 
	{
		//正常巡航停火
		
		//当达到最大角度
		if(gimbal_info.pitch_motor.angle_set + AUTO_PATROL_SPEED_PITCH > gimbal_info.pitch_motor.angle_max || gimbal_info.pitch_motor.angle_set - AUTO_PATROL_SPEED_PITCH < gimbal_info.pitch_motor.angle_min)
		{
			run_flag_pitch = ~run_flag_pitch;
		}
		if(gimbal_info.yaw_motor.angle_set + AUTO_PATROL_SPEED_YAW > gimbal_info.yaw_motor.angle_max || gimbal_info.yaw_motor.angle_set - AUTO_PATROL_SPEED_YAW < gimbal_info.yaw_motor.angle_min)
		{
			run_flag_yaw = ~run_flag_yaw;
		}
		*pitch_add = run_flag_pitch ? AUTO_PATROL_SPEED_PITCH : -AUTO_PATROL_SPEED_PITCH;
		*yaw_add = run_flag_yaw ? AUTO_PATROL_SPEED_YAW : -AUTO_PATROL_SPEED_YAW;
	}
}
//云台遥控控制
static void Gimbal_Rc_mode(fp32 *pitch_add, fp32 *yaw_add)
{
	//初始化相关变量
	int16_t yaw_channel = 0;
	int16_t pitch_channel = 0;
	//遥控器死区处理
	if( gimbal_info.gimbal_RC->rc.ch[YAW_CHANNEL] > GIMBAL_RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[YAW_CHANNEL] < -GIMBAL_RC_DEADLINE )
		yaw_channel = gimbal_info.gimbal_RC->rc.ch[YAW_CHANNEL];
	if( gimbal_info.gimbal_RC->rc.ch[PITCH_CHANNEL] > GIMBAL_RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[PITCH_CHANNEL] < -GIMBAL_RC_DEADLINE )
		pitch_channel = gimbal_info.gimbal_RC->rc.ch[PITCH_CHANNEL];
	//遥控数据比例转换
	*yaw_add = yaw_channel * YAW_RC_SEN;
	*pitch_add = pitch_channel * PITCH_RC_SEN;
}
//云台PID计算
static void gimbal_pid_calculation(Gimbal_Mode_e gimbal_mode)
{
	if( gimbal_mode == GIMBAL_AUTO)
	{
		//PID计算 角度环为外环 速度环为内环
		gimbal_info.pitch_motor.speed_set = PID_Calc(&gimbal_info.pitch_motor.motor_auto_pid, 
														gimbal_info.pitch_motor.angle, gimbal_info.pitch_motor.angle_set );
		gimbal_info.pitch_motor.voltage_set = PID_Calc(&gimbal_info.pitch_motor.motor_speed_pid, 
														gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set );
		gimbal_info.yaw_motor.speed_set = PID_Calc(&gimbal_info.yaw_motor.motor_auto_pid, 
														gimbal_info.yaw_motor.angle, gimbal_info.yaw_motor.angle_set );
		gimbal_info.yaw_motor.voltage_set = PID_Calc(&gimbal_info.yaw_motor.motor_speed_pid, 
														gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set );
	}
	else if( gimbal_mode == GIMBAL_RC )
	{
		//PID计算 角度环为外环 角速度环为内环
		gimbal_info.pitch_motor.speed_set = PID_Calc(&gimbal_info.pitch_motor.motor_angle_pid, 
														rad_format(gimbal_info.pitch_motor.angle), rad_format(gimbal_info.pitch_motor.angle_set) );
		gimbal_info.pitch_motor.voltage_set = PID_Calc(&gimbal_info.pitch_motor.motor_speed_pid, 
														gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set );
		gimbal_info.yaw_motor.speed_set = PID_Calc(&gimbal_info.yaw_motor.motor_angle_pid, 
														gimbal_info.yaw_motor.angle, gimbal_info.yaw_motor.angle_set );
		gimbal_info.yaw_motor.voltage_set = PID_Calc(&gimbal_info.yaw_motor.motor_speed_pid, 
														gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set );
	}
	else
	{
		return;
	}
}
//角度限制
static void Angle_Limit(void)
{
	if( gimbal_info.pitch_motor.angle_set > gimbal_info.pitch_motor.angle_max )
	{
		gimbal_info.pitch_motor.angle_set = gimbal_info.pitch_motor.angle_max;
	}
	else if( gimbal_info.pitch_motor.angle_set < gimbal_info.pitch_motor.angle_min )
	{
		gimbal_info.pitch_motor.angle_set = gimbal_info.pitch_motor.angle_min;
	}
//	if( gimbal_info.yaw_motor.angle_set > gimbal_info.yaw_motor.angle_max )
//	{
//		gimbal_info.yaw_motor.angle_set = gimbal_info.yaw_motor.angle_max;
//	}
//	else if( gimbal_info.yaw_motor.angle_set < gimbal_info.yaw_motor.angle_min )
//	{
//		gimbal_info.yaw_motor.angle_set = gimbal_info.yaw_motor.angle_min;
//	}
}

//static void Angle_Format_PI(fp32 *angle, fp32 *last_angle)
