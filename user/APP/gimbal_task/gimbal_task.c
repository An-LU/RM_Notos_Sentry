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

//�Ƕȹ���
#define ECD_format(ecd,ecd_del)						\
	{												\
		if( (ecd) <= 8191 && (ecd) >= (ecd_del) )	\
			(ecd) -= (ecd_del);						\
		else if( (ecd) < (ecd_del) )				\
			(ecd) = (ecd) - (ecd_del) + 8192;		\
	}

//#if INCLUDE_uxTaskGetStackHighWaterMark	//�鿴�����ջʣ������
//uint32_t gimbal_high_water;
//#endif

//��̨���������������
static Gimbal_Control_s gimbal_info;
//���͵���̨can ָ��
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
    //�ȴ������������������������
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //��̨��ʼ��
    Gimbal_Init();
    //�����ʼ��
    shoot_init();
    //�жϵ���Ƿ�����
    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE)|| toe_is_error(TriggerMotorTOE) )
    {
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);
        Gimbal_Updata();             //��̨���ݷ���
    }

    while (1)
    {
		//ģʽ ң�� �Զ� ����
		Gimbal_mode_set();
		//��̨���ݸ���
		Gimbal_Updata();
		//��̨����������
		Gimbal_Control();
		//�����������
        Shoot_Can_Set_Voltage = shoot_control_loop();  
		//���Ϳ��Ƶ�ѹ
		Gimbal_Send_Voltage();
		//ϵͳ��ʱ
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);

//#if INCLUDE_uxTaskGetStackHighWaterMark
//        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif
    }
}


//���Ϳ��Ƶ�ѹ
static void Gimbal_Send_Voltage(void)
{
	//����Ƿ�װ
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

	//��̨��ң��������״̬��relax ״̬��canָ��Ϊ0����ʹ��Voltage����Ϊ��ķ������Ǳ�֤ң��������һ��ʹ����ֹ̨ͣ
	if (!(toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE) && toe_is_error(TriggerMotorTOE)))
	{
		//������ģʽΪ����״̬ Ϊ�˲��ܸ��� ֱ�ӷ��͵���Ϊ0	
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
//��̨��ʼ��
static void Gimbal_Init()
{
	const static fp32 gimbal_pitch_order_filter[1] = {GIMBAL_PITCH_ACCEL_NUM};
	const static fp32 gimbal_yaw_order_filter[1] = {GIMBAL_YAW_ACCEL_NUM};
	//PID��ʼ��
	Gimbal_PID_Init();
	//�˲���ʼ��
	first_order_filter_init(&gimbal_info.pitch_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_pitch_order_filter);
	first_order_filter_init(&gimbal_info.yaw_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_yaw_order_filter);
	//��ȡң��������
	gimbal_info.gimbal_RC = get_remote_control_point();
	//��ȡ�������
	gimbal_info.yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_info.pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	//���³�ʼ�Ƕ�
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.gimbal_motor_measure->last_ecd;
	gimbal_info.pitch_motor.ecd_now = gimbal_info.pitch_motor.gimbal_motor_measure->ecd;
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.gimbal_motor_measure->last_ecd;
	gimbal_info.yaw_motor.ecd_now = gimbal_info.yaw_motor.gimbal_motor_measure->ecd;
	
	gimbal_info.pitch_motor.angle = gimbal_info.pitch_motor.angle_set = relative_angle_change( gimbal_info.pitch_motor.gimbal_motor_measure->last_ecd, 
							gimbal_info.pitch_motor.gimbal_motor_measure->ecd, PITCH_ECD_DEL, &gimbal_info.pitch_motor.turn_table_flag, PITCH_ECD_TURN);
	gimbal_info.yaw_motor.angle = gimbal_info.yaw_motor.angle_set = relative_angle_change(gimbal_info.yaw_motor.gimbal_motor_measure->last_ecd, 
							gimbal_info.yaw_motor.gimbal_motor_measure->ecd, YAW_ECD_DEL, &gimbal_info.yaw_motor.turn_table_flag, YAW_ECD_TURN);
	//��ʼ����̨Ȧ��
	gimbal_info.turn_circle_num = 0;
	//��ȡ��������
	gimbal_info.gimbal_AUTO_ctrl = get_AUTO_control_point();
	//���Ƕ�����
	gimbal_info.yaw_motor.angle_max = (YAW_ECD_MAX - YAW_ECD_DEL) * Ecd_to_Rad;
	gimbal_info.yaw_motor.angle_min = -1.6400f;//( YAW_ECD_MIN - YAW_ECD_DEL) * Ecd_to_Rad;//
	gimbal_info.pitch_motor.angle_max = (PITCH_ECD_MAX - PITCH_ECD_DEL) * Ecd_to_Rad;
	gimbal_info.pitch_motor.angle_min = (PITCH_ECD_MIN - PITCH_ECD_DEL) * Ecd_to_Rad;
}
//��̨PID��ʼ��
static void Gimbal_PID_Init(void)
{
	//�ٶȻ� �ڻ�
	const fp32 yaw_motor_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
	const fp32 pitch_motor_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
	//�ǶȻ� �⻷
	const fp32 yaw_motor_angle_pid[3] = {YAW_ANGLE_PID_KP, YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD};
	const fp32 pitch_motor_angle_pid[3] = {PITCH_ANGLE_PID_KP, PITCH_ANGLE_PID_KI, PITCH_ANGLE_PID_KD};
	//����ǶȻ� �⻷
	const fp32 yaw_motor_auto_pid[3] = {YAW_AUTO_PID_KP, YAW_AUTO_PID_KI, YAW_AUTO_PID_KD};
	const fp32 pitch_motor_auto_pid[3] = {PITCH_AUTO_PID_KP, PITCH_AUTO_PID_KI, PITCH_AUTO_PID_KD};
	
	PID_Init(&gimbal_info.yaw_motor.motor_speed_pid, PID_POSITION, yaw_motor_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT );
	PID_Init(&gimbal_info.yaw_motor.motor_angle_pid, PID_POSITION, yaw_motor_angle_pid, YAW_ANGLE_PID_MAX_OUT, YAW_ANGLE_PID_MAX_IOUT );
	PID_Init(&gimbal_info.yaw_motor.motor_auto_pid, PID_POSITION, yaw_motor_auto_pid, YAW_AUTO_PID_MAX_OUT, YAW_AUTO_PID_MAX_IOUT );
	
	PID_Init(&gimbal_info.pitch_motor.motor_speed_pid, PID_POSITION, pitch_motor_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT );
	PID_Init(&gimbal_info.pitch_motor.motor_angle_pid, PID_POSITION, pitch_motor_angle_pid, PITCH_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT );
	PID_Init(&gimbal_info.pitch_motor.motor_auto_pid, PID_POSITION, pitch_motor_auto_pid, PITCH_AUTO_PID_MAX_OUT, PITCH_AUTO_PID_MAX_IOUT );
}
//��̨ģʽѡ��
static void Gimbal_mode_set(void)
{
	//��ң�������ߵ�ʱ��ΪAUTOģʽ
	if (toe_is_error(DBUSTOE))
	{
		gimbal_info.gimbal_mode = GIMBAL_AUTO;//�Զ�
	}
    else if (switch_is_mid(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))		//�м�Ϊң��
    {
        gimbal_info.gimbal_mode = GIMBAL_RC;//ң��
    }
    else if (switch_is_up(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))		//�ϲ�Ϊ�Զ�
    {
        gimbal_info.gimbal_mode = GIMBAL_AUTO;//�Զ�
    }
    else if (switch_is_down(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))	//�²�Ϊ����
    {
        gimbal_info.gimbal_mode = GIMBAL_RELAX;//��̨����
    }
}
//��ԽǶ�
static fp32 relative_angle_change(uint16_t last_ecd, uint16_t ecd, uint16_t offset_ecd, uint8_t *turn_table_flag, uint16_t ecd_turn)
{
	int32_t relative_ecd = 0;
	int16_t err_ecd;
	//������Ի�е�Ƕ�
	ECD_format(ecd, offset_ecd);
	ECD_format(last_ecd, offset_ecd);
	//��е�ǶȲ�
	err_ecd = ecd - last_ecd;
	//����Ƕ�����
	if (err_ecd > 6000 || (!gimbal_info.turn_mid_flag && ecd < 8192 && ecd > 6000))		//�������ת����ʼ�� 0(8191) ���λ��Ӧ��Ϊ����
	{
		*turn_table_flag = 0;
	}
	else if (err_ecd < -6000 || (!gimbal_info.turn_mid_flag && ecd <= 6000))//&& ecd >= 0 �����ת������ʼ�� 8191(0) ���λ��Ӧ��Ϊ����
	{
		*turn_table_flag = 1;
	}
	if (*turn_table_flag)		//���Ƕ����� (0,2PI)
	{
		relative_ecd = ecd;
	}
	else
	{
		relative_ecd = ecd - 8192;
	}
	//�������ֵת��Ϊrad
	return relative_ecd * Ecd_to_Rad;
}
//������̨Ȧ����ʵ�ʽǶ�
static void calc_turn_angle(fp32 *angle_last, fp32 *angle_now)
{
	//����Ȧ��Ϊ�� ��ʱ��
	if(*angle_now - *angle_last < -4.0f)
	{
		gimbal_info.turn_circle_num++;
		*angle_now += 2 * PI;
	}
	//����Ȧ��Ϊ�� ˳ʱ��
	else if(*angle_now - *angle_last > 4.0f)
	{
		gimbal_info.turn_circle_num--;
		*angle_now -= 2 * PI;
	}
}
//��̨���ݸ���
static void Gimbal_Updata(void)
{
	//fp32 angle_temp = gimbal_info.yaw_motor.angle;
	const fp32 *gimbal_INT_angle_point = get_INS_angle_point();//��ȡ��������̬���������� �Ƕ�
	const fp32 *gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();//��ȡ������ԭʼ���� ���ٶ�
	//�Ƕȸ���  �⻷
	gimbal_info.pitch_motor.angle_last = gimbal_info.pitch_motor.angle;//angle_pitch_temp;
	gimbal_info.yaw_motor.angle_last = gimbal_info.yaw_motor.angle;
	//���»�е�Ƕ�(ecd)����ԽǶ�(angle)
	gimbal_info.pitch_motor.ecd_now = gimbal_info.pitch_motor.gimbal_motor_measure->ecd;
	gimbal_info.yaw_motor.ecd_now = gimbal_info.yaw_motor.gimbal_motor_measure->ecd;
	gimbal_info.pitch_motor.angle = relative_angle_change(gimbal_info.pitch_motor.ecd_last, gimbal_info.pitch_motor.ecd_now, 
											PITCH_ECD_DEL, &gimbal_info.pitch_motor.turn_table_flag, PITCH_ECD_TURN);
	gimbal_info.yaw_motor.angle = relative_angle_change( gimbal_info.yaw_motor.ecd_last, gimbal_info.yaw_motor.ecd_now, 
											YAW_ECD_DEL, &gimbal_info.yaw_motor.turn_table_flag, YAW_ECD_TURN) + gimbal_info.turn_circle_num * 2 * PI;
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.ecd_now;
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.ecd_now;
	//����yaw����Ȧ��
	calc_turn_angle(&gimbal_info.yaw_motor.angle_last, &gimbal_info.yaw_motor.angle);
	//���ٶȸ���  �ڻ�
	//gimbal_info.pitch_motor.speed = *(gimbal_INT_gyro_point + INS_PITCH_ADDRESS_OFFSET);		//�������ܹ�������̨pitch��
	gimbal_info.pitch_motor.speed = gimbal_info.pitch_motor.gimbal_motor_measure->speed_rpm * Ecd_to_Rad;	//�������õ��������ת��ֵrmp
	//gimbal_info.yaw_motor.speed = *(gimbal_INT_gyro_point + INS_YAW_ADDRESS_OFFSET);
	gimbal_info.yaw_motor.speed = gimbal_info.yaw_motor.gimbal_motor_measure->speed_rpm * Ecd_to_Rad;
	//��ѹ����
	gimbal_info.yaw_motor.last_give_voltage = gimbal_info.yaw_motor.give_voltage;
	gimbal_info.pitch_motor.last_give_voltage = gimbal_info.pitch_motor.give_voltage;
}
//����������
static void Gimbal_Control(void)
{
	fp32 add_yaw_angle = 0.0f;
	fp32 add_pitch_angle = 0.0f;
	//ѡ�����������
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
	//���д���
	if( !gimbal_info.turn_mid_flag )
	{
		//��������
		if(gimbal_info.pitch_motor.angle > 0.1f || gimbal_info.pitch_motor.angle < -0.1f)
		{
			add_pitch_angle = gimbal_info.pitch_motor.angle < 0.0f ? GIMBAL_RETURN_PITCH : -GIMBAL_RETURN_PITCH;		//��̧��pitch��
			add_yaw_angle = 0.0f;
		}
		else if(gimbal_info.yaw_motor.angle > 0.1f || gimbal_info.yaw_motor.angle < -0.1f)
		{
			add_pitch_angle = 0.0f;
			add_yaw_angle = gimbal_info.yaw_motor.angle < 0.0f ? GIMBAL_RETURN_YAW : -GIMBAL_RETURN_YAW;
		}
		else	//�ѻ���
		{
			gimbal_info.turn_mid_flag = 1;
		}
		//�Ƕȵ���
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
		//�Ƕȵ���
		gimbal_info.pitch_motor.angle_set += add_pitch_angle;
		gimbal_info.yaw_motor.angle_set += add_yaw_angle;
		//�Ƕ��޷�
		Angle_Limit();
	}
	//PID����
	gimbal_pid_calculation(gimbal_info.gimbal_mode);
	//��ѹ
	gimbal_info.yaw_motor.give_voltage = (int16_t)gimbal_info.yaw_motor.voltage_set;
	gimbal_info.pitch_motor.give_voltage = (int16_t)gimbal_info.pitch_motor.voltage_set;
}
static void Gimbal_Calibration_mode(void)
{
	return;
}
//��̨����ģʽ
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
	//���б�־��0
	gimbal_info.turn_mid_flag = 0;
	gimbal_info.turn_circle_num = 0;
}
//��̨�Զ�ģʽ
static void Gimbal_Auto_mode(fp32 *pitch_add, fp32 *yaw_add)
{
	//��ʼ����ر���
	static uint8_t run_flag_pitch;	//0������ 1������
	static uint8_t run_flag_yaw;
	uint8_t vision_flag = 0;//Vision_update_flag();			//��ȡ�Ӿ���־λ
	//gimbal_info.turn_mid_flag = 0;
	//��Ħ����
	
	//�Ӿ��Ƿ�ʶ��
	if( vision_flag )	//ʶ��Ŀ�� ��̨���Ӿ�����
	{
		*yaw_add = gimbal_info.gimbal_AUTO_ctrl->yaw_angle * YAW_AUTO_SEN;
		*pitch_add = gimbal_info.gimbal_AUTO_ctrl->pitch_angle * PITCH_AUTO_SEN;
		Vision_clean_flag();	
		//Ŀ��ֵ�뵱ǰֵ���Ϊ0.5fʱ���� 
		if(0)
		{
			
		}
	}
	else				//��̨����Ѳ�� 
	{
		//����Ѳ��ͣ��
		
		//���ﵽ���Ƕ�
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
//��̨ң�ؿ���
static void Gimbal_Rc_mode(fp32 *pitch_add, fp32 *yaw_add)
{
	//��ʼ����ر���
	int16_t yaw_channel = 0;
	int16_t pitch_channel = 0;
	//ң������������
	if( gimbal_info.gimbal_RC->rc.ch[YAW_CHANNEL] > GIMBAL_RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[YAW_CHANNEL] < -GIMBAL_RC_DEADLINE )
		yaw_channel = gimbal_info.gimbal_RC->rc.ch[YAW_CHANNEL];
	if( gimbal_info.gimbal_RC->rc.ch[PITCH_CHANNEL] > GIMBAL_RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[PITCH_CHANNEL] < -GIMBAL_RC_DEADLINE )
		pitch_channel = gimbal_info.gimbal_RC->rc.ch[PITCH_CHANNEL];
	//ң�����ݱ���ת��
	*yaw_add = yaw_channel * YAW_RC_SEN;
	*pitch_add = pitch_channel * PITCH_RC_SEN;
}
//��̨PID����
static void gimbal_pid_calculation(Gimbal_Mode_e gimbal_mode)
{
	if( gimbal_mode == GIMBAL_AUTO)
	{
		//PID���� �ǶȻ�Ϊ�⻷ �ٶȻ�Ϊ�ڻ�
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
		//PID���� �ǶȻ�Ϊ�⻷ ���ٶȻ�Ϊ�ڻ�
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
//�Ƕ�����
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
