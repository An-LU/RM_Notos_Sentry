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

//������Ϣ
static chassis_control chassis_info;

//��������ʼ��
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
/***����***/
void test_dir(void);
fp32 test_speed(void);
/**********/

//������
void chassis_task(void *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	//��ʼ������
	Chassis_Init();
	//�жϵ����ң���Ƿ�����
	while (toe_is_error(ChassisMotor1TOE) || toe_is_error(DBUSTOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
	while(1)
	{
		//ģʽ  ң��ģʽ  �Զ�ģʽ
		Chassis_mode_set();
		//�������ݸ���  
		Chassis_Updata();
		//���̿���������
		Chassis_Control();
		//���Ϳ��Ƶ���
		Chassis_Send_Current();
		//ϵͳ��ʱ
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
}

void Chassis_Send_Current(void)
{
	//������ģʽΪ����״̬ Ϊ�˲��ܸ��� ֱ�ӷ��͵���Ϊ0
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
	Sensor_Init();					//��������ʼ��
	Chassis_PID_Init();				//����pid��ʼ��
	
	//���̵�ͨ�˲���ʼ��
	first_order_filter_init(&chassis_info.chassis_first_OF, CHASSIS_CONTROL_TIME, chassis_order_filter);
	//��ȡң��������
	chassis_info.chassis_RC = get_remote_control_point();	
	//�����ٶ�����
	chassis_info.chassis_speed_max = MAX_WHEEL_SPEED;
	chassis_info.chassis_speed_min = -MAX_WHEEL_SPEED;
	//��ʼ������״̬
	chassis_info.chassis_mode = CHASSIS_RELAX;
	chassis_info.dir_flag = R;
}

void Chassis_PID_Init(void)
{
	const fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	chassis_info.chassis_motor.chassis_motor_measure = get_Chassis_Motor_Measure_Point(0);//��Ҫ�޸�
	PID_Init(&chassis_info.chassis_speed_pid, PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
}

static void Chassis_mode_set(void)
{
	//ң�������õ���ģʽ
	//��ң�������ߵ�ʱ��ΪAUTOģʽ
	if (toe_is_error(DBUSTOE))
	{
		chassis_info.last_chassis_mode = chassis_info.chassis_mode;
		chassis_info.chassis_mode = CHASSIS_AUTO;//�Զ�
	}
    else if (switch_is_mid(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//�м�Ϊң��
    {
		chassis_info.last_chassis_mode = chassis_info.chassis_mode;
        chassis_info.chassis_mode = CHASSIS_RC;//ң��
    }
    else if (switch_is_up(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))		//�ϲ�Ϊ�Զ�
    {
		chassis_info.last_chassis_mode = chassis_info.chassis_mode;
        chassis_info.chassis_mode = CHASSIS_AUTO;//�Զ�
    }
    else if (switch_is_down(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//�²�Ϊ����
    {
		chassis_info.last_chassis_mode = chassis_info.chassis_mode;
        chassis_info.chassis_mode = CHASSIS_RELAX;//��������
    }
}
void Chassis_Updata(void)
{
	//���µ���ٶȺͼ��ٶ�
	chassis_info.chassis_motor.speed = chassis_info.chassis_motor.chassis_motor_measure->speed_rpm * M3508_MOTOR_RPM_TO_VECTOR;
	chassis_info.chassis_motor.accel = chassis_info.chassis_speed_pid.Dout * CHASSIS_CONTROL_FREQUENCE;
	//���µ����ٶ�
	chassis_info.chassis_speed = chassis_info.chassis_motor.chassis_motor_measure->speed_rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
}
static void Chassis_Control(void)
{
	//ѡ�����������
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
	//pid����
	PID_Calc(&chassis_info.chassis_speed_pid, chassis_info.chassis_speed, chassis_info.chassis_speed_set);
	//���µ������
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
	//��ʼ����ر���
	uint8_t vision_flag = 0;//Vision_update_flag();			//��ȡ�Ӿ���־λ
	//�Ӿ��Ƿ�ʶ��
	if( vision_flag )	//ʶ��Ŀ�� ����ֹͣ CHASSIS_PAUSE
	{
		chassis_info.chassis_speed_set = 0.0f;
	}
	else				//����Ѳ�� CHASSIS_PATROL
	{
		//test_dir();
		chassis_auto_patrol();
	}
}
void Chassis_Rc_mode(void)
{
	int16_t chassis_move_channel;
	fp32 chassis_move_set;
	//ң����������
	if(chassis_info.chassis_RC->rc.ch[CHASSIS_MOVE_CHANNEL] > CHASSIS_RC_DEADLINE || chassis_info.chassis_RC->rc.ch[CHASSIS_MOVE_CHANNEL] < -CHASSIS_RC_DEADLINE)
		chassis_move_channel = chassis_info.chassis_RC->rc.ch[CHASSIS_MOVE_CHANNEL];
	else
		chassis_move_channel = 0;
	//ҡ��ת�����ٶ�
	chassis_move_set = chassis_move_channel * CHASSIS_CHANNEL_RC_SEN;
	//һ�׵�ͨ�˲�����б�º�������
	first_order_filter_cali(&chassis_info.chassis_first_OF, chassis_move_set);
	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if (chassis_move_set < CHASSIS_RC_DEADLINE * CHASSIS_CHANNEL_RC_SEN && chassis_move_set > -CHASSIS_RC_DEADLINE * CHASSIS_CHANNEL_RC_SEN)
		chassis_info.chassis_first_OF.out = 0.0f;
	//�ٶ��޷���ֵ������
	chassis_info.chassis_speed_set = fp32_constrain(chassis_info.chassis_first_OF.out, chassis_info.chassis_speed_min, chassis_info.chassis_speed_max);
}
//����Ѳ��
void chassis_auto_patrol(void)
{
	uint8_t sw_status = get_Switch_Status(chassis_info.dir_flag);
	//����δ���� ����Ѳ��
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
	//���̻ص�
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

//ģ�⿪��
void test_dir(void)
{
    if (switch_is_mid(chassis_info.chassis_RC->rc.s[1]))	//�м�Ϊң��
    {
		chassis_info.dir_flag = L;
    }
    else if (switch_is_up(chassis_info.chassis_RC->rc.s[1]))	//�ϲ�Ϊ�Զ�
    {
		chassis_info.dir_flag = R;
    }
    else if (switch_is_down(chassis_info.chassis_RC->rc.s[1]))	//�²�Ϊ����
    {
		chassis_info.dir_flag = S;
    }
}
//�����ٶ�
fp32 test_speed(void)
{
    if (switch_is_mid(chassis_info.chassis_RC->rc.s[1]))	//�м�
    {
		return -2.0f;
    }
    else if (switch_is_up(chassis_info.chassis_RC->rc.s[1]))	//�ϲ�
    {
		return -2.5f;
    }
    else if (switch_is_down(chassis_info.chassis_RC->rc.s[1]))	//�²�
    {
		return -1.5f;
    }
	else
	{
		return 0.0f;
	}
}




