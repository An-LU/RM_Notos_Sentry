#ifndef __TEST_H__
#define __TEST_H__

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"
#include "miniPC.h"
#include "user_lib.h"

//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201
//��̨����������
#define GIMBAL_CONTROL_TIME_MS 1
//����ģʽѡ�� ��߿���
#define GIMBAL_MODE_SW		1
//���ҡ�˿���
#define YAW_CHANNEL			2
#define PITCH_CHANNEL		3
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define GIMBAL_RC_DEADLINE	10
//yaw��pitch�Ƕ���ң�����������
#define YAW_RC_SEN 0.000001f
#define PITCH_RC_SEN -0.000002f //0.005
//��̨��е�Ƕ���λ
#define YAW_ECD_MAX		8191
#define YAW_ECD_MIN		0
#define YAW_ECD_DEL		0
#define PITCH_ECD_MAX	8191
#define PITCH_ECD_MIN	0
#define PITCH_ECD_DEL	0
#define Half_Ecd_range	4096
//�˲�ϵ��
#define GIMBAL_YAW_ACCEL_NUM 0.1666666667f
#define GIMBAL_PITCH_ACCEL_NUM 0.3333333333f
//�������ֵת���ɽǶ�ֵ    2��/8191
#define Ecd_to_Rad		0.000798488132787405781442588526584f
//GM6020���CAN���͵�ѹ
#define MAX_GM6020_CAN_VOLTAGE	30000.0f
//PID
//pitch �ٶȻ� �ڻ�
#define PITCH_SPEED_PID_KP			1000.0f   //2500.0f //��ʼ��2000.0f
#define PITCH_SPEED_PID_KI			0.0f  //1.0f
#define PITCH_SPEED_PID_KD			0.0f  //30.0f
#define PITCH_SPEED_PID_MAX_OUT		MAX_GM6020_CAN_VOLTAGE
#define PITCH_SPEED_PID_MAX_IOUT	5000.0f
//yaw �ٶȻ�
#define YAW_SPEED_PID_KP			1500.0f  //8000.0f//��ʼ��2200.0f
#define YAW_SPEED_PID_KI			0.0f  //20.0f //��ʼ��20.0f
#define YAW_SPEED_PID_KD			0.5f  //0.0f
#define YAW_SPEED_PID_MAX_OUT		MAX_GM6020_CAN_VOLTAGE
#define YAW_SPEED_PID_MAX_IOUT		5000.0f
//pitch �ǶȻ� �����Ƕ��ɱ������ṩ �⻷
#define PITCH_ANGLE_PID_KP 150.0f//65
#define PITCH_ANGLE_PID_KI 0.0f
#define PITCH_ANGLE_PID_KD 0.0f//2.0
#define PITCH_ANGLE_PID_MAX_OUT 10.0f
#define PITCH_ANGLE_PID_MAX_IOUT 0.0f
//yaw �ǶȻ� �����Ƕ����������ṩ
#define YAW_ANGLE_PID_KP 178.0f
#define YAW_ANGLE_PID_KI 0.0f
#define YAW_ANGLE_PID_KD 0.0f
#define YAW_ANGLE_PID_MAX_OUT 10.0f
#define YAW_ANGLE_PID_MAX_IOUT 0.0f
//pitch ����ǶȻ�
#define PITCH_AUTO_PID_KP 100.0f
#define PITCH_AUTO_PID_KI 0.0f
#define PITCH_AUTO_PID_KD 0.0f
#define PITCH_AUTO_PID_MAX_OUT 10.0f
#define PITCH_AUTO_PID_MAX_IOUT 0.0f
//yaw ����ǶȻ�
#define YAW_AUTO_PID_KP       100.0f
#define YAW_AUTO_PID_KI        0.0f
#define YAW_AUTO_PID_KD        0.0f
#define YAW_AUTO_PID_MAX_OUT   10.0f
#define YAW_AUTO_PID_MAX_IOUT  0.0f

typedef enum
{
	GIMBAL_RELAX,				//��̨����
	GIMBAL_AUTO,				//��̨�Զ�Ѳ��ģʽ
	GIMBAL_RC,					//��̨ң�ؿ���
	GIMBAL_CALI					//��̨У׼
}Gimbal_Mode_e;
//typedef struct
//{
//	fp32 angle_max;
//	fp32 angle_min;
//	
//}Gimbal_Limit_s;
typedef struct
{
	const motor_measure_t *gimbal_motor_measure;	//���������Ϣ
	first_order_filter_type_t gimbal_motor_first_OF;	//���һ�׵�ͨ�˲�
	PidTypeDef motor_speed_pid;	//���ٶȻ�pid
	PidTypeDef motor_angle_pid;	//�ǶȻ�pid
	PidTypeDef motor_auto_pid;	//����ǶȻ�pid
	fp32 angle;				//��ǰ�Ƕ� rad
	fp32 angle_set;			//�趨�Ƕ�
	uint16_t angle_max;		//��е�Ƕ���λrad
	uint16_t angle_min;
	fp32 speed;				//��ǰ���ٶ�rad/s
	fp32 speed_set;			//�趨���ٶ�
	fp32 speed_max;			//���ٶ���λ
	fp32 speed_min;
	fp32 voltage_set;
	int16_t last_give_voltage;
	int16_t give_voltage;	//���͵ĵ�ѹֵ
} Gimbal_Motor_s;
typedef struct
{
	const RC_ctrl_t *gimbal_RC;				//��̨ʹ�õ�ң����ָ��
	Gimbal_Mode_e gimbal_mode;				//��̨ģʽ
	Gimbal_Mode_e last_gimbal_mode;			//��̨��һ��ģʽ
	
	Gimbal_Motor_s pitch_motor;				//��̨pitch���
	Gimbal_Motor_s yaw_motor;				//��̨yaw���
	
}Gimbal_Control_s;


#endif

