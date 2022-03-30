#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

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

/******�����ػ�е�Ƕ� �������ʱ��Ҫdebug�޸ģ�����******************/
#define YAW_ECD_MAX		2709		//��̨��е�Ƕ���λ
#define YAW_ECD_MIN		6793
#define YAW_ECD_DEL		655			//��̨����ʱ����Ļ�е�Ƕ�
#define YAW_ECD_TURN	4000		//��̨����ʱ�ж��������̻�е�Ƕ� С�ڴ�ֵΪ������ ���ڴ�ֵΪ������ ���ڻ���ʱ ����״̬����
#define PITCH_ECD_MAX	3811
#define PITCH_ECD_MIN	2036
#define PITCH_ECD_DEL	2696
#define PITCH_ECD_TURN	6000
/*******************��ػ�е�Ƕ�_END**********************************/

//��̨ģʽѡ�� ��߿���
#define GIMBAL_MODE_SW		1
//���ҡ�˿���
#define YAW_CHANNEL			2
#define PITCH_CHANNEL		3
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define GIMBAL_RC_DEADLINE	10
//yaw��pitch�Ƕ���ң�����������
#define YAW_RC_SEN		0.000003f
#define PITCH_RC_SEN	-0.000003f
//��̨Ѳ���ٶ�
#define AUTO_PATROL_SPEED_YAW	0.0008f
#define AUTO_PATROL_SPEED_PITCH	0.001f
//��̨�����ٶ�
#define GIMBAL_RETURN_PITCH	0.0008f
#define GIMBAL_RETURN_YAW	0.0008f
//�˲�ϵ��
#define GIMBAL_YAW_ACCEL_NUM	0.3333333333f//0.1666666667f
#define GIMBAL_PITCH_ACCEL_NUM	0.3333333333f
//�������ֵת���ɽǶ�ֵ    2��/8191
#define Ecd_to_Rad		0.000798488132787405781442588526584f
//GM6020���CAN���͵�ѹ
#define MAX_GM6020_CAN_VOLTAGE	30000.0f

/***********************PID***************************/
//pitch �ٶȻ� �ڻ�
#define PITCH_SPEED_PID_KP			1000.0f   //2500.0f //2000.0f
#define PITCH_SPEED_PID_KI			0.0f  //1.0f
#define PITCH_SPEED_PID_KD			2.0f  //30.0f
#define PITCH_SPEED_PID_MAX_OUT		MAX_GM6020_CAN_VOLTAGE
#define PITCH_SPEED_PID_MAX_IOUT	5000.0f
//yaw �ٶȻ�
#define YAW_SPEED_PID_KP			1500.0f  //8000.0f//2200.0f
#define YAW_SPEED_PID_KI			0.0f  //20.0f //20.0f
#define YAW_SPEED_PID_KD			0.5f  //0.0f
#define YAW_SPEED_PID_MAX_OUT		MAX_GM6020_CAN_VOLTAGE
#define YAW_SPEED_PID_MAX_IOUT		5000.0f
//pitch �ǶȻ� �����Ƕ��ɱ������ṩ �⻷
#define PITCH_ANGLE_PID_KP 150.0f//65
#define PITCH_ANGLE_PID_KI 0.0f
#define PITCH_ANGLE_PID_KD 0.0f//2.0f
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
/**********************PID_END************************/
typedef enum
{
	GIMBAL_RELAX,				//��̨����
	GIMBAL_AUTO,				//��̨�Զ�Ѳ��ģʽ
	GIMBAL_RC,					//��̨ң�ؿ���
	GIMBAL_CALI					//��̨У׼
}Gimbal_Mode_e;
typedef struct
{
	const motor_measure_t *gimbal_motor_measure;	//���������Ϣ
	first_order_filter_type_t gimbal_motor_first_OF;	//���һ�׵�ͨ�˲�
	PidTypeDef motor_speed_pid;	//���ٶȻ�pid
	PidTypeDef motor_angle_pid;	//�ǶȻ�pid
	PidTypeDef motor_auto_pid;	//����ǶȻ�pid
	fp32 angle_last;
	fp32 angle;				//��ǰ�Ƕ� rad
	fp32 angle_set;			//�趨�Ƕ�
	fp32 angle_max;		//��е�Ƕ���λrad
	fp32 angle_min;
	fp32 speed;				//��ǰ���ٶ�rad/s
	fp32 speed_set;			//�趨���ٶ�
//	fp32 speed_max;			//���ٶ���λ
//	fp32 speed_min;
	fp32 voltage_set;
	int16_t ecd_last;
	int16_t ecd_now;
	int16_t last_give_voltage;
	int16_t give_voltage;	//���͵ĵ�ѹֵ
	uint8_t turn_table_flag;//�Ƕ������� 1Ϊ�� 0Ϊ��
} Gimbal_Motor_s;
typedef struct
{
	const RC_ctrl_t *gimbal_RC;				//��̨ʹ�õ�ң����ָ��
	Auto_Gimbal_Ctrl_t *gimbal_AUTO_ctrl;	//����Ƕ�����
	Gimbal_Mode_e gimbal_mode;				//��̨ģʽ
	Gimbal_Mode_e last_gimbal_mode;			//��̨��һ��ģʽ
	
	Gimbal_Motor_s pitch_motor;				//��̨pitch���
	Gimbal_Motor_s yaw_motor;				//��̨yaw���
	
	int16_t turn_circle_num;
	uint8_t turn_mid_flag;
}Gimbal_Control_s;

extern const Gimbal_Motor_s *get_yaw_motor_point(void);
extern const Gimbal_Motor_s *get_pitch_motor_point(void);
extern void GIMBAL_task(void *pvParameters);

#endif

