#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"
#include "sensor_sw.h"

//����ģʽѡ�� �ұ߿���
#define CHASSIS_MODE_SW			0
//���̿���ͨ�� �ұ�ҡ��
#define CHASSIS_MOVE_CHANNEL	0
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define CHASSIS_RC_DEADLINE 10
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//���̷���ʱ���� 2ms
#define CHASSIS_RETURN_TIME_MS 2
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//���ٶȸ���ʹ��		�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_M3508_CAN_CURRENT 16000.0f
//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������    2��/60 * r 
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f//0.000798488132787405781442588526584f//0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//���̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_M3508_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//���������ٶ�
#define CHASSIS_SPEED_1		-1.5f
#define CHASSIS_SPEED_2		-1.5f

//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_CHANNEL_RC_SEN			0.005f
//�˲�ϵ��
#define CHASSIS_ACCEL_NUM				0.1666666667f


////����ǰ�����ҿ��ư���
//#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
//#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
//#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
//#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D


typedef struct
{
	const motor_measure_t *chassis_motor_measure;	//���������Ϣ
	fp32 accel;				//���ٶ�
	fp32 speed;				//��ǰ�ٶ�
	fp32 speed_set;			//�趨�ٶ�
	int16_t last_give_current;
	int16_t give_current;	//���͵ĵ���ֵ
} Chassis_Motor_t;
/******************************************************/
//typedef enum
//{
//	CHASSIS_PAUSE,				//����ͣ�� ʶ��Ŀ��
//	CHASSIS_PATROL,				//��������Ѳ��
//}chassis_auto_mode;
//typedef enum
//{
//	L,		//�����˶�
//	S,		//���˶�
//	R,		//�����˶�
//}move_dir;
typedef enum
{
	CHASSIS_RELAX,				//��������
	CHASSIS_AUTO,				//�Զ�ģʽ
	CHASSIS_RC					//����ң�ؿ���
}chassis_mode_e;
typedef struct
{
	//gimbal_vision_info;//��̨��Ϣ
	const RC_ctrl_t *chassis_RC;				//ң����ָ��
	chassis_mode_e chassis_mode;				//����ģʽ
	chassis_mode_e last_chassis_mode;			//������һ��ģʽ
	Chassis_Motor_t chassis_motor;				//���̵������
	PidTypeDef chassis_speed_pid;				//�����ٶȻ�pid
	first_order_filter_type_t chassis_first_OF;	//һ�׵�ͨ�˲�����
	fp32 chassis_speed;							//�����ٶ�
	fp32 chassis_speed_set;						//�����ٶ��趨
	fp32 chassis_speed_max;						//��������ٶ�
	fp32 chassis_speed_min;						//������С�ٶ�
	//move_dir last_dir_flag;						//������һ���˶������־
	move_dir dir_flag;							//�����˶������־
}chassis_control;
/******************************************************/
extern void chassis_task(void *pvParameters);
//extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_control *chassis_move_rc_to_vector);
extern const chassis_control *get_motor_point(void);
#endif
