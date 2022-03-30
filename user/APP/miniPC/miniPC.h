#ifndef __MINIPC_H__
#define __MINIPC_H__

#include "main.h"
#include "judge_system.h"

#define VISION_CTRL_LENGTH	23 //����Ϊ20�ֽڣ�ͷ7�ֽڣ�����14�ֽ�
#define VISION_HEAD_SOF		0xA5
#define VISION_UP_SOF		0xA0
#define	VISION_LENGTH_SOF	5	//֡ͷ����
#define	VISION_LENGTH_TAIL	2	//֡β����
#define	VISION_LENGTH_CMD	2
#define VISION_CTRL_LEN		14	//�������ݴ�С
#define VISION_DATA_LEN		16	//�ϴ����ݴ�С

//�Ƿ����Ӳ���ϵͳ
#define JUDGE_SYSTEM_OFFLINE	0
#define JUDGE_SYSTEM_ONLINE		1
#define ROBOT_MODE	JUDGE_SYSTEM_OFFLINE
#define RED_ROBOT	1
#define BLUE_ROBOT	2
//���������� ����ģʽ Red:1 Blue:2
#define ROBOT_TYPE	RED_ROBOT

#define YAW_AUTO_SEN    -0.0055f
#define PITCH_AUTO_SEN  -0.07f //0.005

/* ֡ͷ����ƫ���� */
typedef enum
{
	SOF_offset_v = 0,
	DataLength_offset_v = 1,
	Seq_offset_v = 3,
	CRC8_offset_v = 4,
	Cmd_ID_v = 5,
	AutoCtrl_Data_v = 7
}Vision_Farme_Header_Offset_t;
/* ֡ͷ byte: 5 */
typedef __packed struct
{
	uint8_t  SOF;			
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
}Vision_Frame_Header_t;
/* ֡β byte: 2 */
typedef __packed struct
{
	uint16_t CRC16;
}Vision_Frame_Tail_t;
/* �������� byte: 14 */
////�Ӿ��Ƕ�
//typedef __packed struct
//{
//	float pitch_angle;
//	float yaw_angle;
//} CTRL_t;
typedef __packed struct
{
    uint32_t	time;
    fp32		pitch_angle;	//��̨����pitch�Ƕ�
    fp32		yaw_angle;		//��̨����yaw�Ƕ�
	//CTRL_t ctrl_angle;
    uint8_t		visual_valid;	//������Чλ
    uint8_t		buff_shoot;		//�Զ�����
}Auto_Gimbal_Ctrl_t;
/* miniPC-->stm32 byte: 23 */
typedef __packed struct
{
	/* ͷ */
	Vision_Frame_Header_t head_data;
	/* ������ID */
	uint16_t cmd_ID;
	/* ��̨���� */
	Auto_Gimbal_Ctrl_t auto_gimbal_ctrl;
	/* ֡β */
	Vision_Frame_Tail_t tail_data;
}Vision_Recv_Data_t;
/* stm32�������� byte: 16 */
typedef __packed struct
{
	uint8_t		mode;			//ģʽ��0װ�װ� 1��� 2��
    uint8_t		enemy_color;	//�з���ɫ��1 red 2 blue
    uint8_t		is_left;		//
    uint8_t		run_left;		//
    fp32		bullet_spd;		//�����ٶ�
    fp32		pitch;			//��ǰpitch�Ƕ�
    fp32		yaw;			//��ǰyaw�Ƕ�
    //fp32		pitch_offset;	//pit�����Ƕ�
    //fp32		yaw_offset;		//yaw�����Ƕ�
}Stm32_Info_t;
/* stm32-->miniPC  byte: 25 */
typedef __packed struct
{
	/* ͷ */
	Vision_Frame_Header_t head_data;
	/* ������ID */
	uint16_t cmd_ID;
	/* stm32���� */
	Stm32_Info_t stm32_info;
	/* ֡β */
	Vision_Frame_Tail_t tail_data;
}Vision_Send_Data_t;

//typedef struct
//{
//	union 
//	{
//		uint8_t databuf[16];
//		Stm32_Info_t stm32_info;
//	}data;
//}Stm32_Info_t_2;
typedef enum
{
	ARMOR_PLATE,	//װ�װ�
	BUFF_ANTI,		//���
	NO_VISION
}Vision_Mode_e;
//-----------------------------------��������Ϣ�ṹ�塿--------------------------------------------
// brief��������̨����ģʽ��pitch�Ƕȣ�yaw�Ƕȵ���Ϣ���ǿ��Ʋ����˶�������

typedef enum
{
  CHASSIS_DATA_ID     = 0x0010,
  GIMBAL_DATA_ID      = 0x0011,
  SHOOT_TASK_DATA_ID  = 0x0012,
  INFANTRY_ERR_ID     = 0x0013,
  CONFIG_RESPONSE_ID  = 0x0014,
  CALI_RESPONSE_ID    = 0x0015,
  REMOTE_CTRL_INFO_ID = 0x0016,
  BOTTOM_VERSION_ID   = 0x0017,
  SENTRY_DATA_ID      = 0x0018,

  CHASSIS_CTRL_ID     = 0x00A0,
  GIMBAL_CTRL_ID      = 0x00A1,
  SHOOT_CTRL_ID       = 0x00A2,
  ERROR_LEVEL_ID      = 0x00A3,
  INFANTRY_STRUCT_ID  = 0x00A4,
  CALI_GIMBAL_ID      = 0x00A5,
} infantry_data_id_e;

typedef enum
{
  BOTTOM_DEVICE        = 0,
  GIMBAL_GYRO_OFFLINE  = 1,
  CHASSIS_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE   = 3,
  CHASSIS_M2_OFFLINE   = 4,
  CHASSIS_M3_OFFLINE   = 5,
  CHASSIS_M4_OFFLINE   = 6,
  REMOTE_CTRL_OFFLINE  = 7,
  JUDGE_SYS_OFFLINE    = 8,
  PC_SYS_OFFLINE       = 9,
  GIMBAL_YAW_OFFLINE   = 10,
  GIMBAL_PIT_OFFLINE   = 11,
  TRIGGER_MOTO_OFFLINE = 12,
  BULLET_JAM           = 13,
  CHASSIS_CONFIG_ERR   = 14,
  GIMBAL_CONFIG_ERR    = 15,
  ERROR_LIST_LENGTH    = 16,
} err_id_e;

typedef enum
{
  DEVICE_NORMAL = 0,
  ERROR_EXIST   = 1,
  UNKNOWN_STATE = 2,
} bottom_err_e;

typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  GIMBAL_ERROR         = 4,
  CHASSIS_ERROR        = 5,
  HARAWARE_ERROR       = 6,
} err_level_e;

typedef enum
{
  NO_CONFIG      = 0,//�޶���
  DEFAULT_CONFIG = 1,//Ĭ�϶���
  CUSTOM_CONFIG  = 3,//�û�����
} struct_config_e;

/********** the information send to computer ***********/


///* ͷ֡ byte: 7 */
//typedef struct
//{
//	/* ͷ */
//	uint8_t		SOF;				//֡ͷ��ʼλ,�ݶ�0xAA
//	uint8_t		SendDataLen;		//����
//	uint8_t		empty_1[3];
//	uint8_t		CMDID;				//
//	uint8_t		empty_2;
//	//����ID
//}__attribute__((packed))	extVisionSendHeader_t;

///* ������Ҫ��Ϣ byte: 14 */
//typedef struct  //__packed
//{
//    uint32_t time;
//    float    pit_ref;      /* gimbal pitch reference angle(degree) */
//    float    yaw_ref;      /* gimbal yaw reference angle(degree) */
//    uint8_t  visual_valid; /* visual information valid or not */
//    uint8_t  buff_shoot;
//}__attribute__((packed))   gimbal_ctrl_t;
///* STM32���� byte: 21 */
//typedef struct
//{
//	/* ͷ */
//	extVisionSendHeader_t	head_data;
//	/* ���� */
//	gimbal_ctrl_t	gimbal_ctrl;
//	/* β */  
//	//uint8_t   END;			//����λ���ݶ�0xBB
//}__attribute__((packed))	extVisionRecvData_t;
//typedef struct
//{
//	/* ͷ */
//	uint8_t		SOF;		//֡ͷ��ʼλ,�ݶ�0xA0
//	uint8_t		empty_3;
//	uint8_t		DataLen;	//���ݳ���
//	uint16_t	empty;
//	uint16_t	DataID;		//����ID
//}
///* STM32����,һ�ֽ�һ�ֽڷ��� byte: 23 */
//typedef struct //__packed struct
//{
//	uint8_t		mode;   //ģʽ��0װ�װ� 1��� 2��
//    uint8_t		enemy_color; //�з���ɫ��1 red 2 blue
//    uint8_t		is_left;
//    uint8_t		run_left;
//    float		bullet_spd;	//�����ٶ�
//    float		pitch;
//    float		yaw;
//    //float		pitch_offset; // pit�����Ƕ�
//    //float		yaw_offset;  // yaw�����Ƕ�
//}__attribute__((packed))	extVisionSendData_t;
//�Ӿ��Ƕ�
//typedef struct
//{
//	float pitch_angle;
//	float yaw_angle;
//} CTRL;
//extern extVisionRecvData_t VisionRecvData;//�Ӿ����սṹ��

//������
//void Vision_Ctrl(void);
void Vision_Init(void);
void Vision_Read_Data(uint8_t *Usart_Information);		//�Ӿ����ݶ�ȡ����
uint8_t Vision_update_flag(void);						//�Ӿ������Ƿ����
void Vision_clean_flag(void);							//�Ӿ����ݸ���λ��0
void Data_Send_to_Vision(void);
void Limit_Auto_Angle(fp32 *pitch_angle, fp32 *yaw_angle);//�Ƕ��޷�
extern Auto_Gimbal_Ctrl_t *get_AUTO_control_point(void);				//��ȡ����ǶȽṹ��ָ��

#endif


