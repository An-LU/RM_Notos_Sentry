#ifndef __JUDGE_SYSTEM_H__
#define __JUDGE_SYSTEM_H__

#include "main.h"

#define	JUDGE_2021		21
#define	JUDGE_VERSION	JUDGE_2021


#if JUDGE_VERSION == JUDGE_2021

/***************������ID********************

	ID: 0x0001          Byte: 11      ����״̬����                   ����Ƶ�� 1Hz
	ID: 0x0002          Byte: 1       �����������                   ������������
	ID: 0x0003          Byte: 32      ����������Ѫ������             ����Ƶ�� 1Hz
	ID: 0x0004          Byte: 3       ���ڷ���״̬                   ���ڷ������
	ID: 0x0005          Byte: 11      �˹�������ս���ӳ���ͷ�       ����Ƶ�� 1Hz
	ID: 0x0101          Byte: 4       �����¼�����                   �¼��ı����
	ID: 0x0102          Byte: 4       ���ز���վ������ʶ����         �����ı����
	ID: 0X0103          Byte: 2       ���ز���վԤԼ�ӵ�����         �����ӷ��ͣ�����10Hz��RM�Կ���δ���ţ� 
	ID: 0X0104          Byte: 2       ���о�������                   ���淢������
	ID: 0x0105          Byte: 1       ���ڷ���ڵ���ʱ               ����Ƶ�� 1Hz
	ID: 0X0201          Byte: 27      ������״̬����                 ����Ƶ�� 10Hz
	ID: 0X0202          Byte: 16      ʵʱ������������               ����Ƶ�� 50Hz
	ID: 0X0203          Byte: 16      ������λ������                 ����Ƶ�� 10Hz
	ID: 0X0204          Byte: 1       ��������������                 ����״̬�ı����
	ID: 0X0205          Byte: 2       ���л���������״̬����         ����Ƶ�� 10Hz��ֻ�п��л��������ط���
	ID: 0X0206          Byte: 1       �˺�״̬����                   �˺���������
	ID: 0X0207          Byte: 7       ʵʱ�������                   �ӵ��������
	ID: 0X0208          Byte: 6       �ӵ�ʣ�෢����                 ���˻����ڱ�����  ����Ƶ�� 1Hz
	ID: 0X0209          Byte: 4       ������RFID״̬                 ����Ƶ�� 1Hz
	ID: 0x020A          Byte: 12      ���ڻ����˿ͻ���ָ����         ����Ƶ�� 10Hz
	ID: 0X0301          Byte: n       �����˼佻������               ���ͷ���������    ����Ƶ�� 10Hz
	ID: 0x0302          Byte: n       �Զ���������������ݽӿ�       �ͻ��˴�������    ����Ƶ�� 30Hz
	ID: 0x0303          Byte: 15      �ͻ���С��ͼ��������           ��������
	ID: 0x0304          Byte: 12      ���̡������Ϣ                 ͨ��ͼ�����ڷ���
	ID: 0x0305			Byte: 10	  �ͻ���С��ͼ������Ϣ
	
********************************************/

#define JUDGE_HEAD_SOF		0xA5
#define JUDGE_UP_SOF		0xA0
#define	JUDGE_LENGTH_SOF	5	//֡ͷ����
#define	JUDGE_LENGTH_TAIL	2	//֡β����
#define	JUDGE_LENGTH_CMD	2
#define JUDGE_CTRL_LEN		14	//�������ݴ�С
#define JUDGE_DATA_LEN		16	//�ϴ����ݴ�С
/*******************������ID**************************/
#define       Judge_Game_StatusData              0x0001 
#define       Judge_Game_ResultData              0x0002 
#define       Judge_Robot_HP                     0x0003 
#define       Judge_Dart_Launch                  0x0004
#define       Judge_AI_ChallengeBuff             0x0005
#define       Judge_Event_Data                   0x0101
#define       Judge_Supply_Station               0x0102
//#define       Judge_Request_Recharge             0x0103(�Կ���δ����)
#define       Judge_Referee_Warning              0x0104
#define       Judge_Dart_Countdown               0x0105
#define       Judge_Robot_State                  0x0201
#define       Judge_Power_Heat                   0x0202
#define       Judge_Robot_Position               0x0203
#define       Judge_Robot_Buff                   0x0204
#define       Judge_Aerial_Energy                0x0205
#define       Judge_Injury_State                 0x0206
#define       Judge_RealTime_Shoot               0x0207
#define       Judge_Remaining_Rounds             0x0208
#define       Judge_Robot_RFID                   0x0209
#define       Judge_Dart_Client                  0x020A
#define       Judge_Robot_Communicate            0x0301
#define       Judge_User_Defined                 0x0302
#define       Judge_Map_Interaction              0x0303
#define       Judge_KeyMouse_Message             0x0304
#define       Judge_Client_Map                   0x0305
/*****************************************************/
typedef enum
{
	Red_Robor = 1,
	Blue_Robor = 2
}Robot_Type_e;
//֡ͷ����ƫ����
typedef enum
{
	SOF_offset = 0,
	DataLength_offset = 1,
	Seq_offset = 3,
	CRC8_offset = 4
}Farme_Header_Offset;
/**********����ϵͳ���ݽṹ��***************/
/*	֡ͷ	*/
typedef __packed struct
{
	uint8_t  SOF;						
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
}Frame_Header_t;

/* ID: 0x0001  Byte:  11    ����״̬���� */
typedef __packed struct 
{ 
	uint8_t game_type : 4;				//����ģʽ
	uint8_t game_progress : 4;			//��ǰ�����׶�
	uint16_t stage_remain_time;			//��ǰ�׶�ʣ��ʱ�� ��λs
	uint64_t SyncTimeStamp;
} ext_game_state_t; 

/* ID: 0x0002  Byte:  1    ����������� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  32    ����������Ѫ������ */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;   
	uint16_t red_2_robot_HP;  
	uint16_t red_3_robot_HP;  
	uint16_t red_4_robot_HP;  
	uint16_t red_5_robot_HP;  
	uint16_t red_7_robot_HP;  
	uint16_t red_outpost_HP; 
	uint16_t red_base_HP; 
	
	uint16_t blue_1_robot_HP;   
	uint16_t blue_2_robot_HP;   
	uint16_t blue_3_robot_HP;   
	uint16_t blue_4_robot_HP;   
	uint16_t blue_5_robot_HP;     
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP; 
	uint16_t blue_base_HP;   
}  ext_game_robot_HP_t; 

/* ID: 0x0004  Byte:  3    ���ڷ���״̬ */
typedef __packed struct 
{  
	uint8_t dart_belong; 				//������ڵĶ���
	uint16_t stage_remaining_time; 		//����ʱʣ��ı���ʱ��
} ext_dart_status_t;

/* ID: 0x0005  Byte:  11   �˹�������ս�� buff and debuff */
typedef __packed struct
{ 
	uint8_t F1_zone_status:1;  							//�����Ƿ����ı�־
	uint8_t F1_zone_buff_debuff_status:3;   //�췽��Ѫ
	
	uint8_t F2_zone_status:1;  
	uint8_t F2_zone_buff_debuff_status:3;   //�췽��ҩ
	
	uint8_t F3_zone_status:1;  
	uint8_t F3_zone_buff_debuff_status:3;		//������Ѫ  
	
	uint8_t F4_zone_status:1;  
	uint8_t F4_zone_buff_debuff_status:3;  	//������ҩ
	
	uint8_t F5_zone_status:1;  
	uint8_t F5_zone_buff_debuff_status:3;  	//��ֹ�����
	
	uint8_t F6_zone_status:1;  
	uint8_t F6_zone_buff_debuff_status:3;  	//��ֹ�ƶ���
	
	uint16_t red1_bullet_left;							//������ʣ�൯��
	uint16_t red2_bullet_left;
	uint16_t blue1_bullet_left;
	uint16_t blue2_bullet_left;
	
} ext_ICRA_buff_debuff_zone_status_t; 

/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	uint32_t event_type;										//0~4bit���ã�5~31����
	
} ext_event_data_t; 

/* ID: 0x0102  Byte:  4    ���ز���վ������ʶ���� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;						//����վ��ID
	uint8_t supply_robot_id;								//��ǰ����������ID
	uint8_t supply_projectile_step;					//����վ������״̬
	
	uint8_t supply_projectile_num;					//������Ŀ
} ext_supply_projectile_action_t; 

/* ID: 0x0104  Byte: 2   ����ϵͳ������Ϣ */
typedef __packed struct 
{ 
	uint8_t level; 													//����ȼ�
	uint8_t foul_robot_id;									//���������ID
}  ext_referee_warning_t;  

/* ID: 0x0105  Byte:1  ���ڷ���ڵ���ʱ */
typedef __packed struct 
{ 
	uint8_t dart_remaining_time; 						//15s����ʱ
}  ext_dart_remaining_time_t;  


/* ID: 0X0201  Byte: 27    ������״̬���� */
/* �������� ID�� 
	1���췽Ӣ�ۻ����ˣ� 
	2���췽���̻����ˣ� 
	3/4/5���췽���������ˣ� 
	6���췽���л����ˣ� 
	7���췽�ڱ������ˣ� 
	8���췽���ڻ����ˣ� 
	9���췽�״�վ�� 
	101������Ӣ�ۻ����ˣ� 
	102���������̻����ˣ� 
	103/104/105���������������ˣ� 106���������л����ˣ� 
	107�������ڱ������ˣ� 
	108���������ڻ����ˣ� 
	109�������״�վ�� 
*/
//0������17mmǹ�ܣ�1����42mmǹ��
typedef __packed struct 			
{ 
	uint8_t robot_id;   								//��������ID��������У�鷢��
	uint8_t robot_level;  								//�����˵ȼ� 1һ����2������3����
	uint16_t remain_HP; 								//������ʣ��Ѫ��
	uint16_t max_HP; 									//����������Ѫ��
	uint16_t shooter_heat0_cooling_rate;  				//������ 17mm 1�ӵ�������ȴ�ٶ� ��λ /s
	uint16_t shooter_heat0_cooling_limit;   			//������ 17mm 1�ӵ���������
	uint16_t shooter_heat0_speed_limit;					//������17mm 1ǹ�������ٶ�
	uint16_t shooter_heat1_cooling_rate;   				//������17mm 2ǹ��ÿ����ȴֵ
	uint16_t shooter_heat1_cooling_limit;  				//������17mm 2ǹ����������
	uint16_t shooter_heat1_speed_limit;					//������17mm 2ǹ�������ٶ�
	uint16_t shooter_42mm_cooling_rate;   				//������42mm ǹ��ÿ����ȴֵ
	uint16_t shooter_42mm_cooling_limit;  				//������42mm ǹ����������
	uint16_t shooter_42mm_speed_limit;					//������42mm ǹ�������ٶ�
	uint16_t max_chassis_power; 						//�����˵��̹�������
	uint8_t mains_power_gimbal_output : 1;  			//���ص�Դ��̨�����״̬
	uint8_t mains_power_chassis_output : 1;  			//���ص�Դ���̿����״̬
	uint8_t mains_power_shooter_output : 1; 			//���ص�Դǹ�ܿ����״̬
} ext_game_robot_state_t;   

/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   											//���������ѹ mV
	uint16_t chassis_current;    									//����������� mA
	float chassis_power;   												//����������� 
	uint16_t chassis_power_buffer;								//���̻��幦��ʣ��
	uint16_t shooter_heat0;												//17mmǹ��1Ŀǰ����
	uint16_t shooter_heat1;  											//17mmǹ��2Ŀǰ����
	uint16_t shooter_heat2_42mm; 								//42mmǹ��Ŀǰ����
} ext_power_heat_data_t; 

/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct 
{   
	float x;   																		//������λ��x
	float y;   																		//������λ��y
	float z;   																		//������λ��z
	float yaw; 																		//������ƫ����
} ext_game_robot_pos_t; 

/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct 
{ 
	uint8_t power_rune_buff;											// 
} ext_buff_musk_t; 

/* ID: 0x0205  Byte:  1    ���л���������״̬���� */
typedef __packed struct 
{ 
	uint8_t attack_time; 													//�ɹ���ʱ��
} aerial_robot_energy_t; 

/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 												//����װ�װ�ID
	uint8_t hurt_type : 4; 												//Ѫ���۳�����
} ext_robot_hurt_t; 

/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef __packed struct 
{ 
	uint8_t bullet_type;   												//�ӵ�����
	uint8_t shooter_id;   												//�������id
	uint8_t bullet_freq;   												//�ӵ���Ƶ HZ
	float bullet_speed;  													//�ӵ����� m/s
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  6    �ӵ�ʣ������ */
typedef __packed struct 
{ 
	uint16_t bullet_remaining_num_17mm;   							//�ӵ�ʣ�෢����
	uint16_t bullet_remaining_num_42mm;   							//�ӵ�ʣ�෢����
	uint16_t coin_remain;   														//ʣ����
}  ext_bullet_remaining_t; 

/* ID: 0x0209  Byte:  4    FRID״̬ */
typedef __packed struct 
{ 
	uint32_t rfid_status ;												//RFID��״̬
}  ext_rfid_status_t; 


/*���ڻ����˿ͻ���ָ������*/
/* ID: 0x020a  Byte:  4    FRID״̬ */
typedef __packed struct {  
	uint8_t dart_launch_opening_status;  
	uint8_t dart_attack_target;  
	uint16_t target_change_time; 
	uint16_t operate_launch_cmd_time; 
} ext_dart_client_cmd_t; 

/* ID: 0X0301  Byte: n  �����˼佻������ */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 

/********************************************/

uint8_t get_Robot_Type_Judge(void);		//��ȡ�������˺췽����
float get_bullet_speed(void);			//��ȡ�ӵ�����

void Judge_Read_Data(uint8_t *Judge_Usart_Info);

//#elif JUDGE_VERSION == JUDGE_2021


#endif

#endif

