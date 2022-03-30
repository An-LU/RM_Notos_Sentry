#include "miniPC.h"
#include "remote_control.h"
#include "pid.h"
#include "crc.h"
#include "string.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "INS_Task.h"
#include "usart.h"



//��������
static Auto_Gimbal_Ctrl_t auto_gimbal_ctrl;
//stm32�ϴ�����
static Stm32_Info_t stm32_info;
const static fp32 *ins_angle;

//�Ӿ��Ƿ���������,FALSEû��,TRUE�����µ�
static uint8_t Vision_Get_NewData = FALSE;

//��ȡ����Ƕ�
Auto_Gimbal_Ctrl_t *get_AUTO_control_point(void)
{
	return &auto_gimbal_ctrl;
}
//�Ƕ��޷�
void Limit_Auto_Angle(fp32 *pitch_angle, fp32 *yaw_angle)
{
	*yaw_angle *= 0.1428f;
	if(*yaw_angle > 3.14f)
		*yaw_angle = 3.14f;
	else if(*yaw_angle < -3.14f)
		*yaw_angle = -3.14f;	
	if(*pitch_angle > 3.14f)
		*pitch_angle = 3.14f;
	else if(*pitch_angle < -3.14f)
		*pitch_angle = -3.14f;
}

//�Ӿ����ݶ�ȡ����
void Vision_Read_Data(uint8_t *Recv_Info)
{
	uint16_t vision_data_len = 0;
	uint16_t CMD_ID = 0;
	uint8_t num = 0;
	while(1)
	{
		if(Recv_Info[num] == VISION_HEAD_SOF)
			break;
		if(num >= 70)
			return;		
		num++;
	}

	//֡ͷCRC8��֤
	if(Verify_CRC8_Check_Sum( Recv_Info + num, VISION_LENGTH_SOF))
	{
		vision_data_len = VISION_LENGTH_SOF + VISION_LENGTH_TAIL + VISION_LENGTH_CMD + Recv_Info[1];		//���ݳ���
		//֡βcrc16��֤
		if( Verify_CRC16_Check_Sum( Recv_Info + num, vision_data_len))
		{
			CMD_ID = (uint16_t)Recv_Info[num + 5] | (uint16_t)Recv_Info[num + 6] << 8;
			switch( CMD_ID )
			{
				case GIMBAL_CTRL_ID:
					memcpy( (void *)(&auto_gimbal_ctrl) , (const void *)(&Recv_Info[AutoCtrl_Data_v+num]) , VISION_CTRL_LEN );
					Limit_Auto_Angle( (fp32 *)(&auto_gimbal_ctrl.pitch_angle), (fp32 *)(&auto_gimbal_ctrl.yaw_angle) );
					break;
			}
		}
	}
	Vision_Get_NewData = TRUE;

//	//�ж���ʼ�ֽ�
//	if( *Recv_Info == VISION_HEAD_SOF )
//	{
//		//֡ͷCRC8��֤
//		if(Verify_CRC8_Check_Sum( Recv_Info, VISION_LENGTH_SOF))
//		{
//			vision_data_len = VISION_LENGTH_SOF + VISION_LENGTH_TAIL + VISION_LENGTH_CMD + Recv_Info[1];		//���ݳ���
//			//֡βcrc16��֤
//			if( Verify_CRC16_Check_Sum( Recv_Info, vision_data_len))
//			{
//				CMD_ID = (uint16_t)Recv_Info[5] | (uint16_t)Recv_Info[6] << 8;
//				switch( CMD_ID )
//				{
//					case GIMBAL_CTRL_ID:
//						memcpy( (void *)(&auto_gimbal_ctrl) , (const void *)(&Recv_Info[AutoCtrl_Data_v]) , VISION_CTRL_LEN );
//						Limit_Auto_Angle( (fp32 *)(&auto_gimbal_ctrl.pitch_angle), (fp32 *)(&auto_gimbal_ctrl.yaw_angle) );
//						break;
//				}
//			}
//		}
//		Vision_Get_NewData = TRUE;
//	}
}

//�Ӿ������Ƿ����
uint8_t Vision_update_flag(void)
{
	return Vision_Get_NewData;
}

//�Ӿ����ݸ���λ��0
void Vision_clean_flag(void)
{
	Vision_Get_NewData = FALSE;
}
void Vision_Init(void)
{
	//const volatile fp32 *ins_angle;
	ins_angle = get_INS_angle_point();
	
}
//���·�����̨���ݵ�miniPC
void Update_Data(void)
{
#if ROBOT_MODE == JUDGE_SYSTEM_OFFLINE
	//û�а�װ����ϵͳ
	stm32_info.enemy_color = ROBOT_TYPE;
	stm32_info.bullet_spd = 0;
#elif ROBOT_MODE == JUDGE_SYSTEM_ONLINE
	//�Ѱ�װ����ϵͳ �Ӳ���ϵͳ��ȡ����
	stm32_info.enemy_color = get_Robot_Type_Judge();
	stm32_info.bullet_spd = get_bullet_speed();
#endif	/* ROBOT_MODE */
	stm32_info.mode = ARMOR_PLATE;
	stm32_info.is_left = 0;
	stm32_info.run_left = 0;
	stm32_info.pitch = ins_angle[INS_PITCH_ADDRESS_OFFSET];//*(ins_angle + INS_PITCH_ADDRESS_OFFSET);//pitch_motor->relative_angle;
	stm32_info.yaw = ins_angle[INS_YAW_ADDRESS_OFFSET];//yaw_motor->relative_angle;
}
void PackData(uint16_t cmd_id, uint8_t *p_data ,uint16_t len, uint8_t *tx_buf)
{
	uint16_t frame_length = VISION_LENGTH_SOF + VISION_LENGTH_CMD + len + VISION_LENGTH_TAIL;
	Vision_Frame_Header_t *p_header = (Vision_Frame_Header_t *)tx_buf;
	
	p_header->SOF = VISION_UP_SOF;
	memcpy(&tx_buf[1], (uint8_t*)&len, 1);
	p_header->Seq++;
	//֡ͷCRC8��֤ ���������VISION_LENGTH_SOF-1λ
	Append_CRC8_Check_Sum(tx_buf, VISION_LENGTH_SOF);
	
	memcpy(&tx_buf[VISION_LENGTH_SOF], (uint8_t*)&cmd_id, VISION_LENGTH_CMD);
	memcpy(&tx_buf[VISION_LENGTH_SOF + VISION_LENGTH_CMD], p_data, len);
	Append_CRC16_Check_Sum(tx_buf, frame_length);
}
void Data_Send_to_Vision(void)
{
	uint8_t tx_buf[25] = {0};
	//uint8_t a = sizeof(stm32_info);
	Update_Data();
	PackData( GIMBAL_DATA_ID ,(uint8_t *)&stm32_info ,VISION_DATA_LEN ,tx_buf );
	//��������
	VisionUart_SendData(tx_buf, sizeof(tx_buf));
	memset( &stm32_info, 0, sizeof(stm32_info) );
}


