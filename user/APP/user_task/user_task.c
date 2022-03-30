/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
  * @brief      һ����ͨ������������豸�޴����̵�1Hz��˸,Ȼ���ȡ��̬��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "User_Task.h"
#include "main.h"
#include "Gimbal_Task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "led.h"
#include "chassis_task.h"
#include "Detect_Task.h"
#include "INS_Task.h"
#include "oled.h"
#define user_is_error() toe_is_error(errorListLength)

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

int fputc(int ch, FILE *f)  
{  
    USART2->DR = (u8) ch;  
    /* Loop until the end of transmission */  
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {  
    }
    return ch;  
}//printf������д
//��̬�� ��λ��
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};

void UserTask(void *pvParameters)
{

    const volatile fp32 *angle;
	const chassis_control *moto;
	const motor_measure_t *yaw_mode;
    //��ȡ��̬��ָ��
    angle = get_INS_angle_point();
	moto = get_motor_point();
	yaw_mode = get_Yaw_Gimbal_Motor_Measure_Point();
	float moto_speed,moto_accel;
    while (1)
    {

        //��̬�� ��rad ��� �ȣ����������̬�ǵĵ�λΪ�ȣ������ط�����̬�ǣ���λ��Ϊ����
        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;

        if (!user_is_error())
        {
            led_green_on();
        }
		moto_accel = moto->chassis_motor.accel;
		moto_speed = moto->chassis_motor.speed;
		//moto_speed = moto->vx*moto->vx*12.31;
		//USART_SendData(USART2,(unsigned char)yaw_mode->speed_rpm);
		printf("power_out: %f \n\n",moto_speed);
		//printf("moto_speed: %f \n\n",moto->motor_chassis[0].speed);
		//printf("moto_speed: %f \n\n",-moto->motor_chassis[1].speed);
		//printf("moto_speed: %f \n\n",moto->motor_chassis[2].speed);
        //printf("moto_speed: %f \n\n",moto_speed);
		vTaskDelay(500);
        //led_green_off();
        //vTaskDelay(500);
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
