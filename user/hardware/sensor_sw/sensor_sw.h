#ifndef __SENSOR_SW_H__
#define __SENSOR_SW_H__

#include "main.h"
typedef enum
{
	L,		//向左运动
	S,		//无运动
	R,		//向右运动
}move_dir;

#define Switch_Sensor_GPIO_L GPIOF->IDR & 0x0001//GPIO_ReadOutputDataBit(GPIOH, GPIO_Pin_11)
#define Switch_Sensor_GPIO_R GPIOF->IDR >> 1 & 0x0001//GPIO_ReadOutputDataBit(GPIOH, GPIO_Pin_12)

void Sensor_Init(void);
uint8_t get_Switch_Status( move_dir dir);

#endif

