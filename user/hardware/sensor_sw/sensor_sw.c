#include "sensor_sw.h"
#include "stm32f4xx.h"
uint8_t status;
void Sensor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
	
    GPIO_Init(GPIOF, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOH, GPIO_Pin_11|GPIO_Pin_12);
}
//获取光电开关状态
uint8_t get_Switch_Status( move_dir dir)
{
	//uint8_t status;
	uint16_t io = GPIOF->IDR & 0x0001;
	switch(dir)
	{
		case L:
			//status = GPIO_ReadOutputData(GPIOF);
			status = Switch_Sensor_GPIO_L;
			break;
		case R:
			status = Switch_Sensor_GPIO_R;
			break;
		case S:
			status = 2;
			break;
	}
	return status;
}


