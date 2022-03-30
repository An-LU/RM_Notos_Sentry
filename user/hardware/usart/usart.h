#ifndef __USART_H__
#define __USART_H__

#include "main.h"

#include "sys.h"  



/* ���ջ��� */
#define		VISION_BUFFER_LEN           100   //��΢�����
#define		JUDGE_BUFFER_LEN			100
/* �������ݰ���С */
#define		VISION_SEND_LEN		20


/*
usart6-->T:PG14	DMA2 ch6 Stream6 7
		 R:PG9  DMA2 ch6 Stream1 2

usart8-->T:PE1	DMA1 ch5 Stream0
		 R:PE0	DMA1 ch5 Stream6

usart7-->T:PE8	DMA1 ch5 Stream1
		 R:PE7  DMA1 ch5 Stream3
		
*/
#define	Vision_Uart		UART7
#define	Judge_Uart		UART8

#define	Vision_Uart_GPIO		GPIOE
#define	Vision_Uart_Tx			GPIO_Pin_8
#define	Vision_Uart_Rx			GPIO_Pin_7
#define	Judge_Uart_GPIO			GPIOE
#define	Judge_Uart_Tx			GPIO_Pin_1
#define	Judge_Uart_Rx			GPIO_Pin_0


/* �Ӿ� */
void Vision_Usart_Init(void);
void VisionUart_SendData(uint8_t *Data, uint8_t length);
/* ����ϵͳ */
void Judge_Usart_Init(void);
void JudgeUart_SendData(uint8_t *Data, uint8_t length);

void UART7_SendChar(uint8_t cData);
void UART8_SendChar(uint8_t cData);
#endif

