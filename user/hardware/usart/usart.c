#include "usart.h"
#include "stm32f4xx_dma.h"
#include "miniPC.h"
#include "judge_system.h"
//#include "led.h"
#include "string.h"
#include "crc.h"

static uint8_t	Judge_Buffer[ JUDGE_BUFFER_LEN ] = { 0 };
static uint8_t  Vision_Buffer[ VISION_BUFFER_LEN ] = {0};
static int Usart7_Clean_IDLE_Flag = 0;
static int Usart8_Clean_IDLE_Flag = 0;

void Vision_Usart_DMA_Init(void)
{
	DMA_InitTypeDef DMAInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	
	DMA_DeInit( DMA1_Stream3 );
	//while(DMA_GetCmdStatus(DMA1_Stream3));
	DMAInitStruct.DMA_Channel = DMA_Channel_5;//ͨ��ѡ��
	
	DMAInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;//�������赽�洢��

	DMAInitStruct.DMA_PeripheralBaseAddr  = (uint32_t)&(UART7->DR);//DMA�����ַ
	DMAInitStruct.DMA_Memory0BaseAddr     = (uint32_t)Vision_Buffer;//DMA �洢��0��ַ����������
	
	DMAInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;//�洢��������ģʽ
	DMAInitStruct.DMA_BufferSize = 100;//���ݴ����� 
	DMAInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMAInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMAInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMAInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMAInitStruct.DMA_Mode = DMA_Mode_Circular;// ����ģʽ 
	DMAInitStruct.DMA_Priority = DMA_Priority_VeryHigh;//��ߵ����ȼ�
	DMAInitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;//������FIFO
	DMAInitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//FIFO ��ֵѡ��
	DMAInitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMAInitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���

	DMA_Init( DMA1_Stream3, &DMAInitStruct );	
	DMA_Cmd( DMA1_Stream3, ENABLE);
}
void Vision_Usart_Init()
{
	USART_InitTypeDef UsartInitStruct;
	GPIO_InitTypeDef GPIOInitStruct;
	NVIC_InitTypeDef NvicInitStruct;
	/* ʹ��ʱ�� */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART7, ENABLE );
	/* ���Ÿ��� */
	GPIO_PinAFConfig( Vision_Uart_GPIO, GPIO_PinSource8, GPIO_AF_UART7 );
	GPIO_PinAFConfig( Vision_Uart_GPIO, GPIO_PinSource7, GPIO_AF_UART7 ); 
	/*	GPIO TxRx */
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOInitStruct.GPIO_OType = GPIO_OType_PP;//���츴�����
	GPIOInitStruct.GPIO_Pin = Vision_Uart_Tx | Vision_Uart_Rx;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init( Vision_Uart_GPIO, &GPIOInitStruct );
	/* �������� */
	UsartInitStruct.USART_BaudRate = 115200;   //������
	UsartInitStruct.USART_WordLength = USART_WordLength_8b;//�ֳ�8����
	UsartInitStruct.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	UsartInitStruct.USART_Parity = USART_Parity_No;//����żУ��
	UsartInitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//�շ�ģʽ
	UsartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������

	USART_Init( Vision_Uart, &UsartInitStruct );
	USART_Cmd( Vision_Uart, ENABLE );

	//ʹ�ܴ��ڿ����ж�USART_IT_IDLE,������һ�����ݷ������֮�󣬲Ŵ��������ж�
	USART_ITConfig( Vision_Uart, USART_IT_IDLE, ENABLE );  

	//DMA���������ж�
	USART_DMACmd( Vision_Uart, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( Vision_Uart, USART_DMAReq_Tx, ENABLE );

	Vision_Usart_DMA_Init();

	//�ж�����
	NvicInitStruct.NVIC_IRQChannel = UART7_IRQn;
	NvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;//���ȼ�
	NvicInitStruct.NVIC_IRQChannelSubPriority = 0;
	NvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NvicInitStruct );	
}
void Judge_Usart_DMA_Init(void)
{
	DMA_InitTypeDef DMAInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	
	DMA_DeInit( DMA1_Stream6 );
	//while(DMA_GetCmdStatus(DMA1_Stream6));
	DMAInitStruct.DMA_Channel = DMA_Channel_5;//ͨ��ѡ��
	
	DMAInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;//�������赽�洢��

	DMAInitStruct.DMA_PeripheralBaseAddr  = (uint32_t)&(UART8->DR);//DMA�����ַ
	DMAInitStruct.DMA_Memory0BaseAddr     = (uint32_t)Judge_Buffer;//DMA �洢��0��ַ����������
	
	DMAInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;//�洢��������ģʽ
	DMAInitStruct.DMA_BufferSize = 100;//���ݴ����� 
	DMAInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMAInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMAInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMAInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMAInitStruct.DMA_Mode = DMA_Mode_Circular;// ����ģʽ 
	DMAInitStruct.DMA_Priority = DMA_Priority_VeryHigh;//��ߵ����ȼ�
	DMAInitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;//������FIFO
	DMAInitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//FIFO ��ֵѡ��
	DMAInitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMAInitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���

	DMA_Init( DMA1_Stream6, &DMAInitStruct );	
	DMA_Cmd( DMA1_Stream6, ENABLE);
}
void Judge_Usart_Init(void)
{
	USART_InitTypeDef UsartInitStruct;
	GPIO_InitTypeDef GPIOInitStruct;
	NVIC_InitTypeDef NvicInitStruct;
	/* ʹ��ʱ�� */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART8, ENABLE );
	/* ���Ÿ��� */
	GPIO_PinAFConfig( Vision_Uart_GPIO, GPIO_PinSource0, GPIO_AF_UART8 );
	GPIO_PinAFConfig( Vision_Uart_GPIO, GPIO_PinSource1, GPIO_AF_UART8 ); 
	/*	GPIO TxRx */
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOInitStruct.GPIO_OType = GPIO_OType_PP;//���츴�����
	GPIOInitStruct.GPIO_Pin = Judge_Uart_Tx | Judge_Uart_Rx;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init( Vision_Uart_GPIO, &GPIOInitStruct );
	/* �������� */
	UsartInitStruct.USART_BaudRate = 115200;   //������
	UsartInitStruct.USART_WordLength = USART_WordLength_8b;//�ֳ�8����
	UsartInitStruct.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	UsartInitStruct.USART_Parity = USART_Parity_No;//����żУ��
	UsartInitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//�շ�ģʽ
	UsartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������

	USART_Init( Judge_Uart, &UsartInitStruct );
	USART_Cmd( Judge_Uart, ENABLE );

	//ʹ�ܴ��ڿ����ж�USART_IT_IDLE,������һ�����ݷ������֮�󣬲Ŵ��������ж�
	USART_ITConfig( Judge_Uart, USART_IT_IDLE, ENABLE );  

	//DMA���������ж�
	USART_DMACmd( Judge_Uart, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( Judge_Uart, USART_DMAReq_Tx, ENABLE );

	Judge_Usart_DMA_Init();

	//�ж�����
	NvicInitStruct.NVIC_IRQChannel = UART8_IRQn;
	NvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;//���ȼ�
	NvicInitStruct.NVIC_IRQChannelSubPriority = 0;
	NvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NvicInitStruct );	
}


//�Ӿ������жϷ�����
void UART7_IRQHandler(void)
{	
//	uint8_t Data[10] = {'0','1','2','3','4','5','6','7','8','9'};
	if(USART_GetITStatus(Vision_Uart,USART_IT_IDLE)!=RESET)//��⵽������·
	{		
		//����ST�ٷ��ֲ�,��һ��SR��DR�Ĵ���,IDLE�����ٴ�ʹ��,�����һֱ�����ж�,�ͻ�����ڽ����ж�û����
		Usart7_Clean_IDLE_Flag = UART7->SR ;
		Usart7_Clean_IDLE_Flag = UART7->DR ;	
		
		DMA_Cmd(DMA1_Stream3,DISABLE );
		
		Usart7_Clean_IDLE_Flag = VISION_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);

		Vision_Read_Data(Vision_Buffer);//�����Ӿ�����	
		
		memset(Vision_Buffer, 0, 100);
//		Data_Send_to_Vision();
		DMA_Cmd(DMA1_Stream3,ENABLE);
	}
}
//����ϵͳ�����жϷ�����
void UART8_IRQHandler(void)
{	
	if(USART_GetITStatus(Judge_Uart,USART_IT_IDLE)!=RESET)//��⵽������·
	{		
		//����ST�ٷ��ֲ�,��һ��SR��DR�Ĵ���,IDLE�����ٴ�ʹ��,�����һֱ�����ж�,�ͻ�����ڽ����ж�û����
		Usart8_Clean_IDLE_Flag = Judge_Uart->SR ;
		Usart8_Clean_IDLE_Flag = Judge_Uart->DR ;	
		
		DMA_Cmd(DMA1_Stream6,DISABLE );
		
		Usart8_Clean_IDLE_Flag = JUDGE_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream6);

		Judge_Read_Data(Judge_Buffer);//�����Ӿ�����	
		
		memset(Judge_Buffer, 0, 100);
		
		DMA_Cmd(DMA1_Stream6,ENABLE);//D1S2
	}
}
//���ڷ��ͺ�����һ�η���һ���ֽ�����
void UART7_SendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART7, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART7, cData );   
}
void VisionUart_SendData(uint8_t *Data, uint8_t length)
{
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		UART7_SendChar(*Data++);
		
	}
}
//���ڷ��ͺ�����һ�η���һ���ֽ�����
void UART8_SendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART8, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART8, cData );   
}
void JudgeUart_SendData(uint8_t *Data, uint8_t length)
{
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		UART8_SendChar(*Data++);
		
	}
}

