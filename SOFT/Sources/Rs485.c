/***********************************************************************************
* ������������ ������	: ������ ����� �� ���� RS-485
*-----------------------------------------------------------------------------------
* ������				: 1.0
* �����					: ��������
* ���� ��������			: 23.05.2018
************************************************************************************/

//--- INCLUDES -------------------
#include "Rs485.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

//#include "Modbus.h"

// --- DEFINES -------------------

// --- TYPES ---------------------

//--- CONSTANTS ------------------

//--- GLOBAL VARIABLES -----------

//--- EXTERN ---------------------
//extern TPacketBuffer OneMbusMessage;
//extern EMBusState MbusState;
//extern bool IsPacketReady;
extern void On_Usart_ReceiveChar( char chr );

//--- IRQ ------------------------

/*******************************************************
���������� �� ������� �� USART1
********************************************************/
void USART1_IRQHandler(void)
{
	if( USART1->SR & USART_IT_RXNE ) //���������� �� ������ ������
	{
		if ((USART1->SR & (USART_FLAG_NE|USART_FLAG_FE|USART_FLAG_PE)) == 0) //��������� ��� �� ������
		{    	
			On_Usart_ReceiveChar( (char) (USART_ReceiveData(USART1)& 0xFF) ); //��������� ������ � ����� );
		}
		else
		{
			// ������ ������
			USART_ReceiveData(USART1);
		}
	}
}

//--- FUNCTIONS ------------------

/*******************************************************
�������		: ������������� RS-485
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Rs485_Init(void)
{
	// ������������� ������� Read/Write
	// ������� ������ RW �� ����� � �������� �����������
	GPIO_PinConfigure( PORT_RS485_RW, OUT_RS485_RW, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
    // ��������� ������� ������
	GPIO_PinWrite( PORT_RS485_RW, OUT_RS485_RW, 0 );
	
	// ������������� USART1
	
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 0x341;		// Baudrate for 9600 on 8Mhz
	USART1->CR1  |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // USART1 ON, TX ON, RX ON, RXNE Int ON

	RCC->APB2ENR  	|= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; 	// GPIOA Clock ON. Alter function clock ON
	
	GPIOA->CRH	&= ~GPIO_CRH_CNF9; 				// Clear CNF bit 9
	GPIOA->CRH	|= GPIO_CRH_CNF9_1;				// Set CNF bit 9 to 10 - AFIO Push-Pull
	GPIOA->CRH	|= GPIO_CRH_MODE9_0;				// Set MODE bit 9 to Mode 01 = 10MHz
	
	GPIOA->CRH	&= ~GPIO_CRH_CNF10;	// Clear CNF bit 9
	GPIOA->CRH	|= GPIO_CRH_CNF10_0;	// Set CNF bit 9 to 01 = HiZ
	GPIOA->CRH	&= ~GPIO_CRH_MODE10;	// Set MODE bit 9 to Mode 01 = 10MHz

	NVIC_EnableIRQ (USART1_IRQn);
	__enable_irq ();
}

/*******************************************************
�������		: ������������ RS-485 �� ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void prvRs485_ToWrite(void)
{
	GPIO_PinWrite( PORT_RS485_RW, OUT_RS485_RW, 1 );
}

/*******************************************************
�������		: ������������ RS-485 �� ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void prvRs485_ToRead(void)
{
	GPIO_PinWrite( PORT_RS485_RW, OUT_RS485_RW, 0 );
}
	
/*******************************************************
�������		: �������� ������� � USART1
�������� 1	: ������������ ������
�����. ����.: ���
********************************************************/
void prvUsart1_Send( char chr )
{
//	while(!(USART1->SR & USART_SR_TC)) {}
	while(!(USART1->SR & USART_SR_TXE)) {}
	
	USART1->DR = chr;
}

/*******************************************************
�������		: �������� ������ � RS-485
�������� 1	: ������������ �����
�������� 2	: ����� ������ � ��������
�����. ����.: ���
********************************************************/
void RS485_SendBuf( const char * pBuf, uint8_t count )
{
	prvRs485_ToWrite();
	vTaskDelay(DELAY_BEFORE_SEND);
	
	for( int i=0; i<count; i++ )
	{
		prvUsart1_Send( pBuf[i] );
	}

	while(!(USART1->SR & USART_SR_TC)) {}
//	vTaskDelay(DELAY_AFTER_SEND);
	prvRs485_ToRead();
}


