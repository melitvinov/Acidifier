/***********************************************************************************
* ������������ ���� : �������� ������
* ������			: ������ ��������� PH - 3 ������
* ���� ��������		: 30.05.2018
* �����				: ��������
* ������ ��			: 01 
************************************************************************************/

#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f10x.h"

/*================================================================
	�������
==================================================================*/
#define GETBIT(value, bit) 	(((value) & (1<<(bit)))>0 ? 1 : 0)
#define SETBIT(value, bit) 	((value) |= (1<<(bit)))
#define CLRBIT(value, bit) 	((value) &= ~(1<<(bit)))
#define SWAP_W(A)			((((A) & 0xFF00) >> 8)  | (((A) & 0x00FF) << 8)) 

/*================================================================
	�������� ������� ���������� ���������� ������������
==================================================================*/

//-- �����
#define PORT_LED_SYS					GPIOC
#define PORT_LED_WORK_OK				GPIOB
#define PORT_LED_TAR_P1					GPIOC
#define PORT_LED_TAR_P2					GPIOC
#define PORT_LED_NO_WATER				GPIOA
#define PORT_LED_ERR_SETUP_PH_TIMEOUT	GPIOA
#define PORT_LED_ERR_SENSORS			GPIOA
#define PORT_LED_VALVE					GPIOA

//-- ID �������
#define PIN_LED_SYS						3
#define PIN_LED_WORK_OK					0
#define PIN_LED_TAR_P1					5
#define PIN_LED_TAR_P2					4
#define PIN_LED_NO_WATER				6
#define PIN_LED_ERR_SETUP_PH_TIMEOUT	4
#define PIN_LED_ERR_SENSORS				5
#define PIN_LED_VALVE					7

/*================================================================
	�������� ������� ���������� ���������� ���������� ��� 
	7-���������� �����������
==================================================================*/
//-- �����
#define PORT_LED_CLOCK	GPIOC
#define PORT_LED_LATCH	GPIOC
#define PORT_LED_SERIAL	GPIOC
//-- ID �������
#define PIN_LED_CLOCK	13
#define PIN_LED_LATCH	14
#define PIN_LED_SERIAL	15

/*================================================================
	���� I2C
==================================================================*/
#define PORT_I2C		GPIOB
#define PIN_SCL			6
#define PIN_SDA			7

/*================================================================
	�������� ������ ������������� ������ ���������� �� ���� RS-485
==================================================================*/
//-- �����
#define PORT_ADDR_1   GPIOB
#define PORT_ADDR_2   GPIOB
#define PORT_ADDR_4   GPIOB
#define PORT_ADDR_8   GPIOB
//-- ID �������
#define PIN_ADDR_1   12
#define PIN_ADDR_2   13
#define PIN_ADDR_4   14
#define PIN_ADDR_8   15

/*================================================================
	�������� ������ R/W RS-485
==================================================================*/
//-- �����
#define PORT_RS485_RW GPIOA
//-- ID �������
#define OUT_RS485_RW	8

/*================================================================
	�������� ���������� ������
==================================================================*/
//-- ����� ���������� ������
#define PORT_AIN1   GPIOA
#define PORT_AIN2   GPIOA

//-- ID ������� ���������� ������
#define PIN_AIN1   0
#define PIN_AIN2   1

//-- ID �������� - ID ������ ���
#define PH1   ADC_Channel_0
#define PH2   ADC_Channel_1

/*================================================================
	�������� �������� ������ � ������
==================================================================*/
#define PORT_BTN_ESC  	GPIOC
#define PORT_BTN_PLUS   GPIOC
#define PORT_BTN_MINUS  GPIOC

#define PIN_BTN_ESC  	12
#define PIN_BTN_PLUS   	11
#define PIN_BTN_MINUS   10

#define PORT_SENS_WATER GPIOB
#define PIN_SENS_WATER  11

/*================================================================
	�������� ������� ���������� �������
==================================================================*/
#define PORT_RELAY_PUMP		GPIOC
#define PIN_RELAY_PUMP		7

/*================================================================
	�������� ������� ���������� ���� �������
==================================================================*/
#define PORT_RELAY_ALARM  	GPIOC
#define PIN_RELAY_ALARM   	6

/*================================================================
	�������� ������� ���������� ���� �������
==================================================================*/
#define PORT_RELAY_VALVE  	GPIOC
#define PIN_RELAY_VALVE   	8

/*================================================================
	������ � EEPROM ��� ����������
==================================================================*/
#define EEADR_SETUP_PH					100
#define EEADR_COEF_PROP					102
#define EEADR_COEF_INTEG				104
#define EEADR_COEF_DIFF					106
#define EEADR_TIMEOUT_REGULATOR_ON_SEC	108
#define EEADR_TIMEOUT_ERROR_PH_SEC		110
#define EEADR_REG_CYCLETIME_SEC			112
#define EEADR_DELAY_PUMP_OFF_SEC		114

#define EEADR_TAR_PH1_P1_ADC			200

#define EEADR_OPT_TABLE					300

/*================================================================
	������ ������ ����������
==================================================================*/
typedef enum _work_modes
{
	Mode_RegulatorPh = 1,
	Mode_Calibrating_PH1,
	Mode_Calibrating_PH2,
} EWorkMode;

#endif
