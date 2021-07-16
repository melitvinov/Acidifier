/***********************************************************************************
* Заголовочный файл : Описание железа
* Проект			: Модуль измерения PH - 3 канала
* Дата создания		: 30.05.2018
* Автор				: Тимофеев
* Версия ПП			: 01 
************************************************************************************/

#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f10x.h"

/*================================================================
	Макросы
==================================================================*/
#define GETBIT(value, bit) 	(((value) & (1<<(bit)))>0 ? 1 : 0)
#define SETBIT(value, bit) 	((value) |= (1<<(bit)))
#define CLRBIT(value, bit) 	((value) &= ~(1<<(bit)))
#define SWAP_W(A)			((((A) & 0xFF00) >> 8)  | (((A) & 0x00FF) << 8)) 

/*================================================================
	Описание выходов управления системными светодиодами
==================================================================*/

//-- Порты
#define PORT_LED_SYS					GPIOC
#define PORT_LED_WORK_OK				GPIOA
#define PORT_LED_TAR_P1					GPIOA
#define PORT_LED_TAR_P2					GPIOA
#define PORT_LED_NO_WATER				GPIOA
#define PORT_LED_ERR_REGULATOR			GPIOC
#define PORT_LED_ERR_SETUP_PH_TIMEOUT	GPIOC
#define PORT_LED_ERR_SENSORS			GPIOB

#define PORT_LED_MOVE_PH_PLUS			GPIOB
#define PORT_LED_MOVE_PH_MINUS			GPIOB
#define PORT_LED_STOP_PH_PLUS			GPIOB
#define PORT_LED_STOP_PH_MINUS			GPIOB

//-- ID выводов
#define PIN_LED_SYS						3
#define PIN_LED_WORK_OK					4
#define PIN_LED_TAR_P1					5
#define PIN_LED_TAR_P2					6
#define PIN_LED_NO_WATER				7
#define PIN_LED_ERR_REGULATOR			4
#define PIN_LED_ERR_SETUP_PH_TIMEOUT	5
#define PIN_LED_ERR_SENSORS				0

#define PIN_LED_MOVE_PH_PLUS			1
#define PIN_LED_MOVE_PH_MINUS			2
#define PIN_LED_STOP_PH_PLUS			9
#define PIN_LED_STOP_PH_MINUS			8

/*================================================================
	Описание выходов управления сдвиговыми регистрами для 
	7-сегментных индикаторов
==================================================================*/
//-- Порты
#define PORT_LED_CLOCK	GPIOC
#define PORT_LED_LATCH	GPIOC
#define PORT_LED_SERIAL	GPIOC
//-- ID выводов
#define PIN_LED_CLOCK	13
#define PIN_LED_LATCH	14
#define PIN_LED_SERIAL	15

/*================================================================
	Порт I2C
==================================================================*/
#define PORT_I2C		GPIOB
#define PIN_SCL			6
#define PIN_SDA			7

/*================================================================
	Описание входов переключателя адреса устройства на шине RS-485
==================================================================*/
//-- Порты
#define PORT_ADDR_1   GPIOB
#define PORT_ADDR_2   GPIOB
#define PORT_ADDR_4   GPIOB
#define PORT_ADDR_8   GPIOB
//-- ID выводов
#define PIN_ADDR_1   12
#define PIN_ADDR_2   13
#define PIN_ADDR_4   14
#define PIN_ADDR_8   15

/*================================================================
	Описание выхода R/W RS-485
==================================================================*/
//-- Порты
#define PORT_RS485_RW GPIOA
//-- ID выводов
#define OUT_RS485_RW	8

/*================================================================
	Описание аналоговых входов
==================================================================*/
//-- Порты аналоговых входов
#define PORT_AIN1   GPIOA
#define PORT_AIN2   GPIOA

//-- ID выводов аналоговых входов
#define PIN_AIN1   0
#define PIN_AIN2   1

//-- ID датчиков - ID канала АЦП
#define PH1   ADC_Channel_0
#define PH2   ADC_Channel_1

/*================================================================
	Описание цифровых входов и кнопок
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
	Описание выходов управления реле для регулятора
==================================================================*/
//#define PORT_REL_PH_PLUS  		GPIOC
//#define PORT_REL_PH_MINUS 		GPIOC

//#define PIN_REL_PH_PLUS   		9
//#define PIN_REL_PH_MINUS  		8

/*================================================================
	Описание входов виртуальных концевиков
==================================================================*/

//#define PORT_CURRENT_PH_PLUS  	GPIOC
//#define PORT_CURRENT_PH_MINUS 	GPIOC

//#define PIN_CURRENT_PH_PLUS   	0
//#define PIN_CURRENT_PH_MINUS  	1

/*================================================================
	Описание выходов управления насосом
==================================================================*/
#define PORT_PUMP  			GPIOC
#define PIN_PUMP   			7

/*================================================================
	Описание выходов управления реле тревоги
==================================================================*/
#define PORT_KLAPAN  		GPIOC
#define PIN_KLAPAN   		6

/*================================================================
	Адреса в EEPROM для переменных
==================================================================*/
#define EEADR_SETUP_PH				100
#define EEADR_COEF_PROP				102
#define EEADR_COEF_INTEG			104
#define EEADR_COEF_DIFF				106
#define EEADR_MAX_OUT_OF_WATER_SEC	108
#define EEADR_MAX_TIME_ERROR_PH_SEC	110
#define EEADR_REG_CYCLETIME_SEC		112
#define EEADR_REG_LAST_REGPOS_VALUE	114	// последнее значение открытия регулятора в %

#define EEADR_TAR_PH1_P1_ADC		200

/*================================================================
	Режимы работы устройства
==================================================================*/
typedef enum _work_modes
{
	Mode_RegulatorPh = 1,
	Mode_Calibrating_PH1,
	Mode_Calibrating_PH2,
} EWorkMode;

#endif
