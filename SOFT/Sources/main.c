/***********************************************************************************
* Наименование : Проект "Acidifier"
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 09.03.2021
************************************************************************************/

//--- INCLUDES -------------------

#include "FreeRTOS.h"
#include "task.h"

#include "stdio.h"

#include "Leds.h"
#include "AddrSwitch.h"
#include "AnaInputs.h"
#include "Modbus.h"
#include "leds-dig.h"
#include "Regulator.h"
#include "Rs485.h"

// --- TYPES ---------------------

//--- CONSTANTS ------------------
const uint16_t UINT_VERSION = 102;

const uint16_t DEFAULT_SETUP_PH = 50;
const float MIN_VALUE_SETUP_PH = 20;
const float MAX_VALUE_SETUP_PH = 100;

//--- GLOBAL VARIABLES -----------
uint8_t g_DeviceAddr = 0;	// текущий адрес устройства на шине RS-485

EWorkMode g_WorkMode;				// текущий режим работы / отображения
bool g_isNoWater = false;
//bool g_isErrRegulator = false;
bool g_isErrTimeoutSetupPh = false;
bool g_isErrSensors = false;

float g_Sensor_PH;					// текущее значение PH с датчиков
float g_Setup_PH;					// заданное пользователем значение PH

bool g_isBtnPlusClick;
bool g_isBtnMinusClick;
bool g_isDblBtnPressed;
bool g_isEscPressed;
bool g_isEscClick;

int g_WaterCounter;

//--- IRQ ------------------------
/*
void EXTI15_10_IRQHandler()
{
	// Make sure that interrupt flag is set
	if( EXTI_GetITStatus(EXTI_Line11) != RESET ) 
	{
		// Clear interrupt flag
		EXTI_ClearITPendingBit(EXTI_Line11);

		g_WaterCounter++;
	}    
	
	EXTI_ClearFlag(EXTI_Line11);
	
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
}

void Init_EXTI(void)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	// Enable clock for AFIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	// Set pin as input
	// Вход скорости ветра
	GPIO_PinConfigure( PORT_SENS_WATER, PIN_SENS_WATER, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	
	// Add IRQ vector to NVIC
	// PB11 is connected to EXTI_Line11, which has EXTI4_IRQn vector
	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
	// Set priority
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	// Set sub priority
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	// Enable interrupt
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	// Add to NVIC
	NVIC_Init(&NVIC_InitStruct);

	// Tell system that you will use PB0 for EXTI_Line0
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);

	// PB11 is connected to EXTI_Line11
	EXTI_InitStruct.EXTI_Line = EXTI_Line11;
	// Enable interrupt
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	// Interrupt mode
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	// Triggers on rising and falling edge
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	// Add to EXTI
	EXTI_Init(&EXTI_InitStruct);

	NVIC_EnableIRQ(EXTI15_10_IRQn);	//Разрешаем прерывание в контроллере прерываний	
}
*/

//--- FUNCTIONS ------------------

void Thread_WORK( void *pvParameters );

/*******************************************************
Функция		: Вкл. / выкл. тревоги
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void switch_ALARM( uint8_t on )
{
	GPIO_PinWrite( PORT_RELAY_ALARM , PIN_RELAY_ALARM, on );
}

int ReadWorkMode( uint16_t idx )
{
	return g_WorkMode;
}

/*******************************************************
Функция		: Получения адреса устройства
Параметр 1	: нет
Возвр. знач.: Адрес устройства
********************************************************/
uint8_t GetDeviceAddress(void)
{
	return g_DeviceAddr;
}

/*******************************************************
Поток		: Мониторинг изменения адреса на шине RS-485
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void CheckAddrChange( void *pvParameters )
{
	for(;;)
	{
		uint16_t addr = ADRSW_GetAdr();
		if( addr != g_DeviceAddr )
		{
			vTaskDelay(2000);
			if( addr == ADRSW_GetAdr() )
			{
				// адрес устройства изменился, меняем
				g_DeviceAddr = addr;
				for( int i=0; i<addr; i++ )
				{
					LedSYS( 1 );
					vTaskDelay(200);
					LedSYS( 0 );
					vTaskDelay(200);
				}
			}
		}
	}
}

bool _isBtnPlusPressed( void )
{
	return (bool) (GPIO_PinRead( PORT_BTN_PLUS, PIN_BTN_PLUS )==0);
}
bool _isBtnMinusPressed( void )
{
	return (bool) (GPIO_PinRead( PORT_BTN_MINUS, PIN_BTN_MINUS )==0);
}
bool _isBtnEscPressed( void )
{
	return (bool) (GPIO_PinRead( PORT_BTN_ESC, PIN_BTN_ESC )==0);
}

/*******************************************************
Поток		: Рабочий поток устройства
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_Buttons( void *pvParameters )
{
	const int TIME_MS = 25; 
	const int TIME_MAXMS_BTNDOWN = 500; 
	const int TIME_DBLBTN = 1200; 

	bool isBtnPlusPressedPrev = false;
	bool isBtnMinusPressedPrev = false;
	bool isBtnEscPressedPrev = false;
	
	bool isBtnPlusPressedNow;
	bool isBtnMinusPressedNow;
	bool isBtnEscPressedNow;

	int msBtnPlusPressed = 0;
	int msBtnMinusPressed = 0;
	int msBtnEscPressed = 0;

	bool skipNextUpPlus = false;
	bool skipNextUpMinus = false;
	bool skipDblButtons = false;
	
	for(;;)
	{
		isBtnPlusPressedNow = _isBtnPlusPressed();
		isBtnMinusPressedNow = _isBtnMinusPressed();
		isBtnEscPressedNow = _isBtnEscPressed();
		
		if( !isBtnPlusPressedNow )
		{
			if( isBtnPlusPressedPrev ) 
			{
				if( !skipNextUpPlus )
				{
					if( msBtnPlusPressed < TIME_MAXMS_BTNDOWN )
					g_isBtnPlusClick = true;
				}
				else
				{
					// при отпускании кнопки разблокируем анализ нажатия двух кнопок
					skipDblButtons = false;
				}
				skipNextUpPlus = false;
			}
			msBtnPlusPressed = 0;
		}
		else
		{
			if( isBtnPlusPressedPrev ) 
			{
				msBtnPlusPressed += TIME_MS;
				if(( msBtnPlusPressed > TIME_DBLBTN ) && ( msBtnMinusPressed > TIME_DBLBTN ) && !skipDblButtons )
				{
					g_isDblBtnPressed = true;
					skipNextUpPlus = true;
					skipNextUpMinus = true;
					skipDblButtons = true;
				}
			}
		}
		
		if( !isBtnMinusPressedNow )
		{
			if( isBtnMinusPressedPrev ) 
			{
				if( !skipNextUpMinus )
				{
					if( msBtnMinusPressed < TIME_MAXMS_BTNDOWN )
					g_isBtnMinusClick = true;
				}
				else
				{
					// при отпускании кнопки разблокируем анализ нажатия двух кнопок
					skipDblButtons = false;
				}
				skipNextUpMinus = false;
			}
			msBtnMinusPressed = 0;
		}
		else
		{
			if( isBtnMinusPressedPrev ) 
			{
				msBtnMinusPressed += TIME_MS;
				if(( msBtnPlusPressed > TIME_DBLBTN ) && ( msBtnMinusPressed > TIME_DBLBTN ) && !skipDblButtons )
				{
					g_isDblBtnPressed = true;
					skipNextUpPlus = true;
					skipNextUpMinus = true;
					skipDblButtons = true;
				}
			}
		}
			
		if( !isBtnEscPressedNow )
		{
			msBtnEscPressed = 0;
			if( isBtnEscPressedPrev )
			{
				g_isEscClick = true;
			}
		}
		else
		{
			if( !g_isEscPressed && isBtnEscPressedPrev ) 
			{
				msBtnEscPressed += TIME_MS;
				if( msBtnEscPressed > TIME_DBLBTN )
				{
					g_isEscPressed = true;
					msBtnEscPressed = 0;
				}
			}
			else
			{
				msBtnEscPressed = 0;
			}
		}
		
		isBtnMinusPressedPrev = isBtnMinusPressedNow;
		isBtnPlusPressedPrev = isBtnPlusPressedNow;
		isBtnEscPressedPrev = isBtnEscPressedNow;
		
		vTaskDelay(TIME_MS);
	}
}

void Buttons_init( void )
{
	GPIO_PinConfigure( PORT_BTN_PLUS, PIN_BTN_PLUS, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	GPIO_PinConfigure( PORT_BTN_MINUS, PIN_BTN_MINUS, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	GPIO_PinConfigure( PORT_BTN_ESC, PIN_BTN_ESC, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
}

/*******************************************************
Функция		: Инициализация приложения
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Initialize()
{
	Leds_init();
	ADRSW_init();
	Buttons_init();
	
	g_DeviceAddr = 0;
}

/*******************************************************
Поток		: Рабочий поток устройства
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
/*
void Thread_RegulatorCalibrate( void *pvParameters )
{
	const uint16_t STEP_MS = 300;
	uint8_t stepNum = 0;
	float litres_by_minute;
	char szLog[50];
	
	Rs485_Init();
	
	vTaskDelay(5000);
	Reg_Init();
	
	Init_EXTI();
	
	set_StartCalibrateState( &stepNum );
	
	for(;;)
	{
		vTaskDelay( 100 );
		if( _isBtnPlusPressed() )
		{
			stepNum++;
			LcdDig_PrintUInt( stepNum, SideLEFT, true );
			LcdDig_PrintPH( -1, SideRIGHT, false );
			
			Reg_RelayOn( REL_PH_MINUS );
			vTaskDelay( STEP_MS );
			Reg_RelayAllOff();
			
			g_WaterCounter = 0;
			
			vTaskDelay( 3000 );
			
			litres_by_minute = g_WaterCounter * 20;
			litres_by_minute /= 530.0;
			
			LcdDig_PrintUInt( stepNum, SideLEFT, false );
			sprintf( szLog, "%u\t%.2f\r\n", stepNum, litres_by_minute );
			RS485_SendString( szLog );
			
			if( litres_by_minute < 10 )
			{
				litres_by_minute = (roundf( litres_by_minute * 10.0 )) / 10.0;
				LcdDig_PrintPH( litres_by_minute, SideRIGHT, false );
			}
			else
				LcdDig_PrintUInt( (uint8_t) litres_by_minute, SideRIGHT, false );
		}
		if( _isBtnMinusPressed() )
		{
			set_StartCalibrateState( &stepNum );
		}
		
	}
}
*/

/*******************************************************
Функция		: Точка старта приложения
Параметр 1	: нет
Возвр. знач.: бесконечный цикл
********************************************************/
int main(void)
{
	Initialize();
	
	xTaskCreate( CheckAddrChange, (const char*)"ADDRESS", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTaskCreate( AInp_Thread, (const char*)"ANALOG", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTaskCreate( MBUS_Thread, (const char*)"Modbus", 0x1000,	( void * ) NULL, ( tskIDLE_PRIORITY + 2 ), NULL);

	xTaskCreate( Thread_Leds_Dig, (const char*)"LedsDig", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	
	xTaskCreate( Thread_Buttons, (const char*)"BTN", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTaskCreate( Thread_WORK, (const char*)"WORK", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTaskCreate( Thread_Regulator, (const char*)"Regulator", 512,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	
	/* Start the scheduler. */
	vTaskStartScheduler();

}

/*******************************************************
	Запись нового значения PH в EEPROM
********************************************************/
bool WriteSetupPhValue( uint16_t idx, uint16_t val )
{
	if(( val < MIN_VALUE_SETUP_PH ) || ( val > MAX_VALUE_SETUP_PH ))
		return false;
	
	if( FM24_WriteBytes( EEADR_SETUP_PH, (uint8_t*) &val, sizeof( uint16_t ) ) )
	{
		// запоминаем в глобальной переменной
		g_Setup_PH = val;
		g_Setup_PH /= 10.0;
		
		return true;
	}
	return false;

}
/*******************************************************
	Чтение значения задания PH из EEPROM
********************************************************/
int ReadSetupPhValue( uint16_t idx )
{
	uint16_t uiValue;
	
	if( !FM24_ReadBytes( EEADR_SETUP_PH, (uint8_t*) &uiValue, sizeof( uint16_t ) ) )
		return -1;
	
	if(( uiValue < MIN_VALUE_SETUP_PH ) || ( uiValue > MAX_VALUE_SETUP_PH ))
	{
		// Значение установленного PH лежит вне рабочего диапазона
		// устанавливаем значение по умолчанию
		uiValue = DEFAULT_SETUP_PH;
		WriteSetupPhValue( 0, uiValue );
	}

	g_Setup_PH = uiValue;
	g_Setup_PH /= 10.0;
	return uiValue;
}

/*******************************************************
	Установка нового значения PH
********************************************************/
bool SetupPhValue( float value )
{
	// тут надо применить новое значение заданного PH
	value *= 10.0;
	uint16_t uiValue  = roundf( value );
	return WriteSetupPhValue( 0, uiValue );
}

// сброс срабатываний всех кнопок и сочетаний
void clearAllButtons( void )
{
	g_isDblBtnPressed = false;
	g_isBtnPlusClick = false;
	g_isBtnMinusClick = false;
	g_isEscPressed = false;
	g_isEscClick = false;
}

void display_clearErrors( void )
{
	const uint8_t DISP_TIME_MS = 50;
	
	TLedDig dig;
	dig.isBlinking = false;
	dig.isPoint = false;

	for( int j=0; j<3; j++ )
	{
		
		for( int i=0; i<4; i++ )
		{
			int idx = i;
			dig.isOn = true;
			dig.value = line_top;
			LcdDig_SetDigit( i, &dig );
			dig.isOn = false;
			if( ++idx > 3 ) idx -= 4;
			LcdDig_SetDigit( idx, &dig );
			if( ++idx > 3 ) idx -= 4;
			LcdDig_SetDigit( idx, &dig );
			if( ++idx > 3 ) idx -= 4;
			LcdDig_SetDigit( idx, &dig );
			LcdDig_refresh( false );
			vTaskDelay( DISP_TIME_MS );
		}
		
		dig.isOn = true;
		dig.value = line_right_top;
		LcdDig_SetDigit( 3, &dig );
		dig.isOn = false;
		LcdDig_SetDigit( 0, &dig );
		LcdDig_SetDigit( 1, &dig );
		LcdDig_SetDigit( 2, &dig );
		LcdDig_refresh( false );
		vTaskDelay( DISP_TIME_MS );
		
		dig.isOn = true;
		dig.value = line_right_bot;
		LcdDig_SetDigit( 3, &dig );
		dig.isOn = false;
		LcdDig_SetDigit( 0, &dig );
		LcdDig_SetDigit( 1, &dig );
		LcdDig_SetDigit( 2, &dig );
		LcdDig_refresh( false );
		vTaskDelay( DISP_TIME_MS );
		
		for( int i=3; i>=0; i-- )
		{
			int idx = i;
			dig.isOn = true;
			dig.value = line_bot;
			LcdDig_SetDigit( i, &dig );
			dig.isOn = false;
			if( ++idx > 3 ) idx -= 4;
			LcdDig_SetDigit( idx, &dig );
			if( ++idx > 3 ) idx -= 4;
			LcdDig_SetDigit( idx, &dig );
			if( ++idx > 3 ) idx -= 4;
			LcdDig_SetDigit( idx, &dig );
			LcdDig_refresh( false );
			vTaskDelay( DISP_TIME_MS );
		}

		dig.isOn = true;
		dig.value = line_left_bot;
		LcdDig_SetDigit( 0, &dig );
		dig.isOn = false;
		LcdDig_SetDigit( 1, &dig );
		LcdDig_SetDigit( 2, &dig );
		LcdDig_SetDigit( 3, &dig );
		LcdDig_refresh( false );
		vTaskDelay( DISP_TIME_MS );

		dig.isOn = true;
		dig.value = line_left_top;
		LcdDig_SetDigit( 0, &dig );
		dig.isOn = false;
		LcdDig_SetDigit( 1, &dig );
		LcdDig_SetDigit( 2, &dig );
		LcdDig_SetDigit( 3, &dig );
		LcdDig_refresh( false );
		vTaskDelay( DISP_TIME_MS );
	}
	LcdDig_DispOff();
}

// Сброс всех ошибок
void clearAllErrors( void )
{
	//g_isErrRegulator = false;
	g_isErrTimeoutSetupPh = false;
	g_isErrSensors = false;

	// Отображение сброса ошибок
	display_clearErrors();
}

/*******************************************************
Поток		: Рабочий поток устройства
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_WORK( void *pvParameters )
{
	float ph1, ph2;
	bool stopWork, IsPhSensorsTooDiff;
	
	// Инициализация выхода тревоги
	GPIO_PinConfigure( PORT_RELAY_ALARM, PIN_RELAY_ALARM, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );

	ReadSetupPhValue(0);
	
	g_WorkMode = Mode_RegulatorPh;
	g_isNoWater = true;
	
//	for( int i=0; i<20; i++ )
//	{
//		LedSYS( 1 );
//		vTaskDelay(100);
//		LedSYS( 0 );
//		vTaskDelay(100);
//	}
	
	for(;;)
	{
		vTaskDelay(50);
		
		//setupMoveLeds();
		
		stopWork = g_isErrSensors || g_isErrTimeoutSetupPh || g_isNoWater;
		
		switch( (int)g_WorkMode )
		{
			case Mode_RegulatorPh:

				switch_ALARM( g_isErrSensors || g_isErrTimeoutSetupPh );

				Led_OnOff( LED_TAR_P1, 0 );
				Led_OnOff( LED_TAR_P2, 0 );

				g_Sensor_PH = AInp_GetSystemPh( &IsPhSensorsTooDiff );

				if( g_isNoWater )
					LcdDig_PrintPH( -1, SideLEFT, false );
				else
					LcdDig_PrintPH( g_Sensor_PH, SideLEFT, false );

				LcdDig_PrintPH( g_Setup_PH, SideRIGHT, false );
			
				if( stopWork )
				{
					LcdDig_DispBlinkOn( SideLEFT );
					LcdDig_DispBlinkOff( SideRIGHT );
				}
				else
				{
					LcdDig_DispBlinkOff( SideLEFT );
					LcdDig_DispBlinkOff( SideRIGHT );
				}
				if( g_isDblBtnPressed )
				{
					// сброс нажатых кнопок
					clearAllButtons();
					g_WorkMode = Mode_Calibrating_PH1;
				}
				else if( g_isBtnPlusClick )
				{
					// сброс нажатых кнопок
					clearAllButtons();
					
					if( g_Setup_PH + 0.1 < 10 )
						SetupPhValue( g_Setup_PH + 0.1 );
				}
				else if( g_isBtnMinusClick )
				{
					// сброс нажатых кнопок
					clearAllButtons();
					
					if( g_Setup_PH - 0.1 > 0 )
						SetupPhValue( g_Setup_PH - 0.1 );
				}
				else if( g_isEscPressed )
				{
					// сброс нажатых кнопок
					clearAllButtons();

					clearAllErrors();
					
					g_WorkMode = Mode_RegulatorPh;
					Reg_RestartWaterTimer();
				}
				break;
			
			case Mode_Calibrating_PH1:
				
				switch_ALARM(0);
				
				Led_OnOff( LED_TAR_P1, 1 );
				Led_OnOff( LED_TAR_P2, 0 );

				ph1 = AInp_GetFloatSensorPh(0);
				ph2 = AInp_GetFloatSensorPh(1);
				LcdDig_PrintPH( ph1, SideLEFT, false );
				LcdDig_PrintPH( ph2, SideRIGHT, false );
			
				if( g_isDblBtnPressed )
				{
					// Подтверждение калибровки точки

					// сброс нажатых кнопок
					clearAllButtons();
					
					// запись калибровки
					AInp_WriteAdcTar1( 0, AInp_ReadAdcValue( 0 ) );
					AInp_WriteAdcTar1( 1, AInp_ReadAdcValue( 1 ) );

					// Вывод на дисплей обновленных значений датчиков
					ph1 = AInp_GetFloatSensorPh(0);
					ph2 = AInp_GetFloatSensorPh(1);
					LcdDig_PrintPH( ph1, SideLEFT, false );
					LcdDig_PrintPH( ph2, SideRIGHT, false );
					vTaskDelay(100);

					// Отображение подтверждения
					LcdDig_DispBlinkOn( SideLEFT | SideRIGHT );
					vTaskDelay( 1500 );
					LcdDig_DispBlinkOff( SideLEFT | SideRIGHT );
					
					g_WorkMode = Mode_Calibrating_PH2;
				}
				else if( g_isBtnPlusClick )
				{
					// Пропуск калибровки точки

					// сброс нажатых кнопок
					clearAllButtons();
					g_WorkMode = Mode_Calibrating_PH2;
				}
				else if( g_isEscClick )
				{
					// Отмена калибровки точки

					// сброс нажатых кнопок
					clearAllButtons();
					
					g_WorkMode = Mode_RegulatorPh;
					Reg_RestartWaterTimer();
				}
			
				break;
				
			case Mode_Calibrating_PH2:
				
				switch_ALARM(0);

				Led_OnOff( LED_TAR_P1, 0 );
				Led_OnOff( LED_TAR_P2, 1 );

				ph1 = AInp_GetFloatSensorPh(0);
				ph2 = AInp_GetFloatSensorPh(1);
				LcdDig_PrintPH( ph1, SideLEFT, false );
				LcdDig_PrintPH( ph2, SideRIGHT, false );
			
				if( g_isDblBtnPressed )
				{
					// Подтверждение калибровки точки

					// сброс нажатых кнопок
					clearAllButtons();
					
					// запись калибровки
					AInp_WriteAdcTar2( 0, AInp_ReadAdcValue( 0 ) );
					AInp_WriteAdcTar2( 1, AInp_ReadAdcValue( 1 ) );

					// Вывод на дисплей обновленных значений датчиков
					ph1 = AInp_GetFloatSensorPh(0);
					ph2 = AInp_GetFloatSensorPh(1);
					LcdDig_PrintPH( ph1, SideLEFT, false );
					LcdDig_PrintPH( ph2, SideRIGHT, false );
					vTaskDelay(100);

					// Отображение подтверждения
					LcdDig_DispBlinkOn( SideLEFT | SideRIGHT );
					vTaskDelay( 1500 );
					LcdDig_DispBlinkOff( SideLEFT | SideRIGHT );
					
					g_WorkMode = Mode_RegulatorPh;
					Reg_RestartWaterTimer();
				}
				else if( g_isEscClick )
				{
					// Отмена калибровки точки
					// сброс нажатых кнопок
					clearAllButtons();
					
					g_WorkMode = Mode_RegulatorPh;
					Reg_RestartWaterTimer();
				}
				
				break;
			
			default:
				g_WorkMode = Mode_RegulatorPh;
				Reg_RestartWaterTimer();
				break;
		}
	}
}
