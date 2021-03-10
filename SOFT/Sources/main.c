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

#include "Leds.h"
#include "AddrSwitch.h"
#include "AnaInputs.h"
#include "Modbus.h"
#include "leds-dig.h"
#include "Regulator.h"

// --- TYPES ---------------------

//--- CONSTANTS ------------------

//--- GLOBAL VARIABLES -----------
static uint8_t g_DeviceAddr = 0;	// текущий адрес устройства на шине RS-485
uint16_t g_Status;					// текущий статус устройства (0-была перезагрузка)
EWorkMode g_WorkMode;				// текущий режим работы / отображения
float g_PH;							// текущее значение PH с датчиков

//--- IRQ ------------------------

//--- FUNCTIONS ------------------
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
			}
		}
	}
}

/*******************************************************
	Возвращает состояние датчика протока воды
********************************************************/
bool IsWaterOk( void )
{
	bool isOk;
	isOk = ( GPIO_PinRead( PORT_SENS_WATER, PIN_SENS_WATER ) == 0 );
	
	isOk ? Led_On( LED_OK ) : Led_Off( LED_OK );
	isOk ? Led_Off( LED_ERR_WATER ) : Led_On( LED_ERR_WATER );
	
	return isOk;
}

/*******************************************************
	Функция	переключает режим работы
********************************************************/
void SwitchWorkMode( EWorkMode newWorkMode )
{
	g_WorkMode = newWorkMode;
	
	Leds_OffAll();
	
	switch( g_WorkMode )
	{
		case Mode_RegulatorPh:
			break;
			
		case Mode_DisplaySensorsPh:
			Led_On( LED_VIEW );
			break;
		
		case Mode_Calibrating:
			Led_On( LED_TAR_P1 );
			break;
	}
}

/*******************************************************
Поток		: Рабочий поток устройства
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_WORK( void *pvParameters )
{
	g_Status = 0;
	
	vTaskDelay(1000);

	SwitchWorkMode( Mode_RegulatorPh );
	
	for(;;)
	{
		vTaskDelay(50);
		// обновляем значение PH с датчиков
		g_PH = AInp_GetSystemPh();
	}
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
	
	g_DeviceAddr = 0;
}

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

	xTaskCreate( MBUS_Thread, (const char*)"Modbus", 2048,	( void * ) NULL, ( tskIDLE_PRIORITY + 2 ), NULL);

	xTaskCreate( Thread_Leds_Dig, (const char*)"LedsDig", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	
	xTaskCreate( Thread_Regulator, (const char*)"Regulator", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTaskCreate( Thread_WORK, (const char*)"WORK", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	/* Start the scheduler. */
	vTaskStartScheduler();

}
