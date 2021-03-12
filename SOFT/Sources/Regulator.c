/***********************************************************************************
* Наименование модуля	: Управление регулятором PH
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 10.03.2021
************************************************************************************/

//--- INCLUDES -------------------
#include "Regulator.h"
#include "Leds.h"
#include "AnaInputs.h"
#include "leds-dig.h"

//--- CONSTANTS ------------------
const TRelDesc Relay[] = {
	{PORT_REL_PLUS, PIN_REL_PLUS},
	{PORT_REL_MINUS, PIN_REL_MINUS},
};

//--- GLOBAL VARIABLES -----------

//--- FUNCTIONS ------------------
extern int ReadWorkMode( uint16_t idx );
extern float g_Sensor_PH;					// текущее значение PH с датчиков
extern float g_Setup_PH;					// заданное пользователем значение PH

/*******************************************************
Функция		: Инициализация железа
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Reg_Init(void)
{
	for( int i=0; i<2; i++ )
	{
		// Перевод вывода для светодиода на выход с открытым коллектором
		GPIO_PinConfigure( Relay[i].GPIOx, Relay[i].pin, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
		
		// Отключение светодиода
		GPIO_PinWrite( Relay[i].GPIOx, Relay[i].pin, REL_OFF );
	}
}

/*******************************************************
	Возвращает состояние датчика протока воды
********************************************************/
bool IsWaterOk( void )
{
	bool isOk;
	isOk = ( GPIO_PinRead( PORT_SENS_WATER, PIN_SENS_WATER ) == 0 );
	
	return isOk;
}

float getPidValue( float errorPh, float deltaTime, float prevPh )
{
    float pidValue = 0, integralValue, diffValue;
	static float prevIntegralValue = 0;
	const float K_INTEGRAL = 0.1;
	const float K_DIFF = 0.05;
	const float K_PROP = 2.0;

    integralValue = prevIntegralValue + (errorPh * deltaTime);
    prevIntegralValue = integralValue;
    integralValue *= K_INTEGRAL;

    diffValue = K_DIFF * (( g_Sensor_PH - prevPh ) / deltaTime);

    pidValue = K_PROP * (errorPh + integralValue + diffValue);

    return pidValue;
}


/*******************************************************
	Один цикл регулирования PH
********************************************************/
void regulator_cycle( float deltaTime )
{
	static float prevPh = 8;
	
	float errorPh, pidValue;
	
	errorPh = g_Sensor_PH - g_Setup_PH;
	
//	if(( errorPh < 0.1 ) || ( errorPh > -0.1 ))
//	{
//		// здесь мы в пределах допуска, останавливаем регулятор
//		Reg_RelayAllOff();
//		return;
//	}
	
	pidValue = getPidValue( errorPh, deltaTime, prevPh );
	
	prevPh = g_Sensor_PH;
	
	if( pidValue > 0.1 )
	{
		Reg_RelayOn( REL_PLUS );
		Reg_RelayOff( REL_MINUS );
	}
	else if( pidValue < -0.1 )
	{
		Reg_RelayOff( REL_PLUS );
		Reg_RelayOn( REL_MINUS );
	}
	else
	{
		Reg_RelayAllOff();
	}
}

/*******************************************************
Поток		: Рабочий поток устройства
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_Regulator( void *pvParameters )
{
	Reg_Init();

	const TickType_t CYCLETIME_MS = 500;			// интервал между циклами регулирования
	const TickType_t MAX_OUT_OF_WATER_MS = 2000;	// макс. длительность отсуствия воды для аварии
	const TickType_t MAX_ERROR_PH_MS = 90000;		// макс. длительность ошибки установки PH
	
	
	TickType_t xLastWakeTime;
	int timeOutOfWater = 0;
	int timeOutErrorPhValue = 0;
	bool isAlarmOutOfWater = false;
	bool isAlarmSensPh = false;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, CYCLETIME_MS );
		
		if( ReadWorkMode(0) != Mode_RegulatorPh )
		{
			Led_Off( LED_ERR_WATER );
			Led_Off( LED_OK );

			isAlarmOutOfWater = false;
			isAlarmSensPh = false;
			timeOutOfWater = 0;
			timeOutErrorPhValue = 0;

			continue;
		}

		// проверяем наличие воды
		if( IsWaterOk() )
		{
			Led_Off( LED_ERR_WATER );
			isAlarmOutOfWater = false;
		}
		else
		{
			timeOutOfWater += CYCLETIME_MS;
			if( timeOutOfWater > MAX_OUT_OF_WATER_MS )
			{
				isAlarmOutOfWater = true;
			}
		}
		// проверяем ошибки по датчикам PH и выводим на дисплей значения PH
		g_Sensor_PH = AInp_GetSystemPh();
		if( g_Sensor_PH < 0 )
		{
			isAlarmSensPh = true;
		}
		else
		{
			isAlarmSensPh = false;
		}
		LcdDig_PrintPH( g_Sensor_PH, SideLEFT );
		LcdDig_PrintPH( g_Setup_PH, SideRIGHT );
		
		if( !isAlarmOutOfWater && !isAlarmSensPh )
		{
			Led_On( LED_OK );
			Led_Off( LED_ERR_WATER );
			Led_Off( LED_ERR_SENS_PH );
			
			regulator_cycle( (float)CYCLETIME_MS / 1000.0 );
			
			if( fabs( g_Sensor_PH - g_Setup_PH ) > 0.5 )
			{
				timeOutErrorPhValue += CYCLETIME_MS;
				if( timeOutErrorPhValue > MAX_ERROR_PH_MS )
				{
					timeOutErrorPhValue = 0;
					Led_On( LED_ERR_REGPH );
				}
			}
			else 
			{
				timeOutErrorPhValue = 0;
				Led_Off( LED_ERR_REGPH );
			}
			
		}
		else 
		{
			Led_Off( LED_OK );
			if( isAlarmSensPh )
				Led_On( LED_ERR_SENS_PH );
			if( isAlarmOutOfWater )
			{
				Led_On( LED_ERR_WATER );
				// здесь надо закрыть кран с кислотой как то...
				// ... доделать !
			}
		}
	}
}


/*******************************************************
Функция		: Включает реле по его индексу
Параметр 1	: индекс реле
Возвр. знач.: нет
********************************************************/
void Reg_RelayOn(uint8_t indx)
{
	if( indx > 1 )
		return;
	
	GPIO_PinWrite( Relay[indx].GPIOx, Relay[indx].pin, REL_ON );
}

/*******************************************************
Функция		: Отключает реле по его индексу
Параметр 1	: индекс реле
Возвр. знач.: нет
********************************************************/
void Reg_RelayOff(uint8_t indx)
{
	if( indx > 1 )
		return;
	
	GPIO_PinWrite( Relay[indx].GPIOx, Relay[indx].pin, REL_OFF );
}

/*******************************************************
Функция		: Отключает оба реле
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Reg_RelayAllOff(void)
{
	GPIO_PinWrite( Relay[0].GPIOx, Relay[0].pin, REL_OFF );
	GPIO_PinWrite( Relay[1].GPIOx, Relay[1].pin, REL_OFF );
}

