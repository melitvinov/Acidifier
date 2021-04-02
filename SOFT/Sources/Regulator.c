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
	{PORT_REL_PH_MINUS, PIN_REL_PH_MINUS},
	{PORT_REL_PH_PLUS, PIN_REL_PH_PLUS},
};

const float K_INTEGRAL_DEFAULT = 0.2;
const float K_DIFF_DEFAULT = 0.2;
const float K_PROP_DEFAULT = 2.0;
const uint16_t FULL_MOVE_TIME_SEC_DEFAULT = 20;
const uint16_t MIN_FULL_MOVE_TIME_SEC = 5;		// минимальное время хода регулятора в секундах
const uint16_t MAX_FULL_MOVE_TIME_SEC = 120;	// максимальное время хода регулятора в секундах

//--- GLOBAL VARIABLES -----------
extern float g_Sensor_PH;					// текущее значение PH с датчиков
extern float g_Setup_PH;					// заданное пользователем значение PH

extern bool g_isNoWater;
extern bool g_isErrRegulator;
extern bool g_isErrTimeoutSetupPh;
extern bool g_isErrSensors;

float g_K_INTEGRAL;
float g_K_DIFF;
float g_K_PROP;

uint16_t MAX_OUT_OF_WATER_SEC;		// макс. длительность отсуствия воды для аварии (сек.)
uint16_t MAX_TIME_ERROR_PH_SEC;		// макс. длительность ошибки установки PH (сек.)
uint16_t FULL_MOVE_TIME_SEC;			// полное время хода регулятора в секундах

//--- FUNCTIONS ------------------
extern int ReadWorkMode( uint16_t idx );
void Reg_RelayOn(uint8_t indx);
void Reg_RelayOff(uint8_t indx);
void Reg_RelayAllOff(void);
bool Reg_ToOpen( void );
bool Reg_IsError( void );

/*******************************************************
Функция		: Инициализация железа
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Reg_Init(void)
{
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE );
	
	for( int i=0; i<2; i++ )
	{
		// Перевод вывода для светодиода на выход с открытым коллектором
		GPIO_PinConfigure( Relay[i].GPIOx, Relay[i].pin, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
		
		// Отключение светодиода
		GPIO_PinWrite( Relay[i].GPIOx, Relay[i].pin, REL_OFF );
	}

	// Перевод вывода на вход с подтяжкой к VCC для токовых датчиков
	GPIO_PinConfigure( PORT_CURRENT_PH_MINUS, PIN_CURRENT_PH_MINUS, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	GPIO_PinConfigure( PORT_CURRENT_PH_PLUS, PIN_CURRENT_PH_PLUS, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );

	// Перевод вывода на вход с подтяжкой к VCC для датчика воды
	GPIO_PinConfigure( PORT_SENS_WATER, PIN_SENS_WATER, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	
	// Перевод выводов реле насоса и тревоги на выход
	GPIO_PinConfigure( PORT_PUMP, PIN_PUMP, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
	// Отключение насоса и тревоги
	GPIO_PinWrite( PORT_PUMP, PIN_PUMP, 0 );
	
	// Загружаем коэффициенты для расчета
	uint16_t Koef[3];
	bool needSetupDef = false;
	
	if( !FM24_ReadWords( EEADR_COEF_PROP, &(Koef[0]), 3 ) )
		needSetupDef = true;
	else if( Koef[0] == 0 || Koef[1] == 0 )
		needSetupDef = true;

	if( needSetupDef )
	{
		g_K_PROP = K_PROP_DEFAULT;
		g_K_INTEGRAL = K_INTEGRAL_DEFAULT;
		g_K_DIFF = K_DIFF_DEFAULT;
		Koef[0] = g_K_PROP * 1000;
		Koef[1] = g_K_INTEGRAL * 1000;
		Koef[2] = g_K_DIFF * 1000;
		
		FM24_WriteWords( EEADR_COEF_PROP, &(Koef[0]), 3 );
	}
	else
	{
		g_K_PROP = (float)Koef[0] / 1000.0;
		g_K_INTEGRAL = (float)Koef[1] / 1000.0;
		g_K_DIFF = (float)Koef[2] / 1000.0;
		
	}

	if( !FM24_ReadWords( EEADR_MAX_OUT_OF_WATER_SEC, &MAX_OUT_OF_WATER_SEC, 1 ) || MAX_OUT_OF_WATER_SEC == 0 )
	{
		MAX_OUT_OF_WATER_SEC = 15;
		FM24_WriteWords( EEADR_MAX_OUT_OF_WATER_SEC, &MAX_OUT_OF_WATER_SEC, 1 );
	}

	if( !FM24_ReadWords( EEADR_MAX_TIME_ERROR_PH_SEC, &MAX_TIME_ERROR_PH_SEC, 1 ) || MAX_TIME_ERROR_PH_SEC == 0 )
	{
		MAX_TIME_ERROR_PH_SEC = 90;
		FM24_WriteWords( EEADR_MAX_TIME_ERROR_PH_SEC, &MAX_TIME_ERROR_PH_SEC, 1 );
	}

	if( !FM24_ReadWords( EEADR_FULL_MOVE_TIME_SEC, &FULL_MOVE_TIME_SEC, 1 ) || FULL_MOVE_TIME_SEC == 0 )
	{
		FULL_MOVE_TIME_SEC = FULL_MOVE_TIME_SEC_DEFAULT;
		FM24_WriteWords( EEADR_FULL_MOVE_TIME_SEC, &FULL_MOVE_TIME_SEC, 1 );
	}
}

/*******************************************************
	Расчет значения PID регулятора
********************************************************/
float getPidValue( float errorPh, float deltaTime, float prevPh )
{
    float pidValue = 0, integralValue, diffValue;
	static float prevIntegralValue = 0;

    integralValue = prevIntegralValue + (errorPh * deltaTime);
    prevIntegralValue = integralValue;
    integralValue *= g_K_INTEGRAL;

    diffValue = g_K_DIFF * (( g_Sensor_PH - prevPh ) / deltaTime);

    pidValue = g_K_PROP * (errorPh + integralValue + diffValue);

    return pidValue;
}

/*******************************************************
	Возвращает состояние концевика реле закрытия
********************************************************/
bool IsCurrent_PH_MINUS( void )
{
	bool isOk;
	isOk = ( GPIO_PinRead( PORT_CURRENT_PH_MINUS, PIN_CURRENT_PH_MINUS ) == 0 );
	
	return isOk;
}

/*******************************************************
	Возвращает состояние концевика реле закрытия
********************************************************/
bool IsCurrent_PH_PLUS( void )
{
	bool isOk;
	isOk = ( GPIO_PinRead( PORT_CURRENT_PH_PLUS, PIN_CURRENT_PH_PLUS ) == 0 );
	
	return isOk;
}

/*******************************************************
	Один цикл регулирования PH
********************************************************/
void regulator_cycle( float deltaTime )
{
	static bool isFirstPid = true;
	static float prevPh;
	float errorPh, pidValue;
	
	errorPh = g_Setup_PH - g_Sensor_PH;

	if( isFirstPid )
	{
		isFirstPid = false;
		prevPh = g_Sensor_PH;
	}
	
	pidValue = getPidValue( errorPh, deltaTime, prevPh );
	
	prevPh = g_Sensor_PH;
	

	if( ( errorPh < 0.1 && errorPh > -0.1 ) && 
		( pidValue < 0.2 && pidValue > -0.2 ))
	{
		// здесь мы в пределах допуска, останавливаем регулятор
		Reg_RelayAllOff();
		return;
	}
	
	if( pidValue < -0.15 )
	{
		Reg_RelayOn( REL_PH_MINUS );
		Reg_RelayOff( REL_PH_PLUS );
	}
	else if( pidValue > 0.15 )
	{
		Reg_RelayOff( REL_PH_MINUS );
		Reg_RelayOn( REL_PH_PLUS );
	}
	else
	{
		Reg_RelayAllOff();
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

void switchPUMP( uint8_t on )
{
	GPIO_PinWrite( PORT_PUMP, PIN_PUMP, on );
}

/*******************************************************
Поток		: Рабочий поток устройства
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_Regulator( void *pvParameters )
{
	Reg_Init();

	const TickType_t CYCLETIME_MS = 200;			// интервал между циклами регулирования
	
	TickType_t xLastWakeTime;
	int timeOutOfWater = 0;
	int timeOutErrorPhValue = 0;
	int timeOutErrorPhSensors = 0;
	bool prevIsRegError = false;
	bool IsPhSensorsTooDiff;

	vTaskDelay( 1000 );
	
	g_isErrRegulator = !Reg_ToOpen();
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, CYCLETIME_MS );
		
		if( prevIsRegError && !g_isErrRegulator )
		{
			// прошел сброс ошибок, надо проверить регулятор
			g_isErrRegulator = Reg_IsError();
		}
		prevIsRegError = g_isErrRegulator;
		
		if( ReadWorkMode(0) != Mode_RegulatorPh )
		{
			// ставим максимальный PH (открываем регулятор полностью), выключаем насос
			switchPUMP( 0 );
			Reg_RelayOff( REL_PH_MINUS );
			Reg_RelayOn( REL_PH_PLUS );
			timeOutErrorPhSensors = 0;
			timeOutErrorPhValue = 0;
			timeOutOfWater = 0;
			continue;
		}
		
		// проверяем наличие воды
		if( IsWaterOk() )
		{
			g_isNoWater = false;
			timeOutOfWater = 0;
			switchPUMP( 1 );
		}
		else if( !g_isNoWater )
		{
			timeOutOfWater += CYCLETIME_MS;
			if( timeOutOfWater > (MAX_OUT_OF_WATER_SEC * 1000) )
			{
				g_isNoWater = true;
				switchPUMP( 0 );
				
				if( !g_isErrRegulator )
				{
					Reg_RelayOn( REL_PH_PLUS );
					vTaskDelay( 200 );
					if( IsCurrent_PH_PLUS() )
						g_isErrRegulator = !Reg_ToOpen();
					Reg_RelayOff( REL_PH_PLUS );
				}
			}
		}
		
		// проверяем ошибки по датчикам PH и выводим на дисплей значения PH
		g_Sensor_PH = AInp_GetSystemPh( &IsPhSensorsTooDiff );
		if( g_Sensor_PH < 0 || IsPhSensorsTooDiff )
		{
			if( !g_isErrSensors )
			{
				timeOutErrorPhSensors  += CYCLETIME_MS;
				if( timeOutErrorPhSensors > (MAX_OUT_OF_WATER_SEC * 1000) )
				{
					g_isErrSensors = true;
					timeOutErrorPhSensors = 0;
				}
			}
		}
		else
		{
			timeOutErrorPhSensors = 0;
		}
		
		if( !g_isNoWater && !g_isErrSensors && !g_isErrTimeoutSetupPh && !g_isErrRegulator )
		{
			// при наличии воды и отсутсвии ошибок - регулируем
			regulator_cycle( (float)CYCLETIME_MS / 1000.0 );
			
			if( fabs( g_Sensor_PH - g_Setup_PH ) > 0.5 )
			{
				timeOutErrorPhValue += CYCLETIME_MS;
				if( timeOutErrorPhValue > MAX_TIME_ERROR_PH_SEC * 1000 )
				{
					timeOutErrorPhValue = 0;
					g_isErrTimeoutSetupPh = true;
				}
			}
			else 
			{
				timeOutErrorPhValue = 0;
			}
		}
		else if( !g_isErrRegulator )
		{
			// открываем регулятор полностью
			Reg_RelayOn( REL_PH_PLUS );
			vTaskDelay( 200 );
			if( IsCurrent_PH_PLUS() ) 
				g_isErrRegulator = Reg_ToOpen();
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

/*******************************************************
	Чтение коэффициентов PID регулятора
********************************************************/
int Reg_ReadCoefficient( uint16_t idx )
{
	int value = -1;
	switch( idx )
	{
		case 0:
			value = g_K_PROP * 1000;
			break;
		case 1:
			value = g_K_INTEGRAL * 1000;
			break;
		case 2:
			value = g_K_DIFF * 1000;
			break;
		default:
			value = -1;
			break;
	}
	
	return value;
}

/*******************************************************
	Запись коэффициентов PID регулятора
********************************************************/
bool Reg_WriteCoefficient( uint16_t idx, uint16_t val )
{
	if( idx > 2 )
		return false;
	
	uint16_t ee_addr = EEADR_COEF_PROP + idx * sizeof(uint16_t);
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteBytes( ee_addr, (uint8_t*) &val, 2 );
	
	if( isWrited )
	{
		switch( idx )
		{
			case 0:
				g_K_PROP = (float)val  / 1000.0;
				break;
			case 1:
				g_K_INTEGRAL = (float)val  / 1000.0;
				break;
			case 2:
				g_K_DIFF = (float)val  / 1000.0;
				break;
			default:
				break;
		}
	}
	
	return isWrited;
}

/*******************************************************
	Чтение максимального времени отсутствия воды
********************************************************/
int Reg_Read_MAX_OUT_OF_WATER_SEC( uint16_t idx )
{
	int ivalue = MAX_OUT_OF_WATER_SEC; 
	
	return ivalue;
}

/*******************************************************
	Чтение максимального времени регулировки PH до ошибки
********************************************************/
int Reg_Read_MAX_TIME_ERROR_PH_SEC( uint16_t idx )
{
	int ivalue = MAX_TIME_ERROR_PH_SEC; 
	
	return ivalue;
}

/*******************************************************
	Запись максимального времени отсутствия воды
********************************************************/
bool Reg_Write_MAX_OUT_OF_WATER_SEC( uint16_t idx, uint16_t val )
{
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteWords( EEADR_MAX_OUT_OF_WATER_SEC, &val, 1 );

	if( isWrited )
	{
		MAX_OUT_OF_WATER_SEC = val;
	}
	
	return isWrited;
}

/*******************************************************
	Запись максимального времени регулировки PH до ошибки
********************************************************/
bool Reg_Write_MAX_TIME_ERROR_PH_SEC( uint16_t idx, uint16_t val )
{
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteWords( EEADR_MAX_TIME_ERROR_PH_SEC, &val, 1 );

	if( isWrited )
	{
		MAX_TIME_ERROR_PH_SEC = val;
	}
	
	return isWrited;
}

/*******************************************************
	Чтение времени хода регуятора PH
********************************************************/
int Reg_Read_FULL_MOVE_TIME_SEC( uint16_t idx )
{
	int ivalue = FULL_MOVE_TIME_SEC; 
	
	return ivalue;
}

/*******************************************************
	Запись времени хода регуятора PH
********************************************************/
bool Reg_Write_FULL_MOVE_TIME_SEC( uint16_t idx, uint16_t val )
{
	if( val < MIN_FULL_MOVE_TIME_SEC || val > MAX_FULL_MOVE_TIME_SEC )
		return false;
	
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteWords( EEADR_FULL_MOVE_TIME_SEC, &val, 1 );

	if( isWrited )
	{
		FULL_MOVE_TIME_SEC = val;
	}
	
	return isWrited;
}

/*******************************************************
	Проверка работоспособности регуятора PH
********************************************************/
bool Reg_IsError( void )
{
	// !!! ВРЕМЕННАЯ ЗАГЛУШКА
	//return false;

	bool isError = false;
	
	Reg_RelayOff( REL_PH_PLUS );
	Reg_RelayOff( REL_PH_MINUS );
	// делаем небольшую паузу
	vTaskDelay( 200 );
	if( IsCurrent_PH_MINUS() || IsCurrent_PH_PLUS() )
	{
		// если есть ток при выключенных реле
		isError = true;
	}
	else
	{
		// Включаем реле закрытия регулятора
		Reg_RelayOn( REL_PH_MINUS );
		// делаем небольшую паузу
		vTaskDelay( 200 );
		if( !IsCurrent_PH_MINUS() )
		{
			// нет тока в цепи закрытия при включенном реле
			// возможно мы находимся в полностью закрытом положении
			// тогда концевик открытия должен быть не замкнут, проверяем
			Reg_RelayOff( REL_PH_MINUS );
			Reg_RelayOn( REL_PH_PLUS );
			// делаем небольшую паузу
			vTaskDelay( 200 );
			if( !IsCurrent_PH_PLUS() )
			{
				// нет тока в цепи открытия в положении когда он должен быть, ошибка
				isError = true;
			}
		}
		else
		{
			// есть ток в цепи закрытия
			Reg_RelayOff( REL_PH_MINUS );
		}
	}
	
	return isError;
}

/*******************************************************
	Попытка полностью открыть регулятор
********************************************************/
bool Reg_ToOpen( void )
{
	// !!! ВРЕМЕННАЯ ЗАГЛУШКА
	//return true;
	
	bool isOpenOk = false;
	
	int openTime;
	
	// Отключаем насос
	switchPUMP( 0 );
	
	if( !Reg_IsError() )
	{
		// Здесь крутим до положения "полностью открыто"
		Reg_RelayOn( REL_PH_PLUS );
		// ждем срабатывания концевика открытия
		openTime = 0;
		
		// ждем пропадания тока в цепи мотора
		for( ;; )
		{
			vTaskDelay( 1000 );
			openTime++;

			if( !IsCurrent_PH_PLUS() ) 
			{
				// нет тока в цепи мотора, сработал концевик, регулятор открыт полностью
				isOpenOk = true;
				break;
			}
			
			if( openTime > FULL_MOVE_TIME_SEC )
			{
				// время хода больше установленного в настройках для этого регулятора
				// либо концевик неисправен, либо мотор не крутится
				break;
			}
		
		}

		Reg_RelayOff( REL_PH_PLUS );
	}
	
	return isOpenOk;
}
