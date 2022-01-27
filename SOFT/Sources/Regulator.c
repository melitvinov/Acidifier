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
#include "Rs485.h"

//--- CONSTANTS ------------------

const float K_INTEGRAL_DEFAULT = 0.15;
const float K_DIFF_DEFAULT = 10.0;
const float K_PROP_DEFAULT = 5.0;
const uint16_t REG_CYCLETIME_SEC_DEFAULT = 6;
const uint16_t MIN_REG_CYCLETIME_SEC = 3;		// минимальный период регулятора в секундах
const uint16_t MAX_REG_CYCLETIME_SEC = 20;		// максимальный период регулятора в секундах

const int MIN_REGIMP_PACK_TIME_MS = 250;			// минимальная длина импульса регулятора в пачке
const int MIN_REGIMP_ONE_TIME_MS = 100;				// минимальная длина импульса регулятора в пачке

//--- GLOBAL VARIABLES -----------
extern float g_Sensor_PH;					// текущее значение PH с датчиков
extern float g_Setup_PH;					// заданное пользователем значение PH

extern bool g_isNoWater;
//bool is_WaterOk_prev = false;
extern bool g_isErrTimeoutSetupPh;
extern bool g_isErrSensors;

float g_K_INTEGRAL;
float g_K_DIFF;
float g_K_PROP;

uint16_t g_TIMEOUT_REGULATOR_ON_SEC;	// задержка включения регулятора и насоса после подачи воды (сек.)
uint16_t g_TIMEOUT_ERROR_PH_SEC;		// длительность отклонения величины PH от заданного до установки ошибки (сек.)
uint16_t g_REGULATOR_CYCLETIME_SEC;		// период регулирования (сек)
uint16_t g_DELAY_PUMP_OFF_SEC;			// задержка отключения насоса (сек)

float g_flRegPercentOn;					// текущее (последнее значение процента открытия регулятора в %)
float g_PID_Value;						// текущее значение PID
float g_flDeltaPercent;					// текущее значение уменьшения или увеличения процента открытия
int g_ImpulseTime_ms;					// длительность открытия клапана в мс.
float g_PID_IntegralValue = 0;			// накопленный интегральный компонент для расчета PID
float g_prev_PhValue = 7;				// предыдущее значение Ph  для расчета PID

int g_TimeToStartReg_ms;				// оставшееся время до начала дозации после включения насоса в мс
int g_TimeToStartCalcPID_ms;			// оставшееся время до начала вычисления PID в мс

// таблица оптимальных начальных значений регулятора для трех ph значений 
// index 0 - Ph 4..4,99
// index 1 - Ph 5..5,99
// index 0 - Ph 6..6,99
TOptTablePoint g_OptTablePoints[3];		

TimerHandle_t xTimer;

//--- FUNCTIONS ------------------
extern int ReadWorkMode( uint16_t idx );

void Thread_Klapan( void *pvParameters );
void switch_VALVE( uint8_t on );
void switch_PUMP( uint8_t on );

/* Define a callback function that will be used by multiple timer
 instances.  The callback function does nothing but count the number
 of times the associated timer expires, and stop the timer once the
 timer has expired 10 times.  The count is saved as the ID of the
 timer. */
void vTimerCallback( TimerHandle_t xTimer )
{
	/* Optionally do something if the pxTimer parameter is NULL. */
	configASSERT( xTimer );
	
	xTimerStop( xTimer, 0 );
	switch_PUMP(0);
}
 
/*******************************************************
Функция		: Включение регулятора. (после подачи воды)
Возвр. знач.: нет
********************************************************/
void Regulator_START(void) 
{
	// Инициализируем переменные для расчета PID
	g_PID_Value = 0;					// текущее значение PID
	g_flDeltaPercent = 0;				// текущее значение уменьшения или увеличения процента открытия
	g_PID_IntegralValue = 0;			// накопленный интегральный компонент для расчета PID
	g_prev_PhValue = 7;					// предыдущее значение Ph  для расчета PID
	
	// Устанавливаем значение регулятора
	int index;
	if( g_Setup_PH >= 6 ) index = 0;
	else if( g_Setup_PH >= 5 ) index = 1;
	else index = 2;
	
	if( g_OptTablePoints[index].ph_setup_user != 0xFFFF && g_OptTablePoints[index].reg_value_opt_calc != 0xFFFF )
		g_flRegPercentOn = g_OptTablePoints[index].reg_value_opt_calc;
	else
		g_flRegPercentOn = g_OptTablePoints[index].reg_value_opt_def;

		if( g_flRegPercentOn > 100 )
	{
		// !!! Ошибка
		g_flRegPercentOn = 5;			// если при начальной установке положения регулятора произошла ошибка, ставим 5% открытия
	}
	
	// устанавливаем таймер на старт процесса регулирования
	g_TimeToStartReg_ms = g_TIMEOUT_REGULATOR_ON_SEC * 1000;
	// устанавливаем таймер на старт процесса вычисления PID (10 сек.)
	g_TimeToStartCalcPID_ms = 10000;	
	
	// Включаем насос
	configASSERT( xTimer );
	xTimerStop( xTimer, 0 );
	switch_PUMP( 1 );
}

void Regulator_STOP(void) 
{
	// Выключаем насос через задержку
	configASSERT( xTimer );
	
	xTimerChangePeriod( xTimer, g_DELAY_PUMP_OFF_SEC > 0 ? (g_DELAY_PUMP_OFF_SEC * 1000) : 100, 0 );
	
	BaseType_t result = xTimerStart( xTimer, 50 );
	if( result != pdPASS )
	{ /* The timer could not be set into the Active
	   state. */
	
		// через таймер не вышло, отключаем сразу
		switch_PUMP( 0 ); 
	}
		
	// Отключаем клапан
	switch_VALVE(0);	
}

/*******************************************************
Функция		: Вкл. / выкл. насоса
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void switch_PUMP( uint8_t on )
{
	Led_OnOff( LED_WORK_OK, on );
	GPIO_PinWrite( PORT_RELAY_PUMP, PIN_RELAY_PUMP, on );
	
	// Отключаем клапан (на всякий случай)
	switch_VALVE(0);	
}

bool IsPumpTurnON( void )
{
	bool isOn = GPIO_PinRead( PORT_RELAY_PUMP, PIN_RELAY_PUMP ) == 0 ? false : true;
	return isOn;
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

/*******************************************************
	Возвращает флаг нужно сейчас регулировать или нет
********************************************************/
bool IsRegulatingOn( void )
{
	bool isOn;
	
	isOn = (!g_isNoWater && !g_isErrSensors && !g_isErrTimeoutSetupPh && (g_TimeToStartReg_ms <= 0) );
	
	return isOn;
}

/*******************************************************
Функция		: Вкл. / выкл. клапана
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void switch_VALVE( uint8_t on )
{
	if( on == 0 ) {
		Led_OnOff( LED_VALVE, 0 );
		GPIO_PinWrite( PORT_RELAY_VALVE , PIN_RELAY_VALVE, 1 );
	}
	else if( IsPumpTurnON() ) {
		Led_OnOff( LED_VALVE, 1 );
		GPIO_PinWrite( PORT_RELAY_VALVE , PIN_RELAY_VALVE, 0 );
	}
}

void setUpDefaultOptTable( void )
{
	// Инициализируем значениями по умолчанию таблицу оптимальных настроек
	for( int i=0; i<3; i++ )
	{
		g_OptTablePoints[i].ph_setup_def = 65 - (i*10);				// PH 		= 6.5	5.5		4.5
		g_OptTablePoints[i].reg_value_opt_def = 10 + (i*20);		// Value 	= 10	30		50
		g_OptTablePoints[i].ph_setup_user = 0xFFFF;					// PH 		= 0xFFFF - нет значения
		g_OptTablePoints[i].reg_value_opt_calc = 0xFFFF;			// Value 	= 0xFFFF - нет значения
	}
	FM24_WriteBytes( EEADR_OPT_TABLE, (uint8_t*) &g_OptTablePoints, sizeof(g_OptTablePoints) );
}

/*******************************************************
Функция		: Инициализация железа
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Reg_Init(void)
{
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE );

	// Перевод вывода на вход с подтяжкой к VCC для датчика воды
	GPIO_PinConfigure( PORT_SENS_WATER, PIN_SENS_WATER, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	
	// Перевод выводов реле насоса и клапана на выход
	GPIO_PinConfigure( PORT_RELAY_PUMP, PIN_RELAY_PUMP, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
	GPIO_PinConfigure( PORT_RELAY_VALVE, PIN_RELAY_VALVE, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
	// Отключение насоса и клапана
	switch_PUMP( 0 );
	switch_VALVE( 0 );
	
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

	if( !FM24_ReadWords( EEADR_TIMEOUT_REGULATOR_ON_SEC, &g_TIMEOUT_REGULATOR_ON_SEC, 1 ) || g_TIMEOUT_REGULATOR_ON_SEC == 0 )
	{
		g_TIMEOUT_REGULATOR_ON_SEC = 5;
		FM24_WriteWords( EEADR_TIMEOUT_REGULATOR_ON_SEC, &g_TIMEOUT_REGULATOR_ON_SEC, 1 );
	}

	if( !FM24_ReadWords( EEADR_TIMEOUT_ERROR_PH_SEC, &g_TIMEOUT_ERROR_PH_SEC, 1 ) || g_TIMEOUT_ERROR_PH_SEC == 0 )
	{
		g_TIMEOUT_ERROR_PH_SEC = 90;
		FM24_WriteWords( EEADR_TIMEOUT_ERROR_PH_SEC, &g_TIMEOUT_ERROR_PH_SEC, 1 );
	}

	if( !FM24_ReadWords( EEADR_REG_CYCLETIME_SEC, &g_REGULATOR_CYCLETIME_SEC, 1 ) || g_REGULATOR_CYCLETIME_SEC == 0 )
	{
		g_REGULATOR_CYCLETIME_SEC = REG_CYCLETIME_SEC_DEFAULT;
		FM24_WriteWords( EEADR_REG_CYCLETIME_SEC, &g_REGULATOR_CYCLETIME_SEC, 1 );
	}
	
	// инициализируем таблицу оптимальных значений регулятора в зависимости от установки PH
	if( !FM24_ReadBytes( EEADR_OPT_TABLE, (uint8_t*) &g_OptTablePoints, sizeof(g_OptTablePoints) ) )
	{
		setUpDefaultOptTable();
	}
	else 
	{
		bool needDef = false;
		for( int i=0; i<3; i++ )
			needDef |= g_OptTablePoints[i].ph_setup_def < 40 || g_OptTablePoints[i].ph_setup_def > 70;

		if( needDef )
			setUpDefaultOptTable();
	}

	if( !FM24_ReadWords( EEADR_DELAY_PUMP_OFF_SEC, &g_DELAY_PUMP_OFF_SEC, 1 ) || ( !g_DELAY_PUMP_OFF_SEC ) )
	{
		g_DELAY_PUMP_OFF_SEC = 5;
		FM24_WriteWords( EEADR_DELAY_PUMP_OFF_SEC, &g_DELAY_PUMP_OFF_SEC, 1 );
	}
	if( g_DELAY_PUMP_OFF_SEC > 20 ) {
		g_DELAY_PUMP_OFF_SEC = 20;
		FM24_WriteWords( EEADR_DELAY_PUMP_OFF_SEC, &g_DELAY_PUMP_OFF_SEC, 1 );
	}		
}

/*******************************************************
Поток		: Расчет величины PID 
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_Pid( void *pvParameters )
{
	float prop_value, integ_value, diff_value, error_ph;

	TickType_t xLastWakeTime  = xTaskGetTickCount();
	const TickType_t PERIOD_MS = 1000;
		
	for(;;)
	{
		vTaskDelayUntil( &xLastWakeTime, PERIOD_MS );

		if( IsRegulatingOn() && (g_TimeToStartCalcPID_ms <= 0) )
		{
			// при наличии воды и отсутсвии ошибок вычисляем величину PID
			// ошибка Ph
			error_ph = g_Setup_PH - g_Sensor_PH;
			// пропорциональный компонент
			prop_value = g_K_PROP * error_ph;
			// интегральный компонент
			integ_value =  g_K_INTEGRAL * error_ph;
			// добавляем накопленный интегральный компонент
			integ_value += g_PID_IntegralValue;
			// сохраняем текущее значение накопленного интегрального компонента
			g_PID_IntegralValue = integ_value;
			// дифференциальный компонент
			diff_value = g_Sensor_PH - g_prev_PhValue;
			diff_value *= g_K_DIFF;
			
			// сохраняем текущее значение Ph для будущих расчетов
			g_prev_PhValue = g_Sensor_PH;

			g_PID_Value = prop_value + integ_value + diff_value;
			g_PID_Value *= -1.0;
		}
	}
}

/*******************************************************
Поток		: Управление клапаном дозации
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_Klapan( void *pvParameters )
{
	const int MAX_IMP_COUNT_BY_CYCLE = 6;

	float prevPercentOn;
	bool cycleIsBreaked;
	int impHigh_TimeMs, impLow_TimeMs, impCount;
	
	switch_VALVE(0);
	g_PID_IntegralValue = 0;
	g_PID_Value = 0;
	
	vTaskDelay( 1000 );

	TickType_t xLastWakeTime, xCurrTicks, xTickToNextCycle;
	for(;;)
	{
		// Проверяем что процесс регулирования разрешен
		if( !IsRegulatingOn() )
		{
			// Если нет, закрываем клапан и ждем дальше
			switch_VALVE(0);

			vTaskDelay(50);
			continue;
		}

		// Запоминаем время начала цикла дозации
		xLastWakeTime = xTaskGetTickCount();
		
		// насос включен, то вычисляем параметры импульсов
		g_flDeltaPercent = g_PID_Value;
		g_flRegPercentOn += g_flDeltaPercent;
		
		if( g_flRegPercentOn > 95 )
			g_flRegPercentOn = 95;
		if( g_flRegPercentOn < 0 )
			g_flRegPercentOn = 0;

		g_ImpulseTime_ms = (g_REGULATOR_CYCLETIME_SEC * 10) * g_flRegPercentOn;
		
		if( g_ImpulseTime_ms > MIN_REGIMP_ONE_TIME_MS )
		{
			for( impCount = MAX_IMP_COUNT_BY_CYCLE; impCount>0; impCount-- )
			{
				impHigh_TimeMs = g_ImpulseTime_ms / impCount;
				
				if( impHigh_TimeMs > MIN_REGIMP_PACK_TIME_MS )
				{
					// считаем суммарное время импульсов
					int iSumImpulseMs = impHigh_TimeMs * impCount;
					// считаем паузу между импульсами
					impLow_TimeMs = (((g_REGULATOR_CYCLETIME_SEC * 1000)-100) - iSumImpulseMs) / impCount;
					if( impLow_TimeMs > MIN_REGIMP_PACK_TIME_MS )
					{
						// если все импульсы укладываются в слот времени цикла - выходим
						break;
					}
				}
			}
			
			if( impCount == 0 && (impHigh_TimeMs > MIN_REGIMP_ONE_TIME_MS) )
			{
				impCount = 1;
				impLow_TimeMs = (g_REGULATOR_CYCLETIME_SEC * 1000)-100 - impHigh_TimeMs;
			}
			
			// сохраняем текущее значение регулятора
			prevPercentOn = g_flRegPercentOn;
			cycleIsBreaked = false;
			while( impCount-- )
			{
				switch_VALVE(1);
				vTaskDelay( impHigh_TimeMs );
				switch_VALVE(0);
				vTaskDelay( impLow_TimeMs );
				
				if( (fabs( g_flRegPercentOn - prevPercentOn ) > 5) || 
						g_isNoWater || 
						g_isErrSensors || 
						g_isErrTimeoutSetupPh || 
						(ReadWorkMode(0) != Mode_RegulatorPh) )
				{
					// если за время дозации данного цикла произошло изменение 
					// значения регулятора больше чем на 5% или кончилась вода или возникли ошибки
					// то бросаем этот цикл и делаем все заново чтобы ускорить реакцию на это изменение
					cycleIsBreaked = true;
					break;
				}
			}			

			if(  cycleIsBreaked ) {
				// что то случилось, идем в начало чтобы среагировать
				continue;
			}
		}
		else
		{
			g_ImpulseTime_ms = 0;
			switch_VALVE(0);
		}
		
		xCurrTicks = xTaskGetTickCount();
		
		if( g_REGULATOR_CYCLETIME_SEC * 1000 > ( xCurrTicks - xLastWakeTime ) )
		{			
			xTickToNextCycle = (g_REGULATOR_CYCLETIME_SEC * 1000) - (xCurrTicks - xLastWakeTime);
		}
		else
		{
			xTickToNextCycle = 10;
		}
			
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xTickToNextCycle );
	}
}

void SaveRegulatorValue( float setup_PH, uint16_t value )
{
	int index = 0;
	if( setup_PH < 5 )	index = 2;
	else if( setup_PH < 6 )	index = 1;
	
	g_OptTablePoints[index].ph_setup_user = roundf( setup_PH * 10 );
	g_OptTablePoints[index].reg_value_opt_calc = value;
	
	FM24_WriteBytes( EEADR_OPT_TABLE, (uint8_t*) &g_OptTablePoints, sizeof(g_OptTablePoints) );
}

/*******************************************************
Поток		: Рабочий поток устройства
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_Regulator( void *pvParameters )
{
	const int REG_WUp_Time = 100; // время между циклами работы потока в мс.
	
	TickType_t xLastWakeTime;
	int timeOutErrorPhValue = 0;
	int timeOutErrorPhSensors = 0;
	int timeValuePhIsGood = 0;
	int sumRegValue, countAvgValues;

	bool IsPhSensorsTooDiff;
	//bool is_WaterOk_curr;

	Reg_Init();

	g_isNoWater = true;

	vTaskDelay( 5000 );

	xTaskCreate( Thread_Klapan, (const char*)"Klapan", configMINIMAL_STACK_SIZE, ( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	xTaskCreate( Thread_Pid, (const char*)"Pid", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTimer = xTimerCreate
                   ( /* Just a text name, not used by the RTOS
                     kernel. */
                     "Timer",
                     /* The timer period in ticks, must be
                     greater than 0. */
                     g_DELAY_PUMP_OFF_SEC > 0 ? (g_DELAY_PUMP_OFF_SEC * 1000) : 10,
                     /* The timers will auto-reload themselves
                     when they expire. */
                     pdTRUE,
                     /* The ID is used to store a count of the
                     number of times the timer has expired, which
                     is initialised to 0. */
                     ( void * ) 0,
                     /* Each timer calls the same callback when
                     it expires. */
                     vTimerCallback
                   );

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, REG_WUp_Time );

		// Проверка режима работы
		if( ReadWorkMode(0) != Mode_RegulatorPh || g_isErrSensors || g_isErrTimeoutSetupPh )
		{
			// Если не в режиме регулирования, отключаем насос и ждем 
			switch_PUMP(0); 
			continue;
		}
		
		// Здесь мы в режиме регулирования, ошибок нет

		// Проверка воды
		if( g_isNoWater != !IsWaterOk() )
		{
			// Проверяем изменение наличия воды
			g_isNoWater = !IsWaterOk();
			
			if( g_isNoWater ) {
				// вода пропала
				Regulator_STOP();
			}
			else {
				// вода появилась, начинаем новый полив.
				Regulator_START();
			}
			continue;
		}
		
		if( g_isNoWater ) {
			// если нет воды, просто ждем
			continue;
		}

		// Работа с таймерами
		if( g_TimeToStartReg_ms > 0 ) {
			// уменьщаем остаток времени задержки
			g_TimeToStartReg_ms -= REG_WUp_Time;
			continue;
		}
		
		if( g_TimeToStartCalcPID_ms > 0 ) {
			// после задержки включения процесса регулирования, начинаем уменьшать время задержки вычисления PID
			g_TimeToStartCalcPID_ms -= REG_WUp_Time;
		}
		
		if( !IsPumpTurnON() )
			switch_PUMP(1); // Включаем насос
			
		// Тут у нас вода есть, насос включен, задержка регулирования кончилась. Регулируем.
		
		// проверяем ошибки по датчикам PH и выводим на дисплей значения PH
		g_Sensor_PH = AInp_GetSystemPh( &IsPhSensorsTooDiff );
		if( g_Sensor_PH < 0 || IsPhSensorsTooDiff )
		{
			if( !g_isErrSensors )
			{
				timeOutErrorPhSensors  += REG_WUp_Time;
				if( timeOutErrorPhSensors > TIMEOUT_SENSORS_TOO_DIFF_MS )
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

		if( fabs( g_Sensor_PH - g_Setup_PH ) > 0.5 )
		{
			timeOutErrorPhValue += REG_WUp_Time;
			if( timeOutErrorPhValue > (g_TIMEOUT_ERROR_PH_SEC * 1000) )
			{
				timeOutErrorPhValue = 0;
				g_isErrTimeoutSetupPh = true;
			}
		}
		else 
		{
			timeOutErrorPhValue = 0;
		}
		if( fabs( g_Sensor_PH - g_Setup_PH ) < 0.2 )
		{
			timeValuePhIsGood += REG_WUp_Time;
			countAvgValues++;
			sumRegValue += roundf( g_flRegPercentOn );
			if( timeValuePhIsGood > 60000 )
			{
				timeValuePhIsGood = 0;
				// здесь надо сохранить положение регулятора в таблицу оптимальных значений
				uint16_t avgRegValue = sumRegValue / countAvgValues;
				SaveRegulatorValue( g_Setup_PH, avgRegValue );
			}
		}
		else 
		{
			countAvgValues = 0;
			sumRegValue = 0;
			timeValuePhIsGood = 0;
		}
	}
}


/*******************************************************
Функция		: Включает реле по его индексу
Параметр 1	: индекс реле
Возвр. знач.: нет
********************************************************/
/*
void Reg_RelayOn(uint8_t indx)
{
	if( indx > 1 )
		return;
	
	GPIO_PinWrite( Relay[indx].GPIOx, Relay[indx].pin, REL_ON );
}
*/
/*******************************************************
Функция		: Отключает реле по его индексу
Параметр 1	: индекс реле
Возвр. знач.: нет
********************************************************/
/*
void Reg_RelayOff(uint8_t indx)
{
	if( indx > 1 )
		return;
	
	GPIO_PinWrite( Relay[indx].GPIOx, Relay[indx].pin, REL_OFF );
}
*/
/*******************************************************
Функция		: Отключает оба реле
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
/*
void Reg_RelayAllOff(void)
{
	GPIO_PinWrite( Relay[0].GPIOx, Relay[0].pin, REL_OFF );
	GPIO_PinWrite( Relay[1].GPIOx, Relay[1].pin, REL_OFF );
}
*/
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
int Reg_Read_TIMEOUT_REGULATOR_ON_SEC( uint16_t idx )
{
	int ivalue = g_TIMEOUT_REGULATOR_ON_SEC; 
	
	return ivalue;
}

/*******************************************************
	Чтение максимального времени регулировки PH до ошибки
********************************************************/
int Reg_Read_TIMEOUT_ERROR_PH_SEC( uint16_t idx )
{
	int ivalue = g_TIMEOUT_ERROR_PH_SEC; 
	
	return ivalue;
}

/*******************************************************
	Запись максимального времени отсутствия воды
********************************************************/
bool Reg_Write_TIMEOUT_REGULATOR_ON_SEC( uint16_t idx, uint16_t val )
{
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteWords( EEADR_TIMEOUT_REGULATOR_ON_SEC, &val, 1 );

	if( isWrited )
	{
		g_TIMEOUT_REGULATOR_ON_SEC = val;
	}
	
	return isWrited;
}

/*******************************************************
	Запись максимального времени регулировки PH до ошибки
********************************************************/
bool Reg_Write_TIMEOUT_ERROR_PH_SEC( uint16_t idx, uint16_t val )
{
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteWords( EEADR_TIMEOUT_ERROR_PH_SEC, &val, 1 );

	if( isWrited )
	{
		g_TIMEOUT_ERROR_PH_SEC = val;
	}
	
	return isWrited;
}

int Reg_Read_DELAY_PUMP_OFF_SEC( uint16_t idx )
{
	return g_DELAY_PUMP_OFF_SEC;
}
bool Reg_Write_DELAY_PUMP_OFF_SEC( uint16_t idx, uint16_t val )
{
	if( val > 20 ) val = 20;
	
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteWords( EEADR_DELAY_PUMP_OFF_SEC, &val, 1 );

	if( isWrited )
	{
		g_DELAY_PUMP_OFF_SEC = val;
	}
	
	return isWrited;
	
}

/*******************************************************
	Чтение времени хода регуятора PH
********************************************************/
int Reg_Read_REG_CYCLETIME_SEC( uint16_t idx )
{
	int ivalue = g_REGULATOR_CYCLETIME_SEC; 
	
	return ivalue;
}

/*******************************************************
	Чтение одного из регистров мониторинга текущего состояния
********************************************************/
int Reg_Read_MonitoringValue( uint16_t idx )
{
	int ivalue = -1;
	
	EMonitoringType type = (EMonitoringType) idx;
	
	switch( type )
	{
		case MON_IsNoWater:
			ivalue = g_isNoWater;
			break;
		case MON_IsErrSensors:
			ivalue = g_isErrSensors;
			break;
		case MON_IsErrTimeoutSetupPh:
			ivalue = g_isErrTimeoutSetupPh;
			break;
		case MON_PH_Setup:
			ivalue = g_Setup_PH * 100;
			break;
		case MON_PH1_Current:
			ivalue = AInp_ReadPhValue(0);
			break;
		case MON_PH2_Current:
			ivalue = AInp_ReadPhValue(1);
			break;
		
		case MON_RegPercentOn:
			ivalue = fabs( roundf( g_flRegPercentOn * 100 ) );
			break;
		case MON_PID_Value:
			ivalue = fabs( g_PID_Value ) * 100;
			break;
		case MON_PID_Positive:
			ivalue = g_PID_Value >= 0 ? 1 : 0; 
			break;
		case MON_DeltaPercent:
			ivalue = fabs( roundf( g_flDeltaPercent * 100 ));
			break;
		case MON_DeltaPercentPositive:
			ivalue = g_flDeltaPercent >= 0 ? 1 : 0; 
			break;
		case MON_ImpulseTime_ms:
			ivalue = abs( g_ImpulseTime_ms );
			break;
	}
	
	return ivalue;
}


/*******************************************************
	Запись времени хода регуятора PH
********************************************************/
bool Reg_Write_REG_CYCLETIME_SEC( uint16_t idx, uint16_t val )
{
	if( val < MIN_REG_CYCLETIME_SEC || val > MAX_REG_CYCLETIME_SEC )
		return false;
	
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteWords( EEADR_REG_CYCLETIME_SEC, &val, 1 );

	if( isWrited )
	{
		g_REGULATOR_CYCLETIME_SEC = val;
	}
	
	return isWrited;
}

//void Reg_RestartWaterTimer(void)
//{
//	// запускаем таймер на старт процесса регулирования
//	g_TimeToStartReg_ms = g_TIMEOUT_REGULATOR_ON_SEC * 1000;
//}

/*******************************************************
	Проверка работоспособности регуятора PH
********************************************************/
/*
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
*/

/*******************************************************
	Попытка полностью открыть регулятор
********************************************************/
/*
bool Reg_ToOpen( void )
{
	// !!! ВРЕМЕННАЯ ЗАГЛУШКА
	//return true;
	
	bool isOpenOk = false;
	
	int openTime;
	
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
*/
