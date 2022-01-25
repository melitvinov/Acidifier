/***********************************************************************************
* ������������ ������	: ���������� ����������� PH
*-----------------------------------------------------------------------------------
* ������				: 1.0
* �����					: ��������
* ���� ��������			: 10.03.2021
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
const uint16_t MIN_REG_CYCLETIME_SEC = 3;		// ����������� ������ ���������� � ��������
const uint16_t MAX_REG_CYCLETIME_SEC = 20;		// ������������ ������ ���������� � ��������

const int MIN_REGIMP_PACK_TIME_MS = 250;			// ����������� ����� �������� ���������� � �����
const int MIN_REGIMP_ONE_TIME_MS = 100;				// ����������� ����� �������� ���������� � �����

//--- GLOBAL VARIABLES -----------
extern float g_Sensor_PH;					// ������� �������� PH � ��������
extern float g_Setup_PH;					// �������� ������������� �������� PH

extern bool g_isNoWater;
bool is_WaterOk_prev = false;
extern bool g_isErrTimeoutSetupPh;
extern bool g_isErrSensors;

float g_K_INTEGRAL;
float g_K_DIFF;
float g_K_PROP;

uint16_t MAX_OUT_OF_WATER_SEC;		// ����. ������������ ��������� ���� ��� ������ (���.)
uint16_t MAX_TIME_ERROR_PH_SEC;		// ����. ������������ ������ ��������� PH (���.)
uint16_t REG_CYCLETIME_SEC;			// ������ ������������� (���)

float g_flRegPercentOn;				// ������� (��������� �������� �������� �������� ���������� � %)
float g_PID_Value;					// ������� �������� PID
float g_flDeltaPercent;				// ������� �������� ���������� ��� ���������� �������� ��������
int g_ImpulseTime_ms;				// ������������ �������� ������� � ��.
float g_PID_IntegralValue = 0;		// ����������� ������������ ��������� ��� ������� PID
float g_prev_PhValue = 7;			// ���������� �������� Ph  ��� ������� PID

//const uint16_t TIME_TO_START_REG_SEC = 15;				// ����� �� ������ ������ ���������� ����� ������ ����
int g_TimeToStartReg_ms;// = TIME_TO_START_REG_SEC * 1000;	// ������ ��� ������ ������ ����������

//--- FUNCTIONS ------------------
extern int ReadWorkMode( uint16_t idx );

void Thread_Klapan( void *pvParameters );

/*******************************************************
�������		: ���. / ����. ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void switch_PUMP( uint8_t on )
{
	Led_OnOff( LED_WORK_OK, on );
	GPIO_PinWrite( PORT_RELAY_PUMP, PIN_RELAY_PUMP, on );
	
	if( on ) 
	{
		// ����� �������� �����, �� ������� ����� �����. 
		// �������������� ���������� ��� ������� PID
		g_flRegPercentOn = 20;				// ������� (��������� �������� �������� �������� ���������� � %)
		g_PID_Value = 0;					// ������� �������� PID
		g_flDeltaPercent = 0;				// ������� �������� ���������� ��� ���������� �������� ��������
		g_PID_IntegralValue = 0;			// ����������� ������������ ��������� ��� ������� PID
		g_prev_PhValue = 6;					// ���������� �������� Ph  ��� ������� PID
	}
}

bool isPumpTurnON( void )
{
	bool isOn = GPIO_PinRead( PORT_RELAY_PUMP, PIN_RELAY_PUMP ) == 0 ? false : true;
	return isOn;
}

/*******************************************************
�������		: ���. / ����. �������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void switch_VALVE( uint8_t on )
{
	if( on == 0 ) {
		Led_OnOff( LED_VALVE, 0 );
		GPIO_PinWrite( PORT_RELAY_VALVE , PIN_RELAY_VALVE, 1 );
	}
	else if( isPumpTurnON() ) {
		Led_OnOff( LED_VALVE, 1 );
		GPIO_PinWrite( PORT_RELAY_VALVE , PIN_RELAY_VALVE, 0 );
	}
}

/*******************************************************
�������		: ������������� ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Reg_Init(void)
{
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE );

	// ������� ������ �� ���� � ��������� � VCC ��� ������� ����
	GPIO_PinConfigure( PORT_SENS_WATER, PIN_SENS_WATER, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	
	// ������� ������� ���� ������ � ������� �� �����
	GPIO_PinConfigure( PORT_RELAY_PUMP, PIN_RELAY_PUMP, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
	GPIO_PinConfigure( PORT_RELAY_VALVE, PIN_RELAY_VALVE, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
	// ���������� ������ � �������
	switch_PUMP( 0 );
	switch_VALVE( 0 );
	
	// ��������� ������������ ��� �������
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

	if( !FM24_ReadWords( EEADR_REG_CYCLETIME_SEC, &REG_CYCLETIME_SEC, 1 ) || REG_CYCLETIME_SEC == 0 )
	{
		REG_CYCLETIME_SEC = REG_CYCLETIME_SEC_DEFAULT;
		FM24_WriteWords( EEADR_REG_CYCLETIME_SEC, &REG_CYCLETIME_SEC, 1 );
	}
	
//	uint16_t ee_percent_on;
//	if( !FM24_ReadWords( EEADR_REG_LAST_REGPOS_VALUE, &ee_percent_on, 1 ) || ee_percent_on > 10000 )
//	{
//		g_flRegPercentOn = 5;
//		ee_percent_on = 500;
//		FM24_WriteWords( EEADR_REG_LAST_REGPOS_VALUE, &ee_percent_on, 1 );
//	}
//	else 
//	{
//		g_flRegPercentOn = ee_percent_on;
//		g_flRegPercentOn /= 100.0;
//	}
}

/*******************************************************
	���������� ��������� ������� ������� ����
********************************************************/
bool IsWaterOk( void )
{
	bool isOk;
	isOk = ( GPIO_PinRead( PORT_SENS_WATER, PIN_SENS_WATER ) == 0 );
	
	return isOk;
}

/*******************************************************
�����		: ���������� �������� �������
�������� 1	: �� ������������
�����. ����.: ����������� ����
********************************************************/
void Thread_Klapan( void *pvParameters )
{
	const int MAX_IMP_COUNT_BY_CYCLE = 6;
	
	// Initialise the xLastWakeTime variable with the current time.
	int impHigh_TimeMs, impLow_TimeMs, impCount;
	float prop_value, integ_value, diff_value, error_ph;
	//uint16_t ee_percent_on;
	
	switch_VALVE(0);
	g_PID_IntegralValue = 0;
	g_PID_Value = 0;
	
	vTaskDelay( 1000 );

	TickType_t xLastWakeTime, xCurrTicks, xTickToNextCycle;
	for(;;)
	{
		xLastWakeTime = xTaskGetTickCount();
		
		if( g_isNoWater || g_isErrSensors || g_isErrTimeoutSetupPh || (ReadWorkMode(0) != Mode_RegulatorPh) )
		{
			switch_VALVE(0);
			switch_PUMP(0);
			vTaskDelay(50);
			continue;
		}

		// ��� ������� ���� � ��������� ������ - ����������
		// ������ Ph
		error_ph = g_Setup_PH - g_Sensor_PH;
		// ���������������� ���������
		prop_value = g_K_PROP * error_ph;
		// ������������ ���������
		integ_value =  g_K_INTEGRAL * error_ph;
		// ��������� ����������� ������������ ���������
		integ_value += g_PID_IntegralValue;
		// ��������� ������� �������� ������������ ������������� ����������
		g_PID_IntegralValue = integ_value;
		// ���������������� ���������
		diff_value = g_K_DIFF * ( g_Sensor_PH - g_prev_PhValue ) * 10.0;
		// ��������� ������� �������� Ph ��� ������� ��������
		g_prev_PhValue = g_Sensor_PH;

		g_PID_Value = prop_value + integ_value + diff_value;
		g_PID_Value *= -1.0;
		
		if( g_TimeToStartReg_ms <= 0 )
		{
			// ���� ������ �� ����� ������������� ����� ������ ���� �����, �� ��������� ��������� ���������
			g_flDeltaPercent = g_PID_Value;
			g_flRegPercentOn += g_flDeltaPercent;
			
			if( g_flRegPercentOn > 95 )
				g_flRegPercentOn = 95;
			if( g_flRegPercentOn < 0 )
				g_flRegPercentOn = 0;

			//ee_percent_on = g_flRegPercentOn * 100;
			//FM24_WriteWords( EEADR_REG_LAST_REGPOS_VALUE, &ee_percent_on, 1 );
		}
		else
		{
			// ���� ������ ��� �� ��������, �� ��������� ������ �������� ����� �������� �������������
			g_flDeltaPercent = 0;
			g_flRegPercentOn = g_flRegPercentOn;
		}

		g_ImpulseTime_ms = (REG_CYCLETIME_SEC * 10) * g_flRegPercentOn;
		
		if( g_ImpulseTime_ms > MIN_REGIMP_ONE_TIME_MS )
		{
			for( impCount = MAX_IMP_COUNT_BY_CYCLE; impCount>0; impCount-- )
			{
				impHigh_TimeMs = g_ImpulseTime_ms / impCount;
				
				if( impHigh_TimeMs > MIN_REGIMP_PACK_TIME_MS )
				{
					// ������� ��������� ����� ���������
					int iSumImpulseMs = impHigh_TimeMs * impCount;
					// ������� ����� ����� ����������
					impLow_TimeMs = (((REG_CYCLETIME_SEC * 1000)-100) - iSumImpulseMs) / impCount;
					if( impLow_TimeMs > MIN_REGIMP_PACK_TIME_MS )
					{
						// ���� ��� �������� ������������ � ���� ������� ����� - �������
						break;
					}
				}
			}
			
			if( impCount == 0 && (impHigh_TimeMs > MIN_REGIMP_ONE_TIME_MS) )
			{
				impCount = 1;
				impLow_TimeMs = (REG_CYCLETIME_SEC * 1000)-100 - impHigh_TimeMs;
			}
			
			while( impCount-- )
			{
				switch_VALVE(1);
				vTaskDelay( impHigh_TimeMs );
				switch_VALVE(0);
				if( g_isNoWater || g_isErrSensors || g_isErrTimeoutSetupPh || (ReadWorkMode(0) != Mode_RegulatorPh) )
					break;
				vTaskDelay( impLow_TimeMs );
			}			

			if( g_isNoWater || g_isErrSensors || g_isErrTimeoutSetupPh || (ReadWorkMode(0) != Mode_RegulatorPh) )
				continue;
		}
		else
		{
			g_ImpulseTime_ms = 0;
			switch_VALVE(0);
		}
		
		xCurrTicks = xTaskGetTickCount();
		
		if( REG_CYCLETIME_SEC * 1000 > ( xCurrTicks - xLastWakeTime ) )
		{			
			xTickToNextCycle = (REG_CYCLETIME_SEC * 1000) - (xCurrTicks - xLastWakeTime);
		}
		else
		{
			xTickToNextCycle = 10;
		}
			
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xTickToNextCycle );
	}
}

/*******************************************************
�����		: ������� ����� ����������
�������� 1	: �� ������������
�����. ����.: ����������� ����
********************************************************/
void Thread_Regulator( void *pvParameters )
{
	const int REG_WUp_Time = 50; // ����� ����� ������� ������ ������ � ��.
	
	TickType_t xLastWakeTime;
	int timeOutOfWater = 0;
	int timeOutErrorPhValue = 0;
	int timeOutErrorPhSensors = 0;

	bool IsPhSensorsTooDiff;
	bool is_WaterOk_curr;

	Reg_Init();

	g_isNoWater = true;

	vTaskDelay( 5000 );

	xTaskCreate( Thread_Klapan, (const char*)"Klapan", configMINIMAL_STACK_SIZE, ( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, REG_WUp_Time );

		if( g_TimeToStartReg_ms > 0 )
			g_TimeToStartReg_ms -= REG_WUp_Time;
		
		if( ReadWorkMode(0) != Mode_RegulatorPh )
		{
			switch_VALVE(0); 
			switch_PUMP(0); 
			timeOutErrorPhSensors = 0;
			timeOutErrorPhValue = 0;
			timeOutOfWater = 0;
			continue;
		}
		
		// ��������� ������� ����
		is_WaterOk_curr = IsWaterOk();
		
		if( !is_WaterOk_curr )
		{
			// ������ ��� ����
			switch_PUMP( 0 ); // ��������� �����
			switch_VALVE(0);	// ��������� ������

			g_isNoWater = true;
			timeOutOfWater = 0;
			g_prev_PhValue = g_Sensor_PH;
			
			// ������� ������ �� ����� �������� ������������� ����� ������ ����
			g_TimeToStartReg_ms = MAX_OUT_OF_WATER_SEC * 1000;
		}
		else 
		{
			// ������ ���� ����
			if( !is_WaterOk_prev )
			{
				// ���� ����� ���� �� ���� ����, �������� ������ �� ��������� ������ ����
				timeOutOfWater = 0;
			}
			else if( g_isNoWater )
			{
				// ���� ���� ����, �� ����� ������� ��� ���� ���, ��������� ������ �� ��������� ������
				timeOutOfWater += REG_WUp_Time;
				if( timeOutOfWater > (MAX_OUT_OF_WATER_SEC * 1000) )
				{
					g_isNoWater = false;
					
					switch_PUMP( 1 );	// �������� �����
					switch_VALVE(0);	// ��������� ������ (�� ������ ������)
					
					g_TimeToStartReg_ms = MAX_OUT_OF_WATER_SEC * 1000;				
				}
			}
		}
		is_WaterOk_prev = is_WaterOk_curr;
		
		// ��������� ������ �� �������� PH � ������� �� ������� �������� PH
		g_Sensor_PH = AInp_GetSystemPh( &IsPhSensorsTooDiff );
		if( g_Sensor_PH < 0 || IsPhSensorsTooDiff )
		{
			if( !g_isErrSensors )
			{
				timeOutErrorPhSensors  += REG_WUp_Time;
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
		
		if( !g_isNoWater && !g_isErrSensors && !g_isErrTimeoutSetupPh )
		{
			if( fabs( g_Sensor_PH - g_Setup_PH ) > 0.5 )
			{
				timeOutErrorPhValue += REG_WUp_Time;
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
		else
		{
			switch_VALVE(0);
		}
	}
}


/*******************************************************
�������		: �������� ���� �� ��� �������
�������� 1	: ������ ����
�����. ����.: ���
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
�������		: ��������� ���� �� ��� �������
�������� 1	: ������ ����
�����. ����.: ���
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
�������		: ��������� ��� ����
�������� 1	: ���
�����. ����.: ���
********************************************************/
/*
void Reg_RelayAllOff(void)
{
	GPIO_PinWrite( Relay[0].GPIOx, Relay[0].pin, REL_OFF );
	GPIO_PinWrite( Relay[1].GPIOx, Relay[1].pin, REL_OFF );
}
*/
/*******************************************************
	������ ������������� PID ����������
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
	������ ������������� PID ����������
********************************************************/
bool Reg_WriteCoefficient( uint16_t idx, uint16_t val )
{
	if( idx > 2 )
		return false;
	
	uint16_t ee_addr = EEADR_COEF_PROP + idx * sizeof(uint16_t);
	// ��������� � EEPROM
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
	������ ������������� ������� ���������� ����
********************************************************/
int Reg_Read_MAX_OUT_OF_WATER_SEC( uint16_t idx )
{
	int ivalue = MAX_OUT_OF_WATER_SEC; 
	
	return ivalue;
}

/*******************************************************
	������ ������������� ������� ����������� PH �� ������
********************************************************/
int Reg_Read_MAX_TIME_ERROR_PH_SEC( uint16_t idx )
{
	int ivalue = MAX_TIME_ERROR_PH_SEC; 
	
	return ivalue;
}

/*******************************************************
	������ ������������� ������� ���������� ����
********************************************************/
bool Reg_Write_MAX_OUT_OF_WATER_SEC( uint16_t idx, uint16_t val )
{
	// ��������� � EEPROM
	bool isWrited = FM24_WriteWords( EEADR_MAX_OUT_OF_WATER_SEC, &val, 1 );

	if( isWrited )
	{
		MAX_OUT_OF_WATER_SEC = val;
	}
	
	return isWrited;
}

/*******************************************************
	������ ������������� ������� ����������� PH �� ������
********************************************************/
bool Reg_Write_MAX_TIME_ERROR_PH_SEC( uint16_t idx, uint16_t val )
{
	// ��������� � EEPROM
	bool isWrited = FM24_WriteWords( EEADR_MAX_TIME_ERROR_PH_SEC, &val, 1 );

	if( isWrited )
	{
		MAX_TIME_ERROR_PH_SEC = val;
	}
	
	return isWrited;
}

/*******************************************************
	������ ������� ���� ��������� PH
********************************************************/
int Reg_Read_REG_CYCLETIME_SEC( uint16_t idx )
{
	int ivalue = REG_CYCLETIME_SEC; 
	
	return ivalue;
}

/*******************************************************
	������ ������ �� ��������� ����������� �������� ���������
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
	������ ������� ���� ��������� PH
********************************************************/
bool Reg_Write_REG_CYCLETIME_SEC( uint16_t idx, uint16_t val )
{
	if( val < MIN_REG_CYCLETIME_SEC || val > MAX_REG_CYCLETIME_SEC )
		return false;
	
	// ��������� � EEPROM
	bool isWrited = FM24_WriteWords( EEADR_REG_CYCLETIME_SEC, &val, 1 );

	if( isWrited )
	{
		REG_CYCLETIME_SEC = val;
	}
	
	return isWrited;
}

void Reg_RestartWaterTimer(void)
{
	g_isNoWater = true;
	is_WaterOk_prev = false;
}

/*******************************************************
	�������� ����������������� ��������� PH
********************************************************/
/*
bool Reg_IsError( void )
{
	// !!! ��������� ��������
	//return false;

	bool isError = false;
	
	Reg_RelayOff( REL_PH_PLUS );
	Reg_RelayOff( REL_PH_MINUS );
	// ������ ��������� �����
	vTaskDelay( 200 );
	if( IsCurrent_PH_MINUS() || IsCurrent_PH_PLUS() )
	{
		// ���� ���� ��� ��� ����������� ����
		isError = true;
	}
	else
	{
		// �������� ���� �������� ����������
		Reg_RelayOn( REL_PH_MINUS );
		// ������ ��������� �����
		vTaskDelay( 200 );
		if( !IsCurrent_PH_MINUS() )
		{
			// ��� ���� � ���� �������� ��� ���������� ����
			// �������� �� ��������� � ��������� �������� ���������
			// ����� �������� �������� ������ ���� �� �������, ���������
			Reg_RelayOff( REL_PH_MINUS );
			Reg_RelayOn( REL_PH_PLUS );
			// ������ ��������� �����
			vTaskDelay( 200 );
			if( !IsCurrent_PH_PLUS() )
			{
				// ��� ���� � ���� �������� � ��������� ����� �� ������ ����, ������
				isError = true;
			}
		}
		else
		{
			// ���� ��� � ���� ��������
			Reg_RelayOff( REL_PH_MINUS );
		}
	}
	
	return isError;
}
*/

/*******************************************************
	������� ��������� ������� ���������
********************************************************/
/*
bool Reg_ToOpen( void )
{
	// !!! ��������� ��������
	//return true;
	
	bool isOpenOk = false;
	
	int openTime;
	
	if( !Reg_IsError() )
	{
		// ����� ������ �� ��������� "��������� �������"
		Reg_RelayOn( REL_PH_PLUS );
		// ���� ������������ ��������� ��������
		openTime = 0;
		
		// ���� ���������� ���� � ���� ������
		for( ;; )
		{
			vTaskDelay( 1000 );
			openTime++;

			if( !IsCurrent_PH_PLUS() ) 
			{
				// ��� ���� � ���� ������, �������� ��������, ��������� ������ ���������
				isOpenOk = true;
				break;
			}
			
			if( openTime > FULL_MOVE_TIME_SEC )
			{
				// ����� ���� ������ �������������� � ���������� ��� ����� ����������
				// ���� �������� ����������, ���� ����� �� ��������
				break;
			}
		
		}

		Reg_RelayOff( REL_PH_PLUS );
	}
	
	return isOpenOk;
}
*/
