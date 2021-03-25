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

//--- CONSTANTS ------------------
const TRelDesc Relay[] = {
	{PORT_REL_CLOSE, PIN_REL_CLOSE},
	{PORT_REL_OPEN, PIN_REL_OPEN},
};

const float K_INTEGRAL_DEFAULT = 0.2;
const float K_DIFF_DEFAULT = 0.2;
const float K_PROP_DEFAULT = 2.0;
const uint16_t FULL_MOVE_TIME_SEC_DEFAULT = 20;
const uint16_t MIN_FULL_MOVE_TIME_SEC = 5;		// ����������� ����� ���� ���������� � ��������
const uint16_t MAX_FULL_MOVE_TIME_SEC = 120;	// ������������ ����� ���� ���������� � ��������

//--- GLOBAL VARIABLES -----------
extern float g_Sensor_PH;					// ������� �������� PH � ��������
extern float g_Setup_PH;					// �������� ������������� �������� PH

extern bool g_isNoWater;
extern bool g_isErrRegulator;
extern bool g_isErrTimeoutSetupPh;
extern bool g_isErrSensors;

float g_K_INTEGRAL;
float g_K_DIFF;
float g_K_PROP;

uint16_t MAX_OUT_OF_WATER_SEC;		// ����. ������������ ��������� ���� ��� ������ (���.)
uint16_t MAX_TIME_ERROR_PH_SEC;		// ����. ������������ ������ ��������� PH (���.)
uint16_t FULL_MOVE_TIME_SEC;			// ������ ����� ���� ���������� � ��������

//--- FUNCTIONS ------------------
extern int ReadWorkMode( uint16_t idx );

/*******************************************************
�������		: ������������� ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Reg_Init(void)
{
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE );
	
	for( int i=0; i<2; i++ )
	{
		// ������� ������ ��� ���������� �� ����� � �������� �����������
		GPIO_PinConfigure( Relay[i].GPIOx, Relay[i].pin, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
		
		// ���������� ����������
		GPIO_PinWrite( Relay[i].GPIOx, Relay[i].pin, REL_OFF );
	}

	// ������� ������ �� ���� � ��������� � VCC ��� ������� ��������
	GPIO_PinConfigure( PORT_CURRENT_CLOSE, PIN_CURRENT_CLOSE, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	GPIO_PinConfigure( PORT_CURRENT_OPEN, PIN_CURRENT_OPEN, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );

	// ������� ������� ���� ������ � ������� �� �����
	GPIO_PinConfigure( PORT_PUMP, PIN_PUMP, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
	// ���������� ������ � �������
	GPIO_PinWrite( PORT_PUMP, PIN_PUMP, 0 );
	
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

	if( !FM24_ReadWords( EEADR_FULL_MOVE_TIME_SEC, &FULL_MOVE_TIME_SEC, 1 ) || FULL_MOVE_TIME_SEC == 0 )
	{
		FULL_MOVE_TIME_SEC = FULL_MOVE_TIME_SEC_DEFAULT;
		FM24_WriteWords( EEADR_FULL_MOVE_TIME_SEC, &FULL_MOVE_TIME_SEC, 1 );
	}
}

/*******************************************************
	������ �������� PID ����������
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
	���������� ��������� ��������� ���� ��������
********************************************************/
bool IsCurrent_CLOSE( void )
{
	bool isOk;
	isOk = ( GPIO_PinRead( PORT_CURRENT_CLOSE, PIN_CURRENT_CLOSE ) == 0 );
	
	return isOk;
}

/*******************************************************
	���������� ��������� ��������� ���� ��������
********************************************************/
bool IsCurrent_OPEN( void )
{
	bool isOk;
	isOk = ( GPIO_PinRead( PORT_CURRENT_OPEN, PIN_CURRENT_OPEN ) == 0 );
	
	return isOk;
}

/*******************************************************
	���� ���� ������������� PH
********************************************************/
void regulator_cycle( float deltaTime )
{
	static float prevPh = 8;
	
	float errorPh, pidValue;
	
	errorPh = g_Sensor_PH - g_Setup_PH;
	
	pidValue = getPidValue( errorPh, deltaTime, prevPh );
	
	prevPh = g_Sensor_PH;
	

	if( ( errorPh < 0.1 && errorPh > -0.1 ) && 
		( pidValue < 0.2 && pidValue > -0.2 ))
	{
		// ����� �� � �������� �������, ������������� ���������
		Reg_RelayAllOff();
		return;
	}
	
	if( pidValue > 0.15 )
	{
		Reg_RelayOn( REL_CLOSE );
		Reg_RelayOff( REL_OPEN );
	}
	else if( pidValue < -0.15 )
	{
		Reg_RelayOff( REL_CLOSE );
		Reg_RelayOn( REL_OPEN );

	}
	else
	{
		Reg_RelayAllOff();
	}
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

void switchPUMP( uint8_t on )
{
	GPIO_PinWrite( PORT_PUMP, PIN_PUMP, on );
}

/*******************************************************
�����		: ������� ����� ����������
�������� 1	: �� ������������
�����. ����.: ����������� ����
********************************************************/
void Thread_Regulator( void *pvParameters )
{
	Reg_Init();

	const TickType_t CYCLETIME_MS = 500;			// �������� ����� ������� �������������
	
	TickType_t xLastWakeTime;
	int timeOutOfWater = 0;
	int timeOutErrorPhValue = 0;
	int openTime = 0;

	//vTaskDelay(3000);
	
	g_isErrRegulator = Reg_IsError();
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, CYCLETIME_MS );
		
		if( ReadWorkMode(0) != Mode_RegulatorPh )
		{
			// ��������� ��������� ���������, ��������� �����
			switchPUMP( 0 );
			Reg_RelayOff( REL_CLOSE );
			Reg_RelayOn( REL_OPEN );
			// ��������� ��������
			if( IsCurrent_OPEN() )
			{
				openTime += CYCLETIME_MS;
				if( openTime > (int)FULL_MOVE_TIME_SEC * 1000 )
				{
					g_isErrRegulator = true;
				}
			}
			continue;
		}
		
		// ��������� ������� ����
		if( IsWaterOk() )
		{
			g_isNoWater = false;
		}
		else
		{
			timeOutOfWater += CYCLETIME_MS;
			if( timeOutOfWater > (MAX_OUT_OF_WATER_SEC * 1000) )
			{
				g_isNoWater = true;
			}
		}
		
		// ��������� ������ �� �������� PH � ������� �� ������� �������� PH
		if( g_Sensor_PH < 0 )
		{
			g_isErrSensors = true;
		}
		
		if( !g_isNoWater && !g_isErrSensors && !g_isErrTimeoutSetupPh && !g_isErrRegulator )
		{
			openTime = 0;

			// ��� ������� ���� � ��������� ������ - ����������
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
		else 
		{
			// ��������� ��������� ���������, ��������� �����
			switchPUMP( 0 );
			Reg_RelayOff( REL_CLOSE );
			Reg_RelayOn( REL_OPEN );
			// ��������� ��������
			if( IsCurrent_OPEN() )
			{
				if( !g_isErrRegulator )
				{
					openTime += CYCLETIME_MS;
					if( openTime > (int)FULL_MOVE_TIME_SEC * 1000 )
					{
						g_isErrRegulator = true;
					}
				}
				else
				{
					openTime = 0;
				}
			}
		}
	}
}


/*******************************************************
�������		: �������� ���� �� ��� �������
�������� 1	: ������ ����
�����. ����.: ���
********************************************************/
void Reg_RelayOn(uint8_t indx)
{
	if( indx > 1 )
		return;
	
	GPIO_PinWrite( Relay[indx].GPIOx, Relay[indx].pin, REL_ON );
}

/*******************************************************
�������		: ��������� ���� �� ��� �������
�������� 1	: ������ ����
�����. ����.: ���
********************************************************/
void Reg_RelayOff(uint8_t indx)
{
	if( indx > 1 )
		return;
	
	GPIO_PinWrite( Relay[indx].GPIOx, Relay[indx].pin, REL_OFF );
}

/*******************************************************
�������		: ��������� ��� ����
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Reg_RelayAllOff(void)
{
	GPIO_PinWrite( Relay[0].GPIOx, Relay[0].pin, REL_OFF );
	GPIO_PinWrite( Relay[1].GPIOx, Relay[1].pin, REL_OFF );
}

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
int Reg_Read_FULL_MOVE_TIME_SEC( uint16_t idx )
{
	int ivalue = FULL_MOVE_TIME_SEC; 
	
	return ivalue;
}

/*******************************************************
	������ ������� ���� ��������� PH
********************************************************/
bool Reg_Write_FULL_MOVE_TIME_SEC( uint16_t idx, uint16_t val )
{
	if( val < MIN_FULL_MOVE_TIME_SEC || val > MAX_FULL_MOVE_TIME_SEC )
		return false;
	
	// ��������� � EEPROM
	bool isWrited = FM24_WriteWords( EEADR_FULL_MOVE_TIME_SEC, &val, 1 );

	if( isWrited )
	{
		FULL_MOVE_TIME_SEC = val;
	}
	
	return isWrited;
}

/*******************************************************
	�������� ����������������� ��������� PH
********************************************************/
bool Reg_IsError( void )
{
	bool error = false;
	int openTime;
	
	// ��������� �����
	switchPUMP( 0 );
	
	// �������� �������� ����������
	Reg_RelayOn( REL_CLOSE );
	Reg_RelayOff( REL_OPEN );
	
	// ��������� �������� ��������
	openTime = 0;
	
	// ���� ������������ ��������� ����������, ����� ������ ��� � ���� ������
	do
	{
		vTaskDelay( 1000 );
		openTime++;
		
		if( openTime > FULL_MOVE_TIME_SEC )
		{
			// ����� ���� ������ �������������� � ���������� ��� ����� ����������
			// ���� �������� ����������, ���� ����� �� ��������
			error = true;
			break;
		}
		
	} while( IsCurrent_CLOSE() );
	
	if( !error )
	{
		// ���� ������ ���� ���, �� �� ��������� � ��������� - �������.
		// ��������� ��������
		
		Reg_RelayOff( REL_CLOSE );
		Reg_RelayOn( REL_OPEN );
		
		// ��������� �������� ��������
		vTaskDelay( 1000 );
		
		// ����� �� ��������� � ������ ���� �������� ����������
		// ������� ������ ���� ��� � ���� ���������
		if( !IsCurrent_OPEN() )
		{
			// ��� ���� � ���� ��������� ����������
			error = true;
		}
		
		if( !error )
		{
			openTime = 0;
			// ���� ������������ ��������� ����������, ����� ������ ��� � ���� ������
			do
			{
				vTaskDelay( 1000 );
				openTime++;
				
				if( openTime > FULL_MOVE_TIME_SEC )
				{
					// ����� ���� ������ �������������� � ���������� ��� ����� ����������
					// ���� �������� ����������, ���� ����� �� ��������
					error = true;
					break;
				}
				
			} while( IsCurrent_OPEN() );
		}
	}
	
	return error;
}