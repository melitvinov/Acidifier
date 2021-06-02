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

//#define WRITE_LOG

#ifdef WRITE_LOG
	char szLog[250];
#endif
//--- CONSTANTS ------------------
/*
const TRelDesc Relay[] = {
	{PORT_REL_PH_MINUS, PIN_REL_PH_MINUS},
	{PORT_REL_PH_PLUS, PIN_REL_PH_PLUS},
};
*/

const float K_INTEGRAL_DEFAULT = 0.01;
const float K_DIFF_DEFAULT = 1.0;
const float K_PROP_DEFAULT = 5.0;
const uint16_t REG_CYCLETIME_SEC_DEFAULT = 4;
const uint16_t MIN_REG_CYCLETIME_SEC = 3;		// ����������� ������ ���������� � ��������
const uint16_t MAX_REG_CYCLETIME_SEC = 20;		// ������������ ������ ���������� � ��������

//--- GLOBAL VARIABLES -----------
extern float g_Sensor_PH;					// ������� �������� PH � ��������
extern float g_Setup_PH;					// �������� ������������� �������� PH

extern bool g_isNoWater;
//extern bool g_isErrRegulator;
extern bool g_isErrTimeoutSetupPh;
extern bool g_isErrSensors;

float g_K_INTEGRAL;
float g_K_DIFF;
float g_K_PROP;

uint16_t MAX_OUT_OF_WATER_SEC;		// ����. ������������ ��������� ���� ��� ������ (���.)
uint16_t MAX_TIME_ERROR_PH_SEC;		// ����. ������������ ������ ��������� PH (���.)
uint16_t REG_CYCLETIME_SEC;			// ������ ������������� (���)

int g_RegPercentOn;				// ������� (��������� �������� �������� �������� ���������� � %)

//--- FUNCTIONS ------------------
extern void switchKLAPAN( uint8_t on );
extern int ReadWorkMode( uint16_t idx );
//void Reg_RelayOn(uint8_t indx);
//void Reg_RelayOff(uint8_t indx);
//void Reg_RelayAllOff(void);
//bool Reg_ToOpen( void );
//bool Reg_IsError( void );

/*******************************************************
�������		: ������������� ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Reg_Init(void)
{
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE );
/*	
	for( int i=0; i<2; i++ )
	{
		// ������� ������ �� ����� � �������� ����������� ��� ���������� ����
		GPIO_PinConfigure( Relay[i].GPIOx, Relay[i].pin, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
		
		// ���������� ����
		GPIO_PinWrite( Relay[i].GPIOx, Relay[i].pin, REL_OFF );
	}
	
	// ������� ������ �� ���� � ��������� � VCC ��� ������� ��������
	GPIO_PinConfigure( PORT_CURRENT_PH_MINUS, PIN_CURRENT_PH_MINUS, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	GPIO_PinConfigure( PORT_CURRENT_PH_PLUS, PIN_CURRENT_PH_PLUS, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
*/

	// ������� ������ �� ���� � ��������� � VCC ��� ������� ����
	GPIO_PinConfigure( PORT_SENS_WATER, PIN_SENS_WATER, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	
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

	if( !FM24_ReadWords( EEADR_REG_CYCLETIME_SEC, &REG_CYCLETIME_SEC, 1 ) || REG_CYCLETIME_SEC == 0 )
	{
		REG_CYCLETIME_SEC = REG_CYCLETIME_SEC_DEFAULT;
		FM24_WriteWords( EEADR_REG_CYCLETIME_SEC, &REG_CYCLETIME_SEC, 1 );
	}
	
	g_RegPercentOn = 50;
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
/*
bool IsCurrent_PH_MINUS( void )
{
	bool isOk;
	isOk = ( GPIO_PinRead( PORT_CURRENT_PH_MINUS, PIN_CURRENT_PH_MINUS ) == 0 );
	
	return isOk;
}
*/
/*******************************************************
	���������� ��������� ��������� ���� ��������
********************************************************/
/*
bool IsCurrent_PH_PLUS( void )
{
	bool isOk;
	isOk = ( GPIO_PinRead( PORT_CURRENT_PH_PLUS, PIN_CURRENT_PH_PLUS ) == 0 );
	
	return isOk;
}
*/
/*******************************************************
	���� ���� ������������� PH
********************************************************/
void regulator_cycle( float deltaTime )
{
#ifdef WRITE_LOG
	static int iNumCycle = 0;
#endif
	static bool isFirstPid = true;
	static float prevPh;
	float errorPh, pidValue;
	
	if( isFirstPid )
	{
		isFirstPid = false;
		prevPh = g_Sensor_PH;
	}

	errorPh = g_Setup_PH - g_Sensor_PH;
	pidValue = getPidValue( errorPh, deltaTime, prevPh );
	prevPh = g_Sensor_PH;

	int i_delta_percent = roundf( pidValue ) * (-1);
	
	if( i_delta_percent > 10 )
		i_delta_percent = 10;
	else if( i_delta_percent < -10 )
		i_delta_percent = -10;

	g_RegPercentOn += i_delta_percent; 
	if( g_RegPercentOn > 95 )
		g_RegPercentOn = 95;
	else if( g_RegPercentOn < 0 )
		g_RegPercentOn = 0;
	
	TickType_t xStartImpulseTime, impulseTime_ms;
	
	impulseTime_ms = (REG_CYCLETIME_SEC * 10) * g_RegPercentOn;
	if( impulseTime_ms < 50 )
		impulseTime_ms = 0;
	if( impulseTime_ms > 0 )
	{
		xStartImpulseTime = xTaskGetTickCount();
		switchKLAPAN( 1 );
		vTaskDelayUntil( &xStartImpulseTime, impulseTime_ms );
	}
	switchKLAPAN(0);

#ifdef WRITE_LOG
	sprintf( szLog, "%i\tPID\t%.2f\tdelta_perc\t%i\topen_perc\t%i\timpulseTime_ms\t%.3f\tSetup\t%.1f\tSensor\t%.1f\r\n", ++iNumCycle, pidValue, i_delta_percent, g_RegPercentOn, (((float)impulseTime_ms)/1000.0),
		g_Setup_PH, g_Sensor_PH	);
	RS485_SendString( szLog );
#endif
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
�����		: ������� ����� ����������
�������� 1	: �� ������������
�����. ����.: ����������� ����
********************************************************/
void Thread_Regulator( void *pvParameters )
{
	Reg_Init();

	TickType_t xLastWakeTime;
	int timeOutOfWater = 0;
	int timeOutErrorPhValue = 0;
	int timeOutErrorPhSensors = 0;
//	bool prevIsRegError = false;
	bool IsPhSensorsTooDiff;

	vTaskDelay( 1000 );
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, (REG_CYCLETIME_SEC * 1000) );
		
//		if( prevIsRegError && !g_isErrRegulator )
//		{
//			// ������ ����� ������, ���� ��������� ���������
//			g_isErrRegulator = Reg_IsError();
//		}
//		prevIsRegError = g_isErrRegulator;
		
		if( ReadWorkMode(0) != Mode_RegulatorPh )
		{
			/*
			// ������ ������������ PH (��������� ��������� ���������)
			Reg_RelayOff( REL_PH_MINUS );
			Reg_RelayOn( REL_PH_PLUS );
			*/
			
			switchKLAPAN(0); 
			timeOutErrorPhSensors = 0;
			timeOutErrorPhValue = 0;
			timeOutOfWater = 0;
			continue;
		}
		
		// ��������� ������� ����
		if( IsWaterOk() )
		{
			g_isNoWater = false;
			timeOutOfWater = 0;
		}
		else if( !g_isNoWater )
		{
			timeOutOfWater += (REG_CYCLETIME_SEC * 1000);
			if( timeOutOfWater > (MAX_OUT_OF_WATER_SEC * 1000) )
			{
				g_isNoWater = true;
				
//				if( !g_isErrRegulator )
//				{
//					Reg_RelayOn( REL_PH_PLUS );
//					vTaskDelay( 200 );
//					if( IsCurrent_PH_PLUS() )
//						g_isErrRegulator = !Reg_ToOpen();
//					Reg_RelayOff( REL_PH_PLUS );
//				}
			}
		}
		
		// ��������� ������ �� �������� PH � ������� �� ������� �������� PH
		g_Sensor_PH = AInp_GetSystemPh( &IsPhSensorsTooDiff );
		if( g_Sensor_PH < 0 || IsPhSensorsTooDiff )
		{
			if( !g_isErrSensors )
			{
				timeOutErrorPhSensors  += (REG_CYCLETIME_SEC * 1000);
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
		
		if( !g_isNoWater && !g_isErrSensors && !g_isErrTimeoutSetupPh /* && !g_isErrRegulator */ )
		{
			// ��� ������� ���� � ��������� ������ - ����������
			regulator_cycle( REG_CYCLETIME_SEC );
			
			if( fabs( g_Sensor_PH - g_Setup_PH ) > 0.5 )
			{
				timeOutErrorPhValue += (REG_CYCLETIME_SEC * 1000);
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
		else /*if( !g_isErrRegulator )*/
		{
			// ��������� ��������� ���������
//			Reg_RelayOn( REL_PH_PLUS );
//			vTaskDelay( 200 );
//			if( IsCurrent_PH_PLUS() ) 
//				g_isErrRegulator = Reg_ToOpen();
			switchKLAPAN(0);
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
