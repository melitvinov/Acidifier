/***********************************************************************************
* ������������ : ������ "Acidifier"
*-----------------------------------------------------------------------------------
* ������				: 1.0
* �����					: ��������
* ���� ��������			: 09.03.2021
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
const uint16_t UINT_VERSION = 201;

const uint16_t DEFAULT_SETUP_PH = 70;
const float MIN_VALUE_SETUP_PH = 40;
const float MAX_VALUE_SETUP_PH = 100;
const uint16_t WDT_TIME_SEC = 10; 		// ����� � ���. �� ������������ ������� ��� ������������ ��������

//--- GLOBAL VARIABLES -----------
uint8_t g_DeviceAddr = 0;	// ������� ����� ���������� �� ���� RS-485

EWorkMode g_WorkMode;				// ������� ����� ������ / �����������
bool g_isNoWater;
bool g_isErrTimeoutSetupPh = false;
bool g_isErrSensors = false;

float g_Sensor_PH;					// ������� �������� PH � ��������
float g_Setup_PH;					// �������� ������������� �������� PH

bool g_isBtnPlusClick;
bool g_isBtnMinusClick;
bool g_isDblBtnPressed;
bool g_isEscPressed;
bool g_isEscClick;

int g_WaterCounter;

// WDT flags
uint8_t g_WdtCheckAddr = 0;
uint8_t g_WdtWork = 0;
uint8_t g_WdtButtons = 0;
uint8_t g_WdtModbus = 0;
uint8_t g_WdtAInp = 0;
uint8_t g_WdtLeds = 0;
uint8_t g_WdtRegulator = 0;
uint8_t g_WdtKlapan = 0;
//uint8_t g_WdtPid = 0;

//--- FUNCTIONS ------------------

void Thread_WORK( void *pvParameters );


void initWdt( void )
{
	const uint16_t WDT_GEN_FREQ = 40000; 	// ������� ���������� Wdt � ������
	const uint16_t RELOAD_WDT_VALUE = WDT_TIME_SEC * ( WDT_GEN_FREQ / 256 );
	
	// �������� LSI
	RCC_LSICmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
	// ����������� ������ � ��������� IWDG
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	// ������������� ������������
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	// �������� ��� ������������
	IWDG_SetReload(RELOAD_WDT_VALUE);	
	// ������������ ��������
	IWDG_ReloadCounter();
	// LSI ������ ���� �������
	IWDG_Enable();
}
void resetWdt( void )
{
	g_WdtCheckAddr = 0;
	g_WdtWork = 0;
	g_WdtButtons = 0;
	g_WdtModbus = 0;
	g_WdtAInp = 0;
	g_WdtLeds = 0;
	g_WdtRegulator = 0;
	g_WdtKlapan = 0;
	//g_WdtPid = 0;
	
	IWDG_ReloadCounter();
}
void ThreadCheckWdt( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = (WDT_TIME_SEC - 2) * 1000;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	//bool isOk;
	for(;;)
	{
         // Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		
		// �������� ���� �������
		if( !g_WdtCheckAddr ) goto WdtWaitReset;
		if( !g_WdtWork ) goto WdtWaitReset;
		if( !g_WdtButtons ) goto WdtWaitReset;
		if( !g_WdtModbus ) goto WdtWaitReset;
		if( !g_WdtAInp ) goto WdtWaitReset;
		if( !g_WdtLeds ) goto WdtWaitReset;
		if( !g_WdtRegulator ) goto WdtWaitReset;
		if( !g_WdtKlapan ) goto WdtWaitReset;
		//if( !g_WdtPid ) goto WdtWaitReset;


//		isOk = g_WdtCheckAddr && g_WdtWork && g_WdtButtons && g_WdtModbus && g_WdtAInp && g_WdtLeds && g_WdtRegulator && g_WdtKlapan && g_WdtPid;
//		if( isOk )
//		{
			// ���� ��� ������ ��������, �� ���������� ���������� ������
			resetWdt();
//		}
//		else
//		{
//			// ���� ���, �� ���� ������
//			Leds_OnAll(); 
//			for(;;)
//			{
//				vTaskDelay(100);
//			}
//		}
	}
	
	WdtWaitReset:
			for(;;)
			{
				Leds_OnAll();
				vTaskDelay(100);
				Leds_OffAll();
				vTaskDelay(100);
			}
}

/*******************************************************
�������		: ���. / ����. �������
�������� 1	: ���
�����. ����.: ���
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
�������		: ��������� ������ ����������
�������� 1	: ���
�����. ����.: ����� ����������
********************************************************/
uint8_t GetDeviceAddress(void)
{
	return g_DeviceAddr;
}

/*******************************************************
�����		: ���������� ��������� ������ �� ���� RS-485
�������� 1	: �� ������������
�����. ����.: ����������� ����
********************************************************/
void ThreadCheckAddrChange( void *pvParameters )
{
	for(;;)
	{
		g_WdtCheckAddr = 1;
		
		uint16_t addr = ADRSW_GetAdr();
		if( addr != g_DeviceAddr )
		{
			vTaskDelay(2000);
			g_WdtCheckAddr = 1;
			
			if( addr == ADRSW_GetAdr() )
			{
				// ����� ���������� ���������, ������
				g_DeviceAddr = addr;
				for( int i=0; i<addr; i++ )
				{
					LedSYS( 1 );
					vTaskDelay(200);
					g_WdtCheckAddr = 1;
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
�����		: ������� ����� ����������
�������� 1	: �� ������������
�����. ����.: ����������� ����
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
		g_WdtButtons = 1;
		
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
					// ��� ���������� ������ ������������ ������ ������� ���� ������
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
					// ��� ���������� ������ ������������ ������ ������� ���� ������
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
�������		: ������������� ����������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Initialize()
{
	Leds_init();
	ADRSW_init();
	Buttons_init();
	initWdt();
	
	g_DeviceAddr = 0;
}

/*******************************************************
�������		: ����� ������ ����������
�������� 1	: ���
�����. ����.: ����������� ����
********************************************************/
int main(void)
{
	Initialize();
	
	xTaskCreate( ThreadCheckAddrChange, (const char*)"ADDRESS", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTaskCreate( AInp_Thread, (const char*)"ANALOG", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTaskCreate( MBUS_Thread, (const char*)"Modbus", 0x1000,	( void * ) NULL, ( tskIDLE_PRIORITY + 2 ), NULL);

	xTaskCreate( Thread_Leds_Dig, (const char*)"LedsDig", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	
	xTaskCreate( Thread_Buttons, (const char*)"BTN", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTaskCreate( Thread_WORK, (const char*)"WORK", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	xTaskCreate( Thread_Regulator, (const char*)"Regulator", 512,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	
	xTaskCreate( ThreadCheckWdt, (const char*)"Wdt", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	
	/* Start the scheduler. */
	vTaskStartScheduler();

}

/*******************************************************
	������ ������ �������� PH � EEPROM
********************************************************/
bool WriteSetupPhValue( uint16_t idx, uint16_t val )
{
	if(( val < MIN_VALUE_SETUP_PH ) || ( val > MAX_VALUE_SETUP_PH ))
		return false;
	
	if( FM24_WriteBytes( EEADR_SETUP_PH, (uint8_t*) &val, sizeof( uint16_t ) ) )
	{
		// ���������� � ���������� ����������
		g_Setup_PH = val;
		g_Setup_PH /= 10.0;
		
		return true;
	}
	return false;

}
/*******************************************************
	������ �������� ������� PH �� EEPROM
********************************************************/
int ReadSetupPhValue( uint16_t idx )
{
	uint16_t uiValue;
	
	if( !FM24_ReadBytes( EEADR_SETUP_PH, (uint8_t*) &uiValue, sizeof( uint16_t ) ) )
		return -1;
	
	if(( uiValue < MIN_VALUE_SETUP_PH ) || ( uiValue > MAX_VALUE_SETUP_PH ))
	{
		// �������� �������������� PH ����� ��� �������� ���������
		// ������������� �������� �� ���������
		uiValue = DEFAULT_SETUP_PH;
		WriteSetupPhValue( 0, uiValue );
	}

	g_Setup_PH = uiValue;
	g_Setup_PH /= 10.0;
	return uiValue;
}

/*******************************************************
	��������� ������ �������� PH
********************************************************/
bool SetupPhValue( float value )
{
	// ��� ���� ��������� ����� �������� ��������� PH
	value *= 10.0;
	uint16_t uiValue  = roundf( value );
	return WriteSetupPhValue( 0, uiValue );
}

// ����� ������������ ���� ������ � ���������
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

// ����� ���� ������
void clearAllErrors( void )
{
	// ����������� ������ ������
	display_clearErrors();
}

/*******************************************************
�����		: ������� ����� ����������
�������� 1	: �� ������������
�����. ����.: ����������� ����
********************************************************/
void Thread_WORK( void *pvParameters )
{
	float ph1, ph2;
	bool stopWork, IsPhSensorsTooDiff;
	
	// ������������� ������ �������
	GPIO_PinConfigure( PORT_RELAY_ALARM, PIN_RELAY_ALARM, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );

	ReadSetupPhValue(0);
	
	g_WorkMode = Mode_RegulatorPh;
	g_isNoWater = true;
	
	
	for(;;)
	{
		vTaskDelay(50);
		
		g_WdtWork = 1;
		
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
					// ����� ������� ������
					clearAllButtons();
					
					if( g_isNoWater && !IsPumpTurnON() )
						g_WorkMode = Mode_Calibrating_PH1;
				}
				else if( g_isBtnPlusClick )
				{
					// ����� ������� ������
					clearAllButtons();
					
					if( g_Setup_PH + 0.1 < 10 )
						SetupPhValue( g_Setup_PH + 0.1 );
				}
				else if( g_isBtnMinusClick )
				{
					// ����� ������� ������
					clearAllButtons();
					
					if( g_Setup_PH - 0.1 > 0 )
						SetupPhValue( g_Setup_PH - 0.1 );
				}
				else if( g_isEscPressed )
				{
					// ����� ������� ������
					clearAllButtons();
					clearAllErrors();

					g_WorkMode = Mode_RegulatorPh;
					g_isNoWater = true;
					g_isErrTimeoutSetupPh = false;
					g_isErrSensors = false;
					
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
					// ������������� ���������� �����

					// ����� ������� ������
					clearAllButtons();
					
					// ������ ����������
					AInp_WriteAdcTar1( 0, AInp_ReadAdcValue( 0 ) );
					AInp_WriteAdcTar1( 1, AInp_ReadAdcValue( 1 ) );

					// ����� �� ������� ����������� �������� ��������
					ph1 = AInp_GetFloatSensorPh(0);
					ph2 = AInp_GetFloatSensorPh(1);
					LcdDig_PrintPH( ph1, SideLEFT, false );
					LcdDig_PrintPH( ph2, SideRIGHT, false );
					vTaskDelay(100);

					// ����������� �������������
					LcdDig_DispBlinkOn( SideLEFT | SideRIGHT );
					vTaskDelay( 1500 );
					LcdDig_DispBlinkOff( SideLEFT | SideRIGHT );
					
					g_WorkMode = Mode_Calibrating_PH2;
				}
				else if( g_isBtnPlusClick )
				{
					// ������� ���������� �����

					// ����� ������� ������
					clearAllButtons();
					g_WorkMode = Mode_Calibrating_PH2;
				}
				else if( g_isEscClick )
				{
					// ������ ���������� �����

					// ����� ������� ������
					clearAllButtons();
					
					g_WorkMode = Mode_RegulatorPh;
					g_isNoWater = true;
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
					// ������������� ���������� �����

					// ����� ������� ������
					clearAllButtons();
					
					// ������ ����������
					AInp_WriteAdcTar2( 0, AInp_ReadAdcValue( 0 ) );
					AInp_WriteAdcTar2( 1, AInp_ReadAdcValue( 1 ) );

					// ����� �� ������� ����������� �������� ��������
					ph1 = AInp_GetFloatSensorPh(0);
					ph2 = AInp_GetFloatSensorPh(1);
					LcdDig_PrintPH( ph1, SideLEFT, false );
					LcdDig_PrintPH( ph2, SideRIGHT, false );
					vTaskDelay(100);

					// ����������� �������������
					LcdDig_DispBlinkOn( SideLEFT | SideRIGHT );
					vTaskDelay( 1500 );
					LcdDig_DispBlinkOff( SideLEFT | SideRIGHT );
					
					g_WorkMode = Mode_RegulatorPh;
					g_isNoWater = true;
				}
				else if( g_isEscClick )
				{
					// ������ ���������� �����
					// ����� ������� ������
					clearAllButtons();
					
					g_WorkMode = Mode_RegulatorPh;
					g_isNoWater = true;
				}
				
				break;
			
			default:
				g_WorkMode = Mode_RegulatorPh;
				Regulator_START();
				break;
		}
	}
}
