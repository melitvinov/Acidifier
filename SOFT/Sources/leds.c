/***********************************************************************************
* ������������ ������	: ���������� ���������� ������������
*-----------------------------------------------------------------------------------
* ������				: 1.0
* �����					: ��������
* ���� ��������			: 23.05.2018
************************************************************************************/

//--- INCLUDES -------------------
#include "leds.h"

//--- CONSTANTS ------------------
const TLedDesc LedArray[] = {
	{PORT_LED_WORK_OK, PIN_LED_WORK_OK},
	{PORT_LED_NO_WATER, PIN_LED_NO_WATER},
	{PORT_LED_ERR_SETUP_PH_TIMEOUT, PIN_LED_ERR_SETUP_PH_TIMEOUT},
	{PORT_LED_ERR_SENSORS, PIN_LED_ERR_SENSORS},
	{PORT_LED_TAR_P1, PIN_LED_TAR_P1},
	{PORT_LED_TAR_P2, PIN_LED_TAR_P2},
	{PORT_LED_VALVE, PIN_LED_VALVE},
};

const int LEDS_COUNT = sizeof( LedArray ) / sizeof( TLedDesc );

//--- GLOBAL VARIABLES -----------

//--- FUNCTIONS ------------------

/*******************************************************
�������		: ������������� ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Leds_init(void)
{
	for( int i=0; i<LEDS_COUNT; i++ )
	{
		// ������� ������ ��� ���������� �� ����� � �������� �����������
		GPIO_PinConfigure( LedArray[i].GPIOx, LedArray[i].pin, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
		
		// ���������� ����������
		GPIO_PinWrite( LedArray[i].GPIOx, LedArray[i].pin, TO_OFF );
	}

	// ��������� ���������
	GPIO_PinConfigure( PORT_LED_SYS, PIN_LED_SYS, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	
	// ���������� ����������
	LedSYS(0);
}

/*******************************************************
�������		: ���������/���������� ����������
�������� 1	: ������ ����������
�����. ����.: ���
********************************************************/
void Led_OnOff(uint8_t indx, uint8_t state)
{
	if( indx >= LEDS_COUNT )
		return;
	
	GPIO_PinWrite( LedArray[indx].GPIOx, LedArray[indx].pin, state );
}

void LedSYS( uint8_t state )
{
	bool bstate = state;
	
	GPIO_PinWrite( PORT_LED_SYS, PIN_LED_SYS, !bstate );
}

/*******************************************************
�������		: ��������� ����������
�������� 1	: ����� ����������
�����. ����.: ���
********************************************************/
/*
void Led_On(uint8_t indx)
{
	if( indx >= LEDS_COUNT )
		return;
	
	GPIO_PinWrite( LedArray[indx].GPIOx, LedArray[indx].pin, TO_ON );
}
*/
/*******************************************************
�������		: ���������� ����������
�������� 1	: ����� ����������
�����. ����.: ���
********************************************************/
/*
void Led_Off(uint8_t indx)
{
	if( indx >= LEDS_COUNT )
		return;
	
	GPIO_PinWrite( LedArray[indx].GPIOx, LedArray[indx].pin, TO_OFF );
}
*/

/*******************************************************
�������		: ���������� ��������� ����������
�������� 1	: ����� ����������
�����. ����.: ��������� ����������
********************************************************/
bool Led_IsOn(uint8_t indx)
{
	if( indx >= LEDS_COUNT )
		return false;
	
    uint32_t pinState = GPIO_PinRead( LedArray[indx].GPIOx, LedArray[indx].pin );
	
	bool isOn = ( pinState == TO_ON );

    return isOn;
}

/*******************************************************
�������		: ������������ ����������
�������� 1	: ����� ����������
�����. ����.: ���
********************************************************/
void Led_Switch(uint8_t indx)
{
    Led_OnOff(indx, !Led_IsOn(indx) );
}

/*******************************************************
�������		: ��������� ���� �����������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Leds_OnAll(void)
{
	for( int i=0; i<LEDS_COUNT; i++ )
	{
		GPIO_PinWrite( LedArray[i].GPIOx, LedArray[i].pin, TO_ON );
	}
}

/*******************************************************
�������		: ���������� ���� �����������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Leds_OffAll(void)
{
	for( int i=0; i<LEDS_COUNT; i++ )
	{
		GPIO_PinWrite( LedArray[i].GPIOx, LedArray[i].pin, TO_OFF );
	}
}
