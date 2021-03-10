/***********************************************************************************
* ������������ ������	: ���������� ����������� PH
*-----------------------------------------------------------------------------------
* ������				: 1.0
* �����					: ��������
* ���� ��������			: 10.03.2021
************************************************************************************/

//--- INCLUDES -------------------
#include "Regulator.h"

//--- CONSTANTS ------------------
const TRelDesc Relay[] = {
	{PORT_REL_PLUS, PIN_REL_PLUS},
	{PORT_REL_MINUS, PIN_REL_MINUS},
};

//--- GLOBAL VARIABLES -----------

//--- FUNCTIONS ------------------
extern int ReadWorkMode( uint16_t idx );
extern bool IsWaterOk( void );

/*******************************************************
�������		: ������������� ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Reg_Init(void)
{
	for( int i=0; i<2; i++ )
	{
		// ������� ������ ��� ���������� �� ����� � �������� �����������
		GPIO_PinConfigure( Relay[i].GPIOx, Relay[i].pin, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
		
		// ���������� ����������
		GPIO_PinWrite( Relay[i].GPIOx, Relay[i].pin, REL_OFF );
	}
}

void regulator_cycle( void )
{
	vTaskDelay(10);
}

/*******************************************************
�����		: ������� ����� ����������
�������� 1	: �� ������������
�����. ����.: ����������� ����
********************************************************/
void Thread_Regulator( void *pvParameters )
{
	Reg_Init();
	
	for(;;)
	{
		vTaskDelay(50);
		
		if( ReadWorkMode(0) == Mode_RegulatorPh )
		{
			if( IsWaterOk() )
			{
				regulator_cycle();
			}
		}
		else
		{
			Reg_RelayAllOff();
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

