/***********************************************************************************
* ������������ ������	: ���������� �������� ��������������� ������������
*-----------------------------------------------------------------------------------
* ������				: 1.0
* �����					: ��������
* ���� ��������			: 09.03.2021
************************************************************************************/

//--- INCLUDES -------------------
#include "leds-dig.h"
#include "math.h"

//--- CONSTANTS ------------------
uint8_t DIGIT[] = {
	0x3F,	// '0'
	0x06,	// '1'
	0x5B,	// '2'
	0x4F,	// '3'
	0x66,	// '4'
	0x6D,	// '5'
	0x7D,	// '6'
	0x07,	// '7'
	0x7F,	// '8'
	0x6F,	// '9'
	0x40,	// '-'
};

const int CLOCK_DELAY_TIME = 5;
const int LATCH_DELAY_TIME = 5;

//--- GLOBAL VARIABLES -----------

TLedDig g_LedDig[4];	// ������� ��������� ������� ���� �������


//--- FUNCTIONS ------------------
void delay_cycles( int num )
{
	while( num >= 0 )
	{
		num--;
	}
}

/*******************************************************
�������		: ��������� ������������ �����
�������� 1	: ���
�����. ����.: ���
********************************************************/
void LcdDig_SetDigit( uint8_t idx, TLedDig * value )
{
	if( idx < 4 && value )
	{
		memcpy( &(g_LedDig[idx]), value, sizeof( TLedDig ) );
	}
}
	
/*******************************************************
�������		: ������������� ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void LcdDig_init(void)
{
	// ������� ������� ��� ���������� �� �����
	GPIO_PinConfigure( PORT_LED_CLOCK, PIN_LED_CLOCK, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinConfigure( PORT_LED_LATCH, PIN_LED_LATCH, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinConfigure( PORT_LED_SERIAL, PIN_LED_SERIAL, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );

	GPIO_PinWrite( PORT_LED_CLOCK, PIN_LED_CLOCK, 0 );
	GPIO_PinWrite( PORT_LED_LATCH, PIN_LED_LATCH, 0 );
	GPIO_PinWrite( PORT_LED_SERIAL, PIN_LED_SERIAL, 0 );
}

void lcd_clock( void )
{
	delay_cycles(CLOCK_DELAY_TIME);
	GPIO_PinWrite( PORT_LED_CLOCK, PIN_LED_CLOCK, 1 );
	delay_cycles(CLOCK_DELAY_TIME);
	GPIO_PinWrite( PORT_LED_CLOCK, PIN_LED_CLOCK, 0 );
	delay_cycles(CLOCK_DELAY_TIME);
}

void lcd_latch( void )
{
	delay_cycles( LATCH_DELAY_TIME );
	GPIO_PinWrite( PORT_LED_LATCH, PIN_LED_LATCH, 1 );
	delay_cycles(LATCH_DELAY_TIME);
	GPIO_PinWrite( PORT_LED_LATCH, PIN_LED_LATCH, 0 );
	delay_cycles(LATCH_DELAY_TIME);
}

/*******************************************************
�������		: ��������� ������� � ������������ � �������������� ����������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void LcdDig_refresh( bool current_blinking_on )
{
	uint8_t dig;
	uint32_t shreg;
	
	vPortEnterCritical();
	
	if( !g_LedDig[3].isOn || (g_LedDig[3].isBlinking && !current_blinking_on) )
		dig = 0;
	else
	{
		dig = DIGIT[g_LedDig[3].value];
		if( g_LedDig[3].isPoint ) dig |= 0x80;
	}
	shreg = dig << 24;
	
	if( !g_LedDig[2].isOn || (g_LedDig[2].isBlinking && !current_blinking_on) )
		dig = 0;
	else
	{
		dig = DIGIT[g_LedDig[2].value];
		if( g_LedDig[2].isPoint ) dig |= 0x80;
	}
	shreg |= dig << 16;

	if( !g_LedDig[1].isOn || (g_LedDig[1].isBlinking && !current_blinking_on) )
		dig = 0;
	else
	{
		dig = DIGIT[g_LedDig[1].value];
		if( g_LedDig[1].isPoint ) dig |= 0x80;
	}
	shreg |= dig << 8;

	if( !g_LedDig[0].isOn || (g_LedDig[0].isBlinking && !current_blinking_on) )
		dig = 0;
	else
	{
		dig = DIGIT[g_LedDig[0].value];
		if( g_LedDig[0].isPoint ) dig |= 0x80;
	}
	shreg |= dig;
	
	// ��������� � ��������� �������� ���������� ��������
	for( int i=31; i>=0; i-- )
	{
		GPIO_PinWrite( PORT_LED_SERIAL, PIN_LED_SERIAL, GETBIT( shreg, i ) );
		//vTaskDelay(1);
		lcd_clock();
		//lcd_latch();
	}
	GPIO_PinWrite( PORT_LED_SERIAL, PIN_LED_SERIAL, 0 );
	delay_cycles(100);
	lcd_latch();
	
	vPortExitCritical();
	
}

void LcdDig_DispOff( void )
{
	for( int i=0; i<4; i++ )
		g_LedDig[0].isOn = false;
	LcdDig_refresh(false);
}

void LcdDig_ShowBegin( void )
{
	LcdDig_DispOff();
	vTaskDelay(500);
	
	TLedDig tdig;
	tdig.isPoint = true;
	tdig.isBlinking = false;
	tdig.isOn = true;

	tdig.value = 10;
	tdig.isPoint = false;
	for( int i=0; i<4; i++ )
	{
		LcdDig_SetDigit( i, &tdig );
	}
	LcdDig_refresh(true);
	vTaskDelay(500);

	for( int dig = 9; dig >= 0; dig-- )
	{
		tdig.value = dig;
		tdig.isPoint = true;
		for( int i=0; i<4; i++ )
		{
			LcdDig_SetDigit( i, &tdig );
		}
		LcdDig_refresh(true);
		vTaskDelay(250);
	}
	tdig.value = 10;
	tdig.isPoint = false;
	for( int i=0; i<4; i++ )
	{
		LcdDig_SetDigit( i, &tdig );
	}
	LcdDig_refresh(true);
	vTaskDelay(500);
}

/*******************************************************
�����		: ��������� ���������
�������� 1	: �� ������������
�����. ����.: ����������� ����
********************************************************/
void Thread_Leds_Dig( void *pvParameters )
{
	bool current_blinking_on = true;
	
	LcdDig_init();
	LcdDig_ShowBegin();
	
	for(;;)
	{
		LcdDig_refresh( current_blinking_on );
		
		current_blinking_on = !current_blinking_on;
		vTaskDelay(500);
	}
}

void LcdDig_PrintPH( float valuePH, ELcdSide side, bool isBlink )
{
	TLedDig ledDig;
	int startIndex = side == SideLEFT ? 0 : 2;
	
	if( valuePH < 0 )
	{
		ledDig.isBlinking = false;
		ledDig.isOn = true;
		ledDig.isPoint = false;
		ledDig.value = 10;
		LcdDig_SetDigit( startIndex, &ledDig );
		LcdDig_SetDigit( startIndex+1, &ledDig );
	}
	else
	{
		if( valuePH > 9.9 )
			valuePH = 9.9;
		
		int beforePoint = valuePH;
		int afterPoint = roundf( ((float)(valuePH - beforePoint)) * 10.0 );
		
		ledDig.isBlinking = isBlink;
		ledDig.isOn = true;
		ledDig.isPoint = true;
		ledDig.value = beforePoint;
		LcdDig_SetDigit( startIndex, &ledDig );
		
		ledDig.isPoint = false;
		ledDig.value = afterPoint;
		LcdDig_SetDigit( startIndex+1, &ledDig );
	}
}

void LcdDig_DispBlinkOff( uint8_t side )
{
	if( side & SideLEFT )
		for( int i=0; i<2; i++ )
		{
			g_LedDig[i].isBlinking = false;
		}
	if( side & SideRIGHT )
		for( int i=2; i<4; i++ )
		{
			g_LedDig[i].isBlinking = false;
		}
}

void LcdDig_DispBlinkOn( uint8_t side )
{
	if( side & SideLEFT )
		for( int i=0; i<2; i++ )
		{
			g_LedDig[i].isBlinking = true;
		}
	if( side & SideRIGHT )
		for( int i=2; i<4; i++ )
		{
			g_LedDig[i].isBlinking = true;
		}
}