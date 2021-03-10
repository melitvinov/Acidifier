/*****************************************************************************************
������� ��� ������ FM24V02 

������ -      21.12.2018

*****************************************************************************************/

#include "FM24V02.h"

static bool fm24_initialized;
xSemaphoreHandle fm24Mutex;

// ------------- ������� I2C ------------------------
//void delay( uint16_t delay );

void i2c_init()
{
	// ��������� ��� ������� I2C
	GPIO_PinConfigure( PORT_I2C, PIN_SCL, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_IN_FLOATING, GPIO_MODE_INPUT );
	
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 1 );
}

void i2c_start()
{
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 0 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
}

void i2c_stop()
{
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 0 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 1 );
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_IN_FLOATING, GPIO_MODE_INPUT );
}

void i2c_ask()
{
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 0 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
}

void i2c_sendBit( uint8_t bit )
{
	if( bit == 0 )	GPIO_PinWrite( PORT_I2C, PIN_SDA, 0 );
	else			GPIO_PinWrite( PORT_I2C, PIN_SDA, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
}

bool i2c_sendByte( uint8_t byte )
{
	bool ask = false;
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );

	for( int i=7; i>=0; i-- )
	{
		i2c_sendBit( GETBIT(byte,i) );
	}
	// ask
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_IN_FLOATING, GPIO_MODE_INPUT );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	if( 0 == GPIO_PinRead( PORT_I2C, PIN_SDA ) )
		ask = true;
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
	return ask;
}

uint8_t i2c_readBit()
{
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	uint8_t bit = GPIO_PinRead( PORT_I2C, PIN_SDA );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
	return bit;
}

uint8_t i2c_readByte(void)
{
	uint8_t bit, byte = 0;
	
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_IN_FLOATING, GPIO_MODE_INPUT );

	for( int i=7; i>=0; i-- )
	{
		bit = i2c_readBit();
		byte |= bit << i;
	}
	
	return byte;
}

void delay( uint16_t delay )
{
	while(delay--) {	
	}
}

// ------------- ����� ������� I2C ------------------

/*****************************************************************************************
InitFM24V02 - ������������� FM24V02 
******************************************************************************************/
void FM24_Init( void )
{
	fm24Mutex = xSemaphoreCreateMutex();
	
	i2c_init();
	fm24_initialized = true;

//	uint8_t data;
//	FM24_Read(0, &data, 1);
} 
//<-- ����� ������� FM24_Init -----------------------------------------------------------

bool fm24_setAddress( uint16_t addr )
{
	// ����� ���������� (������� ��� ���� ��� ������)
	bool ask = i2c_sendByte( MEMORY_ADDRESS );
	// ����� ����� ������ ������ (������� ������� ���� ����� �������)
	if( ask )
	{
		uint8_t * pByte = (uint8_t*) &addr;
		ask = i2c_sendByte( pByte[1] );
		if( ask ) ask = i2c_sendByte( pByte[0] );
	}	
	return ask;
}

/*****************************************************************************************
FM24_Write - ������ ������ � ������ 
******************************************************************************************/
bool FM24_WriteBytes( uint16_t addr, const uint8_t *data, uint16_t num )
{
	if( !fm24_initialized ) FM24_Init();
	
	bool ask = false;
	
	if( xSemaphoreTake( fm24Mutex , portMAX_DELAY ) == pdTRUE )
	{
	
		// �������� �����
		i2c_start();

		// �������� �����
		ask = fm24_setAddress( addr );
		
		// ������ ���� ������ ������
		for( int i=0; i<num && ask; i++ )
		{
			ask = i2c_sendByte( data[i] );
		}
			
		// � ����� ����
		i2c_stop();
	
		xSemaphoreGive( fm24Mutex );
	}
	return ask;
}

/*****************************************************************************************
FM24_Read - ������ ������ � ����� 
******************************************************************************************/
bool FM24_ReadBytes( uint16_t addr, uint8_t *data, uint16_t num )
{
	if( !fm24_initialized ) FM24_Init();

	if( xSemaphoreTake( fm24Mutex , portMAX_DELAY ) == pdTRUE )
	{
		// �������� �����
		i2c_start();

		// �������� �����
		bool ask = fm24_setAddress( addr );
		
		if( ask )
		{
			// �������� �����
			i2c_start();

			// ����� ���������� (������� ��� ������� ��� ������)
			ask = i2c_sendByte( MEMORY_ADDRESS | 0x01 );
			
			if( ask )
			{
				// ������ ���� ������ ������
				for( int i=0; i<num; i++ )
				{
					data[i] = i2c_readByte();
					if( i != num-1 ) // �� ��������� ���� �� �������� ask
						i2c_ask();
				}
			}
		}		

		// � ����� ����
		i2c_stop();
		
		xSemaphoreGive( fm24Mutex );
		
		return ask;
	}
	
	return false;
}

bool FM24_ReadWords( uint16_t addr, uint16_t *data, uint16_t num )
{
	uint8_t * pBytes = ( uint8_t * ) data;
	bool result = FM24_ReadBytes( addr, pBytes, num * 2 );
	return result;
}

bool FM24_WriteWords( uint16_t addr, const uint16_t *data, uint16_t num )
{
	uint8_t * pBytes = ( uint8_t * ) data;
	bool result = FM24_WriteBytes( addr, pBytes, num * 2 );
	return result;
}
