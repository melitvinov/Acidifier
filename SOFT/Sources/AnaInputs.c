/***********************************************************************************
* Наименование модуля	: Модуль аналоговых входов
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев 
* Дата создания			: 30.05.2018
************************************************************************************/

//--- INCLUDES -------------------
#include "FreeRTOS.h"
#include "task.h"

#include "AnaInputs.h"

// --- DEFINES -------------------

// --- TYPES ---------------------

//--- CONSTANTS ------------------
const uint8_t ainputs[] = 
{
	PH1, PH2
};

const uint8_t PH_CHANNELS_COUNT = sizeof(ainputs) / sizeof( uint8_t );

// тарировочная точка по умолчанию для датчиков PH
const TTarPoint DefaultPhTarPoint_1 = {2000, 740};
const TTarPoint DefaultPhTarPoint_2 = {3150, 430};

//--- GLOBAL VARIABLES -----------
uint16_t sensorsPH_values[PH_CHANNELS_COUNT];	// в массиве хранятся вычисленные значения датчиков PH (float * 100)
uint16_t sensorsPH_adc[PH_CHANNELS_COUNT];		// в массиве хранятся оцифрованные значения датчиков PH (0-4095)
TTarTable Tar_tables[PH_CHANNELS_COUNT];		// в массиве хранятся тарировочные точки для датчиков PH

//--- IRQ ------------------------

//--- FUNCTIONS ------------------
void adc_Initialize (void);
uint16_t adc_get_value(uint8_t channel);

void set_default_tar_points()
{
	// устанавливаем тарировочные точки по умолчанию
	for( int i=0; i<PH_CHANNELS_COUNT; i++ )
	{
		memcpy( &Tar_tables[i].point1, &DefaultPhTarPoint_1, sizeof( TTarPoint ) );
		memcpy( &Tar_tables[i].point2, &DefaultPhTarPoint_2, sizeof( TTarPoint ) );
	}
	// сохраняем в EEPROM текущие настройки тарировочных точек
	FM24_WriteBytes( EEADR_TAR_PH1_P1_ADC, (uint8_t*) &Tar_tables, sizeof( Tar_tables ) );
}

int AInp_ReadAdcValue( uint16_t idx )
{
	if( idx > 1 ) return -1;
	
	return sensorsPH_adc[idx];
}
int AInp_ReadPhValue( uint16_t idx )
{
	if( idx > 1 ) return -1;
	
	return sensorsPH_values[idx];
}

int AInp_ReadAdcTar1( uint16_t idx )
{
	if( idx > 1 ) return -1;
	
	return Tar_tables[idx].point1.adc_value;
}
int AInp_ReadPhTar1( uint16_t idx )
{
	if( idx > 1 ) return -1;
	
	return Tar_tables[idx].point1.tar_value;
}
int AInp_ReadAdcTar2( uint16_t idx )
{
	if( idx > 1 ) return -1;
	
	return Tar_tables[idx].point2.adc_value;
}
int AInp_ReadPhTar2( uint16_t idx )
{
	if( idx > 1 ) return -1;
	
	return Tar_tables[idx].point2.tar_value;
}
bool AInp_WriteAdcTar1( uint16_t idx, uint16_t val )
{
	if( idx > 1 ) return false;
	
	uint16_t ee_addr = idx==0 ? EEADR_TAR_PH1_P1_ADC : (EEADR_TAR_PH1_P1_ADC + sizeof(TTarTable));
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteBytes( ee_addr, (uint8_t*) &val, 2 );
	
	if( isWrited )
	{
		Tar_tables[idx].point1.adc_value = val;
	}
	
	return isWrited;
}
bool AInp_WritePhTar1( uint16_t idx, uint16_t val )
{
	if( idx > 1 ) return false;
	
	uint16_t ee_addr = idx==0 ? EEADR_TAR_PH1_P1_ADC + 2 : (EEADR_TAR_PH1_P1_ADC + sizeof(TTarTable) + 2 );
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteBytes( ee_addr, (uint8_t*) &val, 2 );
	
	if( isWrited )
	{
		Tar_tables[idx].point1.tar_value = val;
	}

	return isWrited;
}
bool AInp_WriteAdcTar2( uint16_t idx, uint16_t val )
{
	if( idx > 1 ) return false;
	
	uint16_t ee_addr = idx==0 ? EEADR_TAR_PH1_P1_ADC + sizeof(TTarPoint) : (EEADR_TAR_PH1_P1_ADC + + sizeof(TTarTable) + sizeof(TTarPoint) );
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteBytes( ee_addr, (uint8_t*) &val, 2 );

	if( isWrited )
	{
		Tar_tables[idx].point2.adc_value = val;
	}
	
	return isWrited;
}
bool AInp_WritePhTar2( uint16_t idx, uint16_t val )
{
	if( idx > 1 ) return false;
	
	uint16_t ee_addr = idx==0 ? EEADR_TAR_PH1_P1_ADC + sizeof(TTarPoint) + 2 : (EEADR_TAR_PH1_P1_ADC + + sizeof(TTarTable) + sizeof(TTarPoint) + 2 );
	// сохраняем в EEPROM
	bool isWrited = FM24_WriteBytes( ee_addr, (uint8_t*) &val, 2 );

	if( isWrited )
	{
		Tar_tables[idx].point2.tar_value = val;
	}
	
	return isWrited;
}

float AInp_GetFloatSensorPh( uint8_t idx )
{
	float value = AInp_ReadPhValue( idx );
	value /= 100.0;
	
	return value;
}

float AInp_GetSystemPh( void )
{
	float value1 = AInp_GetFloatSensorPh( 0 );
	float value2 = AInp_GetFloatSensorPh( 1 );
	value1 /= 100.0;
	value2 /= 100.0;
	
	float value = 0;
	if( value1 < 0 || value2 < 0 )
	{
		value = -1;
	}
	else
	{
		value = fabs( value1 - value2 );
		if( value > 0.5 )
			value = -1;
		else
			value = value1;
	}
	
	return value;
}

int AInp_ReadSystemPh( uint16_t idx )
{
	int ivalue = 0; 
	float fvalue = AInp_GetSystemPh();
	if( fvalue < 0 )
	{
		ivalue = 65535;
	}
	else
	{
		ivalue = fvalue * 100.0;
	}
	
	return ivalue;
}

/*******************************************************
Поток		: Получение и обработка значений датчиков PH
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void AInp_Thread( void *pvParameters )
{
	adc_Initialize();
	FM24_Init();
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 500;
	
	// Получаем тарировочные точки из EEPROM
	memset( &Tar_tables, 0, sizeof(Tar_tables) );
	if( !FM24_ReadWords( EEADR_TAR_PH1_P1_ADC, (uint16_t*) &Tar_tables, PH_CHANNELS_COUNT * sizeof( TTarPoint ) ) || 
		Tar_tables[0].point1.adc_value == 0 || 
		Tar_tables[0].point2.adc_value == 0 )
	{
		// не удалось получить из EEPROM, устанавливаем тарировочные точки по умолчанию
		set_default_tar_points();
	}
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		for( int i = 0; i < PH_CHANNELS_COUNT; i++ )
		{
			uint32_t value32 = 0;
			for( int j=0; j<AVERAGE_COUNT; j++ )
			{
				value32 += adc_get_value( ainputs[i] );
				vTaskDelay(1);
			}
			sensorsPH_adc[i] = value32 / AVERAGE_COUNT;
		}
		
		// вычисляем значения PH
		for( int i = 0; i < PH_CHANNELS_COUNT; i++ )
		{
			float b, k, dX, dY, currValue;
			
			dX = Tar_tables[i].point2.adc_value - Tar_tables[i].point1.adc_value;
			if( dX == 0 )
			{
				currValue = 0;
			}
			else
			{
				dY = Tar_tables[i].point2.tar_value - Tar_tables[i].point1.tar_value;
				k = dY / dX;
				b = Tar_tables[i].point1.tar_value - k * Tar_tables[i].point1.adc_value;
				
				currValue = k * sensorsPH_adc[i] + b;
				if( currValue < 0 )
				{
					currValue = 0;
				}
			}
			
			sensorsPH_values[i] =  (uint16_t) currValue;
		}
	}
}

/*******************************************************
Функция		: Инициализация приложения
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void adc_Initialize (void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// настройки ADC
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // режим работы - одиночный, независимый
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // не сканировать каналы, просто измерить один канал
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // однократное измерение
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // без внешнего триггера
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //выравнивание битов результат - прижать вправо
	ADC_InitStructure.ADC_NbrOfChannel = 1; //количество каналов - одна штука
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);

	// настройка канала
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

	// калибровка АЦП
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1)); 
	
	sensorsPH_values[0] = 0;
	sensorsPH_values[1] = 0;
}

uint16_t adc_get_value(uint8_t channel)
{
	// настройка канала
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}

