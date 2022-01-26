/***********************************************************************************
* Наименование модуля	: Оисание регистров MODBUS, функции чтения и записи платы РЕЛЕ
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 21.08.2020
************************************************************************************/

//--- INCLUDES -------------------
#include "FreeRTOS.h"
#include "task.h"
#include "modbus_regs.h"

#include "FM24V02.h"
#include "AnaInputs.h"
#include "Regulator.h"
#include "Product_Id.h"

// --- DEFINES -------------------

// --- TYPES ---------------------

//--- CONSTANTS ------------------
extern const uint16_t UINT_VERSION;

// описательные регистры
const uint16_t gDescRegs[] = {
	id_Vendor,	// vendor_id
	id_Acidifier, // product_id
	0,	// firmware_version
	0x0000,	// firmware_git_hash_0
	0x0000,	// firmware_git_hash_1
	0x0000,	// firmware_git_hash_2
	0x0000,	// firmware_git_hash_3
	0x0000,	// firmware_build_timestamp_0
	0x0000	// firmware_build_timestamp_1
};

//--- FORWARD --------------------
TRegEntry * get_regentry_by_regaddr( uint16_t regaddr );
int readDescReg( uint16_t idx );
int readAddIn( uint16_t idx );
bool writeAddIn( uint16_t idx, uint16_t val );
//int readStatus( uint16_t idx );
//bool writeStatus( uint16_t idx, uint16_t val );

//--- GLOBAL VARIABLES -----------

extern uint16_t g_Status;

uint16_t regs_addin[15] = {
	0, 0, 0, 0xBEB0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 
};

//--- EXTERN ---------------------
extern uint8_t GetDeviceAddress(void);
extern int ReadWorkMode( uint16_t idx );
extern int ReadSetupPhValue( uint16_t idx );
extern bool WriteSetupPhValue( uint16_t idx, uint16_t val );

// Регистры платы подкислителя
TRegEntry RegEntries[] = 
{
	{.addr=0, .idx = 0, .read = readDescReg, .write=0, },
	{.addr=1, .idx = 1, .read = readDescReg, .write=0, },
	{.addr=2, .idx = 2, .read = readDescReg, .write=0, },
	{.addr=3, .idx = 3, .read = readDescReg, .write=0, },
	{.addr=4, .idx = 4, .read = readDescReg, .write=0, },
	{.addr=5, .idx = 5, .read = readDescReg, .write=0, },
	{.addr=6, .idx = 6, .read = readDescReg, .write=0, },
	{.addr=7, .idx = 7, .read = readDescReg, .write=0, },
	{.addr=8, .idx = 8, .read = readDescReg, .write=0, },
	
	{.addr=30, .idx = 0, .read = readAddIn, .write=writeAddIn, },
	{.addr=31, .idx = 1, .read = readAddIn, .write=0, },
	{.addr=32, .idx = 2, .read = readAddIn, .write=0, },
	
	{.addr=50, .idx = 3, .read = readAddIn, .write=writeAddIn, },
	{.addr=51, .idx = 4, .read = readAddIn, .write=writeAddIn, },
	{.addr=52, .idx = 5, .read = readAddIn, .write=writeAddIn, },
	{.addr=53, .idx = 6, .read = readAddIn, .write=writeAddIn, },
	{.addr=54, .idx = 7, .read = readAddIn, .write=writeAddIn, },
	{.addr=55, .idx = 8, .read = readAddIn, .write=writeAddIn, },
	{.addr=56, .idx = 9, .read = readAddIn, .write=writeAddIn, },
	{.addr=57, .idx = 10, .read = readAddIn, .write=writeAddIn, },
	{.addr=58, .idx = 11, .read = readAddIn, .write=writeAddIn, },
	{.addr=59, .idx = 12, .read = readAddIn, .write=writeAddIn, },

	// Текущий режим работы ( 1-Регулирование, 2-калибровка Т1, 3-калибровка Т2 )
	{.addr=START_REG_VALUES-1, .idx = 0, .read = ReadWorkMode, .write=0 },

	// Значение АЦП датчика PH №1
	{.addr=START_REG_VALUES+0, .idx = 0, .read = AInp_ReadAdcValue, .write=0 },
	// Значение АЦП датчика PH №2
	{.addr=START_REG_VALUES+1, .idx = 1, .read = AInp_ReadAdcValue, .write=0 },
	// Значение датчика PH №1 ( умноженное на 10 )
	{.addr=START_REG_VALUES+2, .idx = 0, .read = AInp_ReadPhValue, .write=0 },
	// Значение датчика PH №2 ( умноженное на 10 )
	{.addr=START_REG_VALUES+3, .idx = 1, .read = AInp_ReadPhValue, .write=0 },
	// Системное значение PH ( умноженное на 10 ) 
	//( берется с датчика PH №1. Если разбег значений датчиков 1 и 2 больше +-0.5, то -1 )
	{.addr=START_REG_VALUES+4, .idx = 0, .read = AInp_ReadSystemPh, .write=0 },
	// Заданное значение PH ( умноженное на 10 )
	{.addr=START_REG_VALUES+5, .idx = 0, .read = ReadSetupPhValue, .write=WriteSetupPhValue },

	// тарировочная точка 1 для датчика №1
	{.addr=START_REG_TAR_POINT+0, .idx = 0, .read = AInp_ReadAdcTar1, .write=AInp_WriteAdcTar1 },
	{.addr=START_REG_TAR_POINT+1, .idx = 0, .read = AInp_ReadPhTar1, .write=AInp_WritePhTar1 },
	// тарировочная точка 2 для датчика №1
	{.addr=START_REG_TAR_POINT+2, .idx = 0, .read = AInp_ReadAdcTar2, .write=AInp_WriteAdcTar2 },
	{.addr=START_REG_TAR_POINT+3, .idx = 0, .read = AInp_ReadPhTar2, .write=AInp_WritePhTar2 },

	// тарировочная точка 1 для датчика №2
	{.addr=START_REG_TAR_POINT+4, .idx = 1, .read = AInp_ReadAdcTar1, .write=AInp_WriteAdcTar1 },
	{.addr=START_REG_TAR_POINT+5, .idx = 1, .read = AInp_ReadPhTar1, .write=AInp_WritePhTar1 },
	// тарировочная точка 2 для датчика №2
	{.addr=START_REG_TAR_POINT+6, .idx = 1, .read = AInp_ReadAdcTar2, .write=AInp_WriteAdcTar2 },
	{.addr=START_REG_TAR_POINT+7, .idx = 1, .read = AInp_ReadPhTar2, .write=AInp_WritePhTar2 },

	// пропорциональный коэффициент 
	{.addr=START_REG_COEFFICIENT+0, .idx = 0, .read = Reg_ReadCoefficient, .write=Reg_WriteCoefficient },
	// интегральный коэффициент 
	{.addr=START_REG_COEFFICIENT+1, .idx = 1, .read = Reg_ReadCoefficient, .write=Reg_WriteCoefficient },
	// дифференциальный коэффициент 
	{.addr=START_REG_COEFFICIENT+2, .idx = 2, .read = Reg_ReadCoefficient, .write=Reg_WriteCoefficient },
	
	// таймаут в секундах задержки включения насоса и процесса регулирования после подачи воды
	{.addr=START_REG_TIME_DEFINE+0, .idx = 0, .read = Reg_Read_TIMEOUT_TURN_PUMP_ON_SEC, .write=Reg_Write_TIMEOUT_TURN_PUMP_ON_SEC },
	// максимальное время в секундах отклонения значения PH от задания PH для остановки регулятора и насоса
	{.addr=START_REG_TIME_DEFINE+1, .idx = 0, .read = Reg_Read_TIMEOUT_ERROR_PH_SEC, .write=Reg_Write_TIMEOUT_ERROR_PH_SEC },
	// время хода регулятора в секундах
	{.addr=START_REG_TIME_DEFINE+2, .idx = 0, .read = Reg_Read_REG_CYCLETIME_SEC, .write=Reg_Write_REG_CYCLETIME_SEC },
	
	// флаг - отсутствие воды (0-1)
	{.addr=START_REG_MONITORING+0, .idx = MON_IsNoWater, .read = Reg_Read_MonitoringValue, .write=0 },
	// флаг - ошибка датчиков
	{.addr=START_REG_MONITORING+1, .idx = MON_IsErrSensors, .read = Reg_Read_MonitoringValue, .write=0 },
	// флаг - ошибка регулирования
	{.addr=START_REG_MONITORING+2, .idx = MON_IsErrTimeoutSetupPh, .read = Reg_Read_MonitoringValue, .write=0 },
	
	// установленное значение PH
	{.addr=START_REG_MONITORING+3, .idx = MON_PH_Setup, .read = Reg_Read_MonitoringValue, .write=0 },
	// текущее значение PH1
	{.addr=START_REG_MONITORING+4, .idx = MON_PH1_Current, .read = Reg_Read_MonitoringValue, .write=0 },
	// текущее значение PH2
	{.addr=START_REG_MONITORING+5, .idx = MON_PH2_Current, .read = Reg_Read_MonitoringValue, .write=0 },
	
	// % открытия регулятора
	{.addr=START_REG_MONITORING+6, .idx = MON_RegPercentOn, .read = Reg_Read_MonitoringValue, .write=0 },
	// значение PID * 100;
	{.addr=START_REG_MONITORING+7, .idx = MON_PID_Value, .read = Reg_Read_MonitoringValue, .write=0 },
	// знак PID 
	{.addr=START_REG_MONITORING+8, .idx = MON_PID_Positive, .read = Reg_Read_MonitoringValue, .write=0 },
	// DeltaPercent;
	{.addr=START_REG_MONITORING+9, .idx = MON_DeltaPercent, .read = Reg_Read_MonitoringValue, .write=0 },
	// знак DeltaPercent 
	{.addr=START_REG_MONITORING+10, .idx = MON_DeltaPercentPositive, .read = Reg_Read_MonitoringValue, .write=0 },
	// длительность открытия клапана в мс.
	{.addr=START_REG_MONITORING+11, .idx = MON_ImpulseTime_ms, .read = Reg_Read_MonitoringValue, .write=0 },
};

//--- FUNCTIONS ------------------

int readDescReg( uint16_t idx )
{
	if( idx == 2 )
		return UINT_VERSION;
	
	return gDescRegs[idx];
}

int readAddIn( uint16_t idx )
{
	return gDescRegs[idx];
}

bool writeAddIn( uint16_t idx, uint16_t val )
{
	if( idx > 14 ) return false;
	regs_addin[idx] = val;
	return true;
}

//int readStatus( uint16_t idx )
//{
//	return g_Status;
//}

//bool writeStatus( uint16_t idx, uint16_t val )
//{
//	g_Status = val;
//	return true;
//}

/*******************************************************
Функция		: Пытается прочитать значение указанного регистра
Параметр 1	: индекс регистра
Параметр 2	: следующий байт
Возвр. знач.: флаг - значение прочитано
********************************************************/
bool REG_Read( uint16_t RegAddr, uint16_t * pValue )
{
	if( !pValue )
		return false;
	
	TRegEntry * pReg = get_regentry_by_regaddr( RegAddr );
	if( pReg )
	{
		if( pReg->read )
		{
			int result = pReg->read( pReg->idx );
			if( result >= 0 )
			{
				*pValue = (uint16_t) result;
				return true;
			}
		}
	}

	return false;
}	

/*******************************************************
Функция		: Пытается прочитать значение указанного регистра
Параметр 1	: индекс регистра
Параметр 2	: следующий байт
Возвр. знач.: флаг - значение прочитано
********************************************************/
bool REG_Write( uint16_t RegAddr, uint16_t value )
{
	TRegEntry * pReg = get_regentry_by_regaddr( RegAddr );
	if( pReg )
	{
		if( pReg->write )
		{
			bool result = pReg->write( pReg->idx, value );
			return result;
		}
	}
	return false;
}	

/*******************************************************
Функция		: Проверяет регистр на возможность записи в него
Параметр 1	: номер регистра
Возвр. знач.: флаг - запись разрешена 
********************************************************/
bool REG_isWriteEnable( uint16_t regAddr )
{
	TRegEntry * pReg = get_regentry_by_regaddr( regAddr );
	if( pReg )
	{
		if( pReg->write )
		{
			return true;
		}
	}

	return false;
}

/*******************************************************
Функция		: Ищет регистр по его адресу
Параметр 1	: адрес регистра
Возвр. знач.: указатель на структуру регистра
********************************************************/
TRegEntry * get_regentry_by_regaddr( uint16_t regaddr )
{
	TRegEntry * regEntry = 0;

	uint16_t total = sizeof(RegEntries);
	uint16_t one = sizeof(TRegEntry);
	uint16_t count = total / one;	// Количество регистров
	
	for( int i=0; i<count; i++ )
	{
		if( RegEntries[i].addr == regaddr )
		{
			regEntry = &(RegEntries[i]);
			break;
		}
	}
	return regEntry;
}
