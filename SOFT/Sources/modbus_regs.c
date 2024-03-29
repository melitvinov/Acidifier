/***********************************************************************************
* ������������ ������	: ������� ��������� MODBUS, ������� ������ � ������ ����� ����
*-----------------------------------------------------------------------------------
* ������				: 1.0
* �����					: ��������
* ���� ��������			: 21.08.2020
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

// ������������ ��������
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

//uint16_t idx = 0;

//--- EXTERN ---------------------
extern uint8_t GetDeviceAddress(void);
extern int ReadWorkMode( uint16_t idx );
extern int ReadSetupPhValue( uint16_t idx );
extern bool WriteSetupPhValue( uint16_t idx, uint16_t val );

// �������� ����� ������������
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

	// ������� ����� ������ ( 1-�������������, 2-���������� �1, 3-���������� �2 )
	{.addr=START_REG_VALUES+0, .idx = 0, .read = ReadWorkMode, .write=0 },

	// �������� ��� ������� PH �1
	{.addr=START_REG_VALUES+1, .idx = 0, .read = AInp_ReadAdcValue, .write=0 },
	// �������� ��� ������� PH �2
	{.addr=START_REG_VALUES+2, .idx = 1, .read = AInp_ReadAdcValue, .write=0 },
	// �������� ������� PH �1 ( ���������� �� 10 )
	{.addr=START_REG_VALUES+3, .idx = 0, .read = AInp_ReadPhValue, .write=0 },
	// �������� ������� PH �2 ( ���������� �� 10 )
	{.addr=START_REG_VALUES+4, .idx = 1, .read = AInp_ReadPhValue, .write=0 },
	// ��������� �������� PH ( ���������� �� 10 ) 
	//( ������� � ������� PH �1. ���� ������ �������� �������� 1 � 2 ������ +-0.5, �� -1 )
	{.addr=START_REG_VALUES+5, .idx = 0, .read = AInp_ReadSystemPh, .write=0 },
	// �������� �������� PH ( ���������� �� 10 )
	{.addr=START_REG_VALUES+6, .idx = 0, .read = ReadSetupPhValue, .write=WriteSetupPhValue },

	// ������������ ����� 1 ��� ������� �1
	{.addr=START_REG_VALUES+7, .idx = 0, .read = AInp_ReadAdcTar1, .write=AInp_WriteAdcTar1 },
	{.addr=START_REG_VALUES+8, .idx = 0, .read = AInp_ReadPhTar1, .write=AInp_WritePhTar1 },
	// ������������ ����� 2 ��� ������� �1
	{.addr=START_REG_VALUES+9, .idx = 0, .read = AInp_ReadAdcTar2, .write=AInp_WriteAdcTar2 },
	{.addr=START_REG_VALUES+10, .idx = 0, .read = AInp_ReadPhTar2, .write=AInp_WritePhTar2 },

	// ������������ ����� 1 ��� ������� �2
	{.addr=START_REG_VALUES+11, .idx = 1, .read = AInp_ReadAdcTar1, .write=AInp_WriteAdcTar1 },
	{.addr=START_REG_VALUES+12, .idx = 1, .read = AInp_ReadPhTar1, .write=AInp_WritePhTar1 },
	// ������������ ����� 2 ��� ������� �2
	{.addr=START_REG_VALUES+13, .idx = 1, .read = AInp_ReadAdcTar2, .write=AInp_WriteAdcTar2 },
	{.addr=START_REG_VALUES+14, .idx = 1, .read = AInp_ReadPhTar2, .write=AInp_WritePhTar2 },

	// ���������������� ����������� 
	{.addr=START_REG_VALUES+15, .idx = 0, .read = Reg_ReadCoefficient, .write=Reg_WriteCoefficient },
	// ������������ ����������� 
	{.addr=START_REG_VALUES+16, .idx = 1, .read = Reg_ReadCoefficient, .write=Reg_WriteCoefficient },
	// ���������������� ����������� 
	{.addr=START_REG_VALUES+17, .idx = 2, .read = Reg_ReadCoefficient, .write=Reg_WriteCoefficient },
	
	// ������� � �������� �������� ��������� ������ � �������� ������������� ����� ������ ����
	{.addr=START_REG_VALUES+18, .idx = 0, .read = Reg_Read_TIMEOUT_REGULATOR_ON_SEC, .write=Reg_Write_TIMEOUT_REGULATOR_ON_SEC },
	// ������������ ����� � �������� ���������� �������� PH �� ������� PH ��� ��������� ���������� � ������
	{.addr=START_REG_VALUES+19, .idx = 0, .read = Reg_Read_TIMEOUT_ERROR_PH_SEC, .write=Reg_Write_TIMEOUT_ERROR_PH_SEC },
	// ������������ ��������� �������
	{.addr=START_REG_VALUES+20, .idx = 0, .read = Reg_Read_REG_CYCLETIME_SEC, .write=Reg_Write_REG_CYCLETIME_SEC },
	// ������� �������� ���������� ������ � �������� (1-20)
	{.addr=START_REG_VALUES+21, .idx = 0, .read = Reg_Read_DELAY_PUMP_OFF_SEC, .write=Reg_Write_DELAY_PUMP_OFF_SEC },
	
	// ���� - ���������� ���� (0-1)
	{.addr=START_REG_VALUES+22, .idx = MON_IsNoWater, .read = Reg_Read_MonitoringValue, .write=0 },
	// ���� - ������ ��������
	{.addr=START_REG_VALUES+23, .idx = MON_IsErrSensors, .read = Reg_Read_MonitoringValue, .write=0 },
	// ���� - ������ �������������
	{.addr=START_REG_VALUES+24, .idx = MON_IsErrTimeoutSetupPh, .read = Reg_Read_MonitoringValue, .write=0 },
	
	// ������������� �������� PH
	{.addr=START_REG_VALUES+25, .idx = MON_PH_Setup, .read = Reg_Read_MonitoringValue, .write=0 },
	// ������� �������� PH1
	{.addr=START_REG_VALUES+26, .idx = MON_PH1_Current, .read = Reg_Read_MonitoringValue, .write=0 },
	// ������� �������� PH2
	{.addr=START_REG_VALUES+27, .idx = MON_PH2_Current, .read = Reg_Read_MonitoringValue, .write=0 },
	
	// % �������� ����������
	{.addr=START_REG_VALUES+28, .idx = MON_RegPercentOn, .read = Reg_Read_MonitoringValue, .write=0 },
	// �������� PID * 100;
	{.addr=START_REG_VALUES+29, .idx = MON_PID_Value, .read = Reg_Read_MonitoringValue, .write=0 },
	// ���� PID 
	{.addr=START_REG_VALUES+30, .idx = MON_PID_Positive, .read = Reg_Read_MonitoringValue, .write=0 },
	// DeltaPercent;
	{.addr=START_REG_VALUES+31, .idx = MON_DeltaPercent, .read = Reg_Read_MonitoringValue, .write=0 },
	// ���� DeltaPercent 
	{.addr=START_REG_VALUES+32, .idx = MON_DeltaPercentPositive, .read = Reg_Read_MonitoringValue, .write=0 },
	// ������������ �������� ������� � ��.
	{.addr=START_REG_VALUES+33, .idx = MON_ImpulseTime_ms, .read = Reg_Read_MonitoringValue, .write=0 },
	
	// ������� ����������� �������� ����������
	// ������ �������
	// ������ ������
	{.addr=START_REG_VALUES+34, .idx = 0, .read = Reg_OptValues_Read, .write=0 },	// ph_setup_def
	{.addr=START_REG_VALUES+35, .idx = 1, .read = Reg_OptValues_Read, .write=Reg_OptValues_Write },	// reg_value_opt_def
	{.addr=START_REG_VALUES+36, .idx = 2, .read = Reg_OptValues_Read, .write=0 },	// ph_setup_user
	{.addr=START_REG_VALUES+37, .idx = 3, .read = Reg_OptValues_Read, .write=0 },	// reg_value_opt_calc
	// ������ ������
	{.addr=START_REG_VALUES+38, .idx = 4, .read = Reg_OptValues_Read, .write=0 },	// ph_setup_def
	{.addr=START_REG_VALUES+39, .idx = 5, .read = Reg_OptValues_Read, .write=Reg_OptValues_Write },	// reg_value_opt_def
	{.addr=START_REG_VALUES+40, .idx = 6, .read = Reg_OptValues_Read, .write=0 },	// ph_setup_user
	{.addr=START_REG_VALUES+41, .idx = 7, .read = Reg_OptValues_Read, .write=0 },	// reg_value_opt_calc
	// ������ ������
	{.addr=START_REG_VALUES+42, .idx = 8, .read = Reg_OptValues_Read, .write=0 },	// ph_setup_def
	{.addr=START_REG_VALUES+43, .idx = 9, .read = Reg_OptValues_Read, .write=Reg_OptValues_Write },	// reg_value_opt_def
	{.addr=START_REG_VALUES+44, .idx =10, .read = Reg_OptValues_Read, .write=0 },// ph_setup_user
	{.addr=START_REG_VALUES+45, .idx =11, .read = Reg_OptValues_Read, .write=0 },// reg_value_opt_calc
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
�������		: �������� ��������� �������� ���������� ��������
�������� 1	: ������ ��������
�������� 2	: ��������� ����
�����. ����.: ���� - �������� ���������
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
�������		: �������� ��������� �������� ���������� ��������
�������� 1	: ������ ��������
�������� 2	: ��������� ����
�����. ����.: ���� - �������� ���������
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
�������		: ��������� ������� �� ����������� ������ � ����
�������� 1	: ����� ��������
�����. ����.: ���� - ������ ��������� 
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
�������		: ���� ������� �� ��� ������
�������� 1	: ����� ��������
�����. ����.: ��������� �� ��������� ��������
********************************************************/
TRegEntry * get_regentry_by_regaddr( uint16_t regaddr )
{
	TRegEntry * regEntry = 0;

	uint16_t total = sizeof(RegEntries);
	uint16_t one = sizeof(TRegEntry);
	uint16_t count = total / one;	// ���������� ���������
	
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
