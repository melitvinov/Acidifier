/***********************************************************************************
* ������������ ���� ���	: ���������� ����������� PH
*
************************************************************************************/

#ifndef __REGULATOR_H_
#define __REGULATOR_H_

//--- INCLUDES -------------------
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

#include "FreeRTOS.h"
#include "task.h"

// --- DEFINES -------------------
#define REL_ON	(0) // ���������� ������� ��� ��������� ����
#define REL_OFF	(1)	// ���������� ������� ��� ���������� ��������

#define TIMEOUT_SENSORS_TOO_DIFF_MS		5000 // ����-��� � �� ��� ������ ��� ������� ��������� �������� PH ������ ���������� ��������	

// --- TYPES ---------------------

typedef struct _relay_desc
{
	GPIO_TypeDef *GPIOx;
	uint16_t pin;
} TRelDesc;

typedef struct _opt_table_point
{
	uint16_t ph_setup;
	uint16_t opt_def;
	uint16_t opt_calc;
} TOptTablePoint;

//--- FUNCTIONS ------------------
void Reg_Init(void);

void Thread_Regulator( void *pvParameters );

int Reg_ReadCoefficient( uint16_t idx );
bool Reg_WriteCoefficient( uint16_t idx, uint16_t val );

int Reg_Read_TIMEOUT_TURN_PUMP_ON_SEC( uint16_t idx );
int Reg_Read_TIMEOUT_ERROR_PH_SEC( uint16_t idx );
bool Reg_Write_TIMEOUT_TURN_PUMP_ON_SEC( uint16_t idx, uint16_t val );
bool Reg_Write_TIMEOUT_ERROR_PH_SEC( uint16_t idx, uint16_t val );

int Reg_Read_REG_CYCLETIME_SEC( uint16_t idx );
bool Reg_Write_REG_CYCLETIME_SEC( uint16_t idx, uint16_t val );

typedef enum __monitoring_types
{
	MON_IsNoWater	 		= 1,			// ���� - ���������� ���� (0-1)
	MON_IsErrSensors 		= 2,			// ���� - ������ ��������
	MON_IsErrTimeoutSetupPh	= 3,			// ���� - ������ �������������

	MON_PH_Setup			= 4,			// ������������� �������� PH
	MON_PH1_Current			= 5,			// ������� �������� PH1
	MON_PH2_Current			= 6,			// ������� �������� PH2
	
	MON_RegPercentOn 		= 7,			// ������� �������� ���������� (0-100)
	MON_PID_Value 			= 8,			// ������� �������� �������� PID * 100;
	MON_PID_Positive 		= 9,			// ���� PID ( 1 - ����, 0 - ����� )
	MON_DeltaPercent 		= 10,			// ������� �������� ���������� ��� ���������� �������� ��������
	MON_DeltaPercentPositive= 11,			// ���� DeltaPercent ( 1 - ����, 0 - ����� )
	MON_ImpulseTime_ms 		= 12,			// ������������ �������� ������� � ��.
} EMonitoringType;

int Reg_Read_MonitoringValue( uint16_t idx );	// ������ ������ �� ��������� ����������� �������� ���������

void Reg_RestartWaterTimer(void);

#endif
