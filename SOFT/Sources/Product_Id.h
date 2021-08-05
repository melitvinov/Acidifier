#ifndef __PRODUCT_ID_H
#define __PRODUCT_ID_H

typedef enum __prodict_id
{
    id_Vendor				= 0x1234,
    id_WeightTrayModbus 	= 121,
    id_ModuleEC601 			= 150,
    id_ModulePH601 			= 151,
    id_ModuleSensors601 	= 152,
    id_ModuleRelay601 		= 153,
    id_ModuleLevels601 		= 154,
    id_WeatherStation 		= 155,
    id_Acidifier 			= 156,
    id_MeasurementCell      = 157,
} EProductId;

#endif
