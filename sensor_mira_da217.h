

#ifndef SENSOR_MIRA_DA217_H__
#define SENSOR_MIRA_DA217_H__

#include "sensor.h"
//#include "bma400.h"

#define DA217_ADDR_DEFAULT UINT8_C(0x27)

int rt_hw_da217_init(const char *name, struct rt_sensor_config *cfg);

#endif
