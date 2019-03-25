

#ifndef SENSOR_MIRA_DA217_H__
#define SENSOR_MIRA_DA217_H__

#include "sensor.h"
//#include "bma400.h"

//#define BMA400_ADDR_DEFAULT (BMA400_I2C_ADDRESS_SDO_LOW)

int rt_hw_da217_init(const char *name, struct rt_sensor_config *cfg);

#endif
