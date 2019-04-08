/**\mainpage
 * Copyright (C) 2017 - 2019 MiraMEMS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * File		da217.c
 * Date		3 Apr 2019
 * Version	0.0.1
 *
 */
/*! @file da217.c
 * @brief Sensor driver for DA217 sensor
 */
#include "da217.h"
/************************** Internal macros *******************************/
/********************** Static function declarations ************************/
/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of da217_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t null_ptr_check(const struct da217_dev *dev);

/*!
 * @brief This internal API is used to set the accel configurations in sensor
 *
 * @param[in] accel_conf : Structure instance with accel configurations
 * @param[in] dev             : Structure instance of da217_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_accel_conf(const struct da217_acc_conf *accel_conf, const struct da217_dev *dev);

/*!
 * @brief This API reads accel data along with sensor time
 *
 * @param[in,out] accel  : Structure instance to store the accel data
 * @param[in] dev        : Structure instance of da217_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_accel_data(struct da217_sensor_data *accel, const struct da217_dev *dev);

/*!
 * @brief This internal API is used to get the accel configurations in sensor
 *
 * @param[in,out] accel_conf  : Structure instance of basic
 *                              accelerometer configuration
 * @param[in] dev             : Structure instance of da217_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_accel_conf(struct da217_acc_conf *accel_conf, const struct da217_dev *dev);

/********************** Global function definitions ************************/
/*!
 *  @brief This API is the entry point, Call this API before using other APIs.
 *  This API reads the chip-id of the sensor which is the first step to
 *  verify the sensor and updates the trim parameters of the sensor.
 */
int8_t da217_init(struct da217_dev *dev)
{
	int8_t rslt;
	uint8_t chip_id = 0;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == DA217_OK) {
		/* Initial power-up time */
		dev->delay_ms(5);
		/* Assigning dummy byte value, TODO */
		if (dev->intf == DA217_SPI_INTF) {
			/* Dummy Byte availability */
			dev->dummy_byte = 1;
			/* Dummy read of Chip-ID in SPI mode */
			rslt = da217_get_regs(DA217_CHIP_ID_ADDR, &chip_id, 1, dev);
		} else {
			dev->dummy_byte = 0;
		}
		if (rslt == DA217_OK) {
			/* Chip ID of the sensor is read */
			rslt = da217_get_regs(DA217_CHIP_ID_ADDR, &chip_id, 1, dev);
			/* Proceed if everything is fine until now */
			if (rslt == DA217_OK) {
				/* Check for chip id validity */
				if (chip_id == DA217_CHIP_ID) {
					/* Store the chip ID in dev structure */
					dev->chip_id = chip_id;
				} else {
					rslt = DA217_E_DEV_NOT_FOUND;
				}
			}
		}
	}

	return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t da217_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, const struct da217_dev *dev)
{
	int8_t rslt;
	uint8_t count;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if ((rslt == DA217_OK) && (reg_data != NULL)) {
		/* Write the data to the reg_addr */
		/* SPI write requires to set The MSB of reg_addr as 0
		   but in default the MSB is always 0 */
		if (len == 1) {
			rslt = dev->write(dev->intf_ptr, dev->dev_id, reg_addr, reg_data, len);
			if (rslt != DA217_OK) {
				/* Failure case */
				rslt = DA217_E_COM_FAIL;
			}
		}
		/* Burst write is not allowed thus we split burst case write
		 * into single byte writes Thus user can write multiple bytes
		 * with ease */
		if (len > 1) {
			for (count = 0; count < len; count++) {
				rslt = dev->write(dev->intf_ptr, dev->dev_id, reg_addr, &reg_data[count], 1);
				reg_addr++;
			}
		}
	} else {
		rslt = DA217_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t da217_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, const struct da217_dev *dev)
{
	int8_t rslt;
	uint16_t index;
	uint16_t temp_len = len;
	uint8_t temp_buff[temp_len];

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if ((rslt == DA217_OK) && (reg_data != NULL)) {
		if (dev->intf != DA217_I2C_INTF) {
			/* If interface selected is SPI */
			reg_addr = reg_addr | DA217_SPI_RD_MASK;
		}
		/* Read the data from the reg_addr */
		rslt = dev->read(dev->intf_ptr, dev->dev_id, reg_addr, temp_buff, temp_len);
		if (rslt == DA217_OK) {
			for (index = 0; index < len; index++) {
				/* Parse the data read and store in "reg_data"
				 * buffer so that the dummy byte is removed
				 * and user will get only valid data
				 */
				reg_data[index] = temp_buff[index];
			}
		}
		if (rslt != DA217_OK) {
			/* Failure case */
			rslt = DA217_E_COM_FAIL;
		}
	} else {
		rslt = DA217_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API is used to perform soft-reset of the sensor
 * where all the registers are reset to their default values.
 */
int8_t da217_soft_reset(const struct da217_dev *dev)
{
	int8_t rslt;
	uint8_t data = DA217_SOFT_RESET_CMD;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == DA217_OK) {
		/* Reset the device */
		rslt = da217_get_regs(0x00, &data, 1, dev);
		data = DA217_SET_BITS_POS_0(data, DA217_SOFT_RESET_CMD, DA217_SOFT_RESET_CMD);
		rslt = da217_set_regs(DA217_SPI_CONFIG, &data, 1, dev);
		dev->delay_ms(DA217_SOFT_RESET_DELAY_MS);
		if ((rslt == DA217_OK) && (dev->intf == DA217_SPI_INTF)) {
			/* Dummy read of 0x7F register to enable SPI Interface
			 * if SPI is used
			 */
			rslt = da217_get_regs(0x7F, &data, 1, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This API is used to set the power mode of the sensor.
 */
int8_t da217_set_power_mode(uint8_t power_mode, const struct da217_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data = 0;

	rslt = null_ptr_check(dev);

	if (rslt == DA217_OK) {
		rslt = da217_get_regs(DA217_ACCEL_MODE_BW, &reg_data, 1, dev);
	}

	if (rslt == DA217_OK) {
		reg_data = DA217_SET_BITS(reg_data, DA217_POWER_MODE, power_mode);
		/* Set the power mode of sensor */
		rslt = da217_set_regs(DA217_ACCEL_MODE_BW, &reg_data, 1, dev);
		if (power_mode == DA217_LOW_POWER_MODE) {
			/* A delay of 1/ODR is required to switch power modes*/
			dev->delay_ms(100);
		} else {
			dev->delay_ms(100);
		}
	}

	return rslt;
}

/*!
 * @brief This API is used to get the power mode of the sensor.
 */
int8_t da217_get_power_mode(uint8_t *power_mode, const struct da217_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == DA217_OK) {
		rslt = da217_get_regs(DA217_STATUS_ADDR, &reg_data, 1, dev);
		*power_mode = DA217_GET_BITS(reg_data, DA217_POWER_MODE_STATUS);
	}

	return rslt;
}

/*!
 * @brief This API is used to get the accel data along with the sensor-time
 */
int8_t da217_get_accel_data(struct da217_sensor_data *accel, const struct da217_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if ((rslt == DA217_OK) && (accel != NULL)) {
		/* Read and store the accel data */
		rslt = get_accel_data(accel, dev);
	} else {
		rslt = DA217_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API is used to set the sensor settings like sensor
 * configurations and interrupt configurations
 */
int8_t da217_set_sensor_conf(const struct da217_sensor_conf *conf, uint16_t n_sett, const struct da217_dev *dev)
{
	int8_t rslt;
	uint16_t idx = 0;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
    if (rslt == DA217_OK) {
        for (idx = 0; idx < n_sett; idx++) {
            switch (conf[idx].type) {
            case DA217_ACCEL:
                /* Setting Accel configurations */
                rslt = set_accel_conf(&conf[idx].param.accel, dev);
                if (rslt == DA217_OK) {
                    /* Set the INT pin mapping */
                }
                break;
		    default:
				rslt = DA217_E_INVALID_CONFIG;
			}
		}
	}
	return rslt;
}

/*!
 * @brief This API is used to get the sensor settings like sensor
 * configurations and interrupt configurations and store
 * them in the corresponding structure instance
 */
int8_t da217_get_sensor_conf(struct da217_sensor_conf *conf, uint16_t n_sett, const struct da217_dev *dev)
{
	int8_t rslt = DA217_OK;
	uint16_t idx = 0;

	if (conf == NULL) {
		rslt = DA217_E_NULL_PTR;
	}

	for (idx = 0; (idx < n_sett) && (rslt == DA217_OK); idx++) {
		switch (conf[idx].type) {
		case DA217_ACCEL:
			/* Accel configuration settings */
			rslt = get_accel_conf(&conf[idx].param.accel, dev);
			if (rslt == DA217_OK) {
				/* Get the INT pin mapping */
			}
			break;
		default:
			rslt = DA217_E_INVALID_CONFIG;
		}
	}

	return rslt;
}

/*!
 * @brief This API is used to get the device specific settings and store
 * them in the corresponding structure instance
 */
int8_t da217_get_device_conf(struct da217_device_conf *conf, uint8_t n_sett, const struct da217_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == DA217_OK) {

	}

	return rslt;
}

/****************************************************************************/
/**\name	INTERNAL APIs                                               */
/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct da217_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
		/* Device structure pointer is not valid */
		rslt = DA217_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = DA217_OK;
	}

	return rslt;
}

/*!
 * @brief This internal API is used to set the accel configurations in sensor
 */
static int8_t set_accel_conf(const struct da217_acc_conf *accel_conf, const struct da217_dev *dev)
{
	int8_t rslt;
	uint8_t data_array[3] = { 0, 0, 0 };

	/* Update the accel configurations from the user structure
	 * accel_conf */
	rslt = da217_get_regs(DA217_ACCEL_RESOLUTION_RANGE, data_array, 3, dev);
	if (rslt == DA217_OK) {
		data_array[0] = DA217_SET_BITS(data_array[0], DA217_ACCEL_RANGE, accel_conf->range);
		data_array[1] = DA217_SET_BITS_POS_0(data_array[1], DA217_ACCEL_ODR, accel_conf->odr);
		data_array[2] = DA217_SET_BITS(data_array[2], DA217_BW, accel_conf->bw);
		/* Set the accel configurations in the sensor */
		rslt = da217_set_regs(DA217_ACCEL_RESOLUTION_RANGE, data_array, 3, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API is used to set the accel configurations in sensor
 */
static int8_t get_accel_conf(struct da217_acc_conf *accel_conf, const struct da217_dev *dev)
{
	int8_t rslt;
	uint8_t data_array[3];

	rslt = da217_get_regs(DA217_ACCEL_RESOLUTION_RANGE, data_array, 3, dev);
	if (rslt == DA217_OK) {
		accel_conf->range = DA217_GET_BITS(data_array[0], DA217_ACCEL_RANGE);
		accel_conf->odr = DA217_GET_BITS_POS_0(data_array[1], DA217_ACCEL_ODR);
		accel_conf->bw = DA217_GET_BITS(data_array[2], DA217_BW);		
	}

	return rslt;
}

/*!
 * @brief This API reads accel data along with sensor time
 */
static int8_t get_accel_data(struct da217_sensor_data *accel, const struct da217_dev *dev)
{
	int8_t rslt;
	uint8_t data_array[6] = { 0 };

	/* Read the sensor data registers only */
	rslt = da217_get_regs(DA217_ACCEL_DATA_ADDR, data_array, 6, dev);

	if (rslt == DA217_OK) {
		accel->x = ((int16_t)(data_array[1] << 8 | data_array[0]))>> 4;
		accel->y = ((int16_t)(data_array[3] << 8 | data_array[2]))>> 4;
		accel->z = ((int16_t)(data_array[5] << 8 | data_array[4]))>> 4;
	}

	return rslt;
}
