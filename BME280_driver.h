/*
 * BME280_driver.h
 *
 *  Created on: Feb 2018
 *      Author: lourw and joe
 */

#ifndef BME280_DRIVER_H_
#define BME280_DRIVER_H_

#include "bme280.h"
#include "I2c.h"
#include "Timer32_driver.h"


// Configures I2C and initializes BME280 in Normal Mode
int8_t BME280_init(struct bme280_dev *dev);
// Fills data structure with compensated data from sensor
int8_t BME280_read(struct bme280_dev *dev, struct bme280_data *data);


#endif /* BME280_DRIVER_H_ */
