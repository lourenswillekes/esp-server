/*
 * BME280_driver.c
 *
 *  Created on: Feb 2018
 *      Author: lourw and joe
 */

#include "BME280_driver.h"


// configures i2c and initializes bme280 in Normal Mode
int8_t BME280_init(struct bme280_dev *dev) {

    int8_t res;
    int settings_select;

    I2C_Init();
    Timer32_init();

    // configure I2C
    dev->dev_id = BME280_I2C_ADDR_PRIM;
    dev->intf = BME280_I2C_INTF;
    dev->read = (bme280_com_fptr_t) I2C_Read_String;
    dev->write = (bme280_com_fptr_t) I2C_Write_String;
    dev->delay_ms = (bme280_delay_fptr_t) Timer32_waitms;
    // and initiate it
    res = bme280_init(dev);

    // recommended settings
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;
    dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;
    // settings select
    settings_select = BME280_OSR_PRESS_SEL;
    settings_select |= BME280_OSR_TEMP_SEL;
    settings_select |= BME280_OSR_HUM_SEL;
    settings_select |= BME280_STANDBY_SEL;
    settings_select |= BME280_FILTER_SEL;
    // and set all the things
    res = bme280_set_sensor_settings(settings_select, dev);
    res = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

    return res;

}

// fills data structure with compensated data from sensor
int8_t BME280_read(struct bme280_dev *dev, struct bme280_data *data) {

    int8_t res;

    res = bme280_get_sensor_data(BME280_ALL, data, dev);

    return res;

}
