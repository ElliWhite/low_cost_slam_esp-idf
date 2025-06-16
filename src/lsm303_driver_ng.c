/* lsm303_driver_ng.c - New Driver API (driver_ng) for ESP-IDF 5.x+ */

#include "lsm303_driver_ng.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "LSM303_NG"



esp_err_t lsm303_init_desc_ng(lsm303_dev_t *dev, i2c_master_bus_handle_t bus, uint16_t addr_accel, uint16_t addr_mag) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    i2c_device_config_t accel_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_accel,
        .scl_speed_hz = 400000,
    };

    i2c_device_config_t mag_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_mag,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &accel_cfg, &dev->i2c_dev_accel));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &mag_cfg, &dev->i2c_dev_mag));

    return ESP_OK;
}

esp_err_t lsm303_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
}

esp_err_t lsm303_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t value) {
    uint8_t buf[2] = { reg_addr, value };
    return i2c_master_transmit(dev_handle, buf, 2, pdMS_TO_TICKS(1000));
}

esp_err_t lsm303_deinit_desc_ng(lsm303_dev_t *dev, i2c_master_bus_handle_t bus) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev->i2c_dev_accel));
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev->i2c_dev_mag));

    return ESP_OK;
}

esp_err_t lsm303_get_accel(lsm303_dev_t *dev, int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t data[6];
    esp_err_t res = lsm303_read_register(dev->i2c_dev_accel, 0x28 | 0x80, data, 6);
    if (res != ESP_OK) return res;

    *ax = (int16_t)((data[1] << 8) | data[0]);
    *ay = (int16_t)((data[3] << 8) | data[2]);
    *az = (int16_t)((data[5] << 8) | data[4]);

    return ESP_OK;
}

esp_err_t lsm303_get_mag(lsm303_dev_t *dev, int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t data[6];
    esp_err_t res = lsm303_read_register(dev->i2c_dev_mag, 0x03, data, 6);
    if (res != ESP_OK) return res;

    *mx = (int16_t)(data[LSM303_MAG_XL] | (data[LSM303_MAG_XH] << 8));
    *my = (int16_t)(data[LSM303_MAG_YL] | (data[LSM303_MAG_YH] << 8));
    *mz = (int16_t)(data[LSM303_MAG_ZL] | (data[LSM303_MAG_ZH] << 8));

    return ESP_OK;
}

esp_err_t lsm303_acc_raw_to_g(lsm303_dev_t *dev, lsm303_acc_raw_data_t *raw, lsm303_acc_data_t *data)
{
    
    static const float lsb[][4] = {
        [LSM303_ACC_MODE_NORMAL] = {
            [LSM303_ACC_SCALE_2G] = 0.0039,
            [LSM303_ACC_SCALE_4G] = 0.00782,
            [LSM303_ACC_SCALE_8G] = 0.01563,
            [LSM303_ACC_SCALE_16G] = 0.0469
        },
        [LSM303_ACC_MODE_HIGH_RESOLUTION] = {
            [LSM303_ACC_SCALE_2G] = 0.00098,
            [LSM303_ACC_SCALE_4G] = 0.00195,
            [LSM303_ACC_SCALE_8G] = 0.0039,
            [LSM303_ACC_SCALE_16G] = 0.01172
        },
        [LSM303_ACC_MODE_LOW_POWER] = {
            [LSM303_ACC_SCALE_2G] = 0.01563,
            [LSM303_ACC_SCALE_4G] = 0.03126,
            [LSM303_ACC_SCALE_8G] = 0.06252,
            [LSM303_ACC_SCALE_16G] = 0.18758
        },
    };
    static const int shift[] = {
        [LSM303_ACC_MODE_NORMAL] = 6,          // 10-bit
        [LSM303_ACC_MODE_HIGH_RESOLUTION] = 4, // 12-bit
        [LSM303_ACC_MODE_LOW_POWER] = 8        // 8-bit
    };
    data->x = (raw->x >> shift[0]) * lsb[0][0];
    data->y = (raw->y >> shift[0]) * lsb[0][0];
    data->z = (raw->z >> shift[0]) * lsb[0][0];
    return ESP_OK;
}

esp_err_t lsm303_mag_raw_to_uT(lsm303_dev_t *dev, lsm303_mag_raw_data_t *raw, lsm303_mag_data_t *data)
{

    /* gain for XY axis is different from Z axis */
    enum { GAIN_XY = 0, GAIN_Z = 1 };
    /*  { xy , z} */
    static const float gauss_lsb[][2] = {
        [LSM303_MAG_GAIN_1_3] = { 1100, 980 },
        [LSM303_MAG_GAIN_1_9] = { 855, 760 },
        [LSM303_MAG_GAIN_2_5] = { 670, 600 },
        [LSM303_MAG_GAIN_4_0] = { 450, 400 },
        [LSM303_MAG_GAIN_4_7] = { 400, 355 },
        [LSM303_MAG_GAIN_5_6] = { 330, 295 },
        [LSM303_MAG_GAIN_8_1] = { 230, 205 },
    };

    data->x = (float)raw->x / gauss_lsb[0][GAIN_XY] * LSM303_MAG_GAUSS_TO_MICROTESLA;
    data->y = (float)raw->y / gauss_lsb[0][GAIN_XY] * LSM303_MAG_GAUSS_TO_MICROTESLA;
    data->z = (float)raw->z / gauss_lsb[0][GAIN_Z] * LSM303_MAG_GAUSS_TO_MICROTESLA;
    return ESP_OK;
}
