/* l3gd20_driver_ng.c - New Driver API (driver_ng) for ESP-IDF 5.x+ */

#include "l3gd20_driver_ng.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "l3gd20_NG"




esp_err_t l3gd20_init_desc_ng(l3gd20_dev_t *dev, i2c_master_bus_handle_t bus, uint16_t addr_gyro) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    i2c_device_config_t gyro_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_gyro,
        .scl_speed_hz = 400000,
    };


    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &gyro_cfg, &dev->i2c_dev_gyro));

    return ESP_OK;
}

esp_err_t l3gd20_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
}

esp_err_t l3gd20_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t value) {
    uint8_t buf[2] = { reg_addr, value };
    return i2c_master_transmit(dev_handle, buf, 2, pdMS_TO_TICKS(1000));
}

esp_err_t l3gd20_deinit_desc_ng(l3gd20_dev_t *dev, i2c_master_bus_handle_t bus) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev->i2c_dev_gyro));

    return ESP_OK;
}

esp_err_t l3gd20_get_raw_data(l3gd20_dev_t *dev, l3gd20_raw_data_t *raw) {
    uint8_t data[6];
    esp_err_t res = l3gd20_read_register(dev->i2c_dev_gyro, L3GD20_REG_OUT_X_L | L3GD20_AUTOINCREMENT, data, 6);
    if (res != ESP_OK) return res;
    raw->x = (int16_t)(data[1] << 8 | data[0]);
    raw->y = (int16_t)(data[3] << 8 | data[2]);
    raw->z = (int16_t)(data[5] << 8 | data[4]);

    return ESP_OK;
}


esp_err_t l3gd20_raw_to_dps(l3gd20_dev_t *dev, l3gd20_raw_data_t *raw, l3gd20_data_t *data) {

    /* sensitivity factors, datasheet pg. 9 */
    static const float sensivity_factors[] = {
        [L3GD20_SCALE_250] = 0.00875, // 8.75 mdps/digit
        [L3GD20_SCALE_500] = 0.0175,  // 17.5 mdps/digit
        [L3GD20_SCALE_2000] = 0.0700  // 70.0 mdps/digit
    };
    data->x = raw->x * sensivity_factors[1];
    data->y = raw->y * sensivity_factors[1];
    data->z = raw->z * sensivity_factors[1];
    return ESP_OK;

}
