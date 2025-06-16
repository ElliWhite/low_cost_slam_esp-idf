/* lsm303_driver_ng.h - New Driver API (driver_ng) for ESP-IDF 5.x+ */

#ifndef _L3GD20_DRIVER_NG_H_
#define _L3GD20_DRIVER_NG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * Default I2C address
 */
#define L3GD20_ADDR  0x69

/* identification number */
#define L3GD20_WHO_AM_I_ID   0xD4

/* registers addresses */
#define L3GD20_REG_WHO_AM_I      0x0F
#define L3GD20_REG_CTRL_REG1     0x20
#define L3GD20_REG_CTRL_REG2     0x21
#define L3GD20_REG_CTRL_REG3     0x22
#define L3GD20_REG_CTRL_REG4     0x23
#define L3GD20_REG_CTRL_REG5     0x24
#define L3GD20_REG_REFERENCE     0x25
#define L3GD20_REG_OUT_TEMP      0x26
#define L3GD20_REG_STATUS_REG    0x27
#define L3GD20_REG_OUT_X_L       0x28
#define L3GD20_REG_OUT_X_H       0x29
#define L3GD20_REG_OUT_Y_L       0x2A
#define L3GD20_REG_OUT_Y_H       0x2B
#define L3GD20_REG_OUT_Z_L       0x2C
#define L3GD20_REG_OUT_Z_H       0x2D
#define L3GD20_REG_FIFO_CTRL_REG 0x2E
#define L3GD20_REG_FIFO_SRC_REG  0x2F
#define L3GD20_REG_INT1_CFG      0x30
#define L3GD20_REG_INT1_SRC      0x31
#define L3GD20_REG_INT1_THS_XH   0x32
#define L3GD20_REG_INT1_THS_XL   0x33
#define L3GD20_REG_INT1_THS_YH   0x34
#define L3GD20_REG_INT1_THS_YL   0x35
#define L3GD20_REG_INT1_THS_ZH   0x36
#define L3GD20_REG_INT1_THS_ZL   0x37
#define L3GD20_REG_INT1_DURATION 0x38

#define L3GD20_AUTOINCREMENT 0x80

// CTRL_REG1:
#define L3GD20_ENABLE_ALL_AXES          0x07
#define L3GD20_ENABLE_NORMAL_POWER_MODE 0x01
#define L3GD20_DTBW_MASK                0xF0

// CTRL_REG3:
#define L3GD20_ENABLE_DR_INTERRUPT 0x08

// CTRL_REG4:
#define L3GD20_ENABLE_BDU 0x80
#define L3GD20_SCALE_MASK 0x30

// STATUS_REG:
#define L3GD20_STATUS_ZYXOR 0x80
#define L3GD20_STATUS_ZOR   0x40
#define L3GD20_STATUS_YOR   0x20
#define L3GD20_STATUS_XOR   0x10
#define L3GD20_STATUS_ZYXDA 0x08
#define L3GD20_STATUS_ZDA   0x04
#define L3GD20_STATUS_YDA   0x02
#define L3GD20_STATUS_XDA   0x01

/**
 * Sensor type
 */
typedef enum { L3GD20_TYPE_L3G4200D, L3GD20_TYPE_L3GD20, L3GD20_TYPE_UNKNOWN } L3GD20_sensor_type_t;

/**
 * Scales
 */
typedef enum {
    L3GD20_SCALE_250 = 0b00, // full scale to 250 dps
    L3GD20_SCALE_500 = 0b01, // full scale to 500 dps
    L3GD20_SCALE_2000 = 0b10 // full scale to 2000 dps
} l3gd20_scale_t;

/**
 * Data rates and bandwith
 */
typedef enum {
    L3GD20_DRBW_100_125 = 0, // 100 Hz ODR, 12.5 Hz bandwidth
    L3GD20_DRBW_100_25a,
    L3GD20_DRBW_100_25b,
    L3GD20_DRBW_100_25c,
    L3GD20_DRBW_200_125,
    L3GD20_DRBW_200_25,
    L3GD20_DRBW_200_50,
    L3GD20_DRBW_200_70,
    L3GD20_DRBW_400_20,
    L3GD20_DRBW_400_25,
    L3GD20_DRBW_400_50,
    L3GD20_DRBW_400_110,
    L3GD20_DRBW_800_30,
    L3GD20_DRBW_800_35,
    L3GD20_DRBW_800_50,
    L3GD20_DRBW_800_110 // 800 Hz ODR, 110 Hz bandwidth
} l3gd20_drbw_t;

/**
 * Raw measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} l3gd20_raw_data_t;

/**
 * Measurement result, degrees per second
 */
typedef struct
{
    float x;
    float y;
    float z;
} l3gd20_data_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev_gyro;
} l3gd20_dev_t;

esp_err_t l3gd20_init_desc_ng(l3gd20_dev_t *dev, i2c_master_bus_handle_t bus, uint16_t addr_gyro);
esp_err_t l3gd20_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t l3gd20_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t value);
esp_err_t l3gd20_deinit_desc_ng(l3gd20_dev_t *dev, i2c_master_bus_handle_t bus);
esp_err_t l3gd20_get_raw_data(l3gd20_dev_t *dev, l3gd20_raw_data_t *raw);
esp_err_t l3gd20_raw_to_dps(l3gd20_dev_t *dev, l3gd20_raw_data_t *raw, l3gd20_data_t *data);

#ifdef __cplusplus
}
#endif 
#endif // _L3GD20_DRIVER_NG_H_
