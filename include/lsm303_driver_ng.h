/* lsm303_driver_ng.h - New Driver API (driver_ng) for ESP-IDF 5.x+ */

#ifndef _LSM303_DRIVER_NG_H_
#define _LSM303_DRIVER_NG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "esp_err.h"

#define LSM303_ADDR_ACC 0x19
#define LSM303_ADDR_MAG 0x1E

#define LSM303_MAG_GAUSS_TO_MICROTESLA (100.0f) // Gauss to micro-Tesla multiplier

/* mag data order: high before low (different than accel) */
#define LSM303_MAG_XH 0
#define LSM303_MAG_XL 1
#define LSM303_MAG_ZH 2
#define LSM303_MAG_ZL 3
#define LSM303_MAG_YH 4
#define LSM303_MAG_YL 5

/**
 * Accelerometer modes
 */
typedef enum {
    LSM303_ACC_MODE_NORMAL,          //!< Normal measurement mode; 10-bit
    LSM303_ACC_MODE_HIGH_RESOLUTION, //!< High resolution mode; 12-bit
    LSM303_ACC_MODE_LOW_POWER,       //!< Low power mode; 8-bit
} lsm303_acc_mode_t;

/**
 * Accelerometer data rates
 */
typedef enum {
    LSM303_ODR_POWER_DOWN = 0b0000, //!< Power-down mode
    LSM303_ODR_1_HZ = 0b0001,       //!< Normal / low-power mode (1 Hz)
    LSM303_ODR_10_HZ = 0b0010,      //!< Normal / low-power mode (10 Hz)
    LSM303_ODR_25_HZ = 0b0011,      //!< Normal / low-power mode (25 Hz)
    LSM303_ODR_50_HZ = 0b0100,      //!< Normal / low-power mode (50 Hz)
    LSM303_ODR_100_HZ = 0b0101,     //!< Normal / low-power mode (100 Hz)
    LSM303_ODR_200_HZ = 0b0110,     //!< Normal / low-power mode (200 Hz)
    LSM303_ODR_400_HZ = 0b0111,     //!< Normal / low-power mode (400 Hz)
    LSM303_ODR_1620_HZ = 0b1000,    //!< Low-power mode (1.620 kHz)
    LSM303_ODR_5376_HZ = 0b1001     //!< Normal (1.344 kHz) / low-power mode (5.376 KHz)
} lsm303_acc_rate_t;

/**
 * Accelerometer scales
 */
typedef enum {
    LSM303_ACC_SCALE_2G = 0b00, //!< 1 mg/LSB, +- 2G
    LSM303_ACC_SCALE_4G = 0b01, //!< 2 mg/LSB, +- 4G
    LSM303_ACC_SCALE_8G = 0b10, //!< 4 mg/LSB, +- 8G
    LSM303_ACC_SCALE_16G = 0b11 //!< 12 mg/LSB, +- 16G
} lsm303_acc_scale_t;

/**
 * Magnetometer modes
 */
typedef enum {
    LSM303_MAG_MODE_CONT = 0x00,   //!< Continuous-conversion mode
    LSM303_MAG_MODE_SINGLE = 0x01, //!< Single-conversion mode
    LSM303_MAG_MODE_SLEEP1 = 0x02, //!< Sleep-mode. Device is placed in sleep-mode
    LSM303_MAG_MODE_SLEEP2 = 0x03  //!< Sleep-mode. Device is placed in sleep-mode
} lsm303_mag_mode_t;

/**
 * Magnetometer rates
 */
typedef enum {
    LSM303_MAG_RATE_0_75, //!< 0.75 Hz
    LSM303_MAG_RATE_1_5,  //!< 1.5 Hz
    LSM303_MAG_RATE_3_0,  //!< 3.0 Hz
    LSM303_MAG_RATE_7_5,  //!< 7.5 Hz
    LSM303_MAG_RATE_15,   //!< 15 Hz
    LSM303_MAG_RATE_30,   //!< 30 Hz
    LSM303_MAG_RATE_75,   //!< 75 Hz
    LSM303_MAG_RATE_220   //!< 220 Hz
} lsm303_mag_rate_t;

/**
 * Magnetometer gains
 */
typedef enum {
    LSM303_MAG_GAIN_1_3, //!< +/- 1.3
    LSM303_MAG_GAIN_1_9, //!< +/- 1.9
    LSM303_MAG_GAIN_2_5, //!< +/- 2.5
    LSM303_MAG_GAIN_4_0, //!< +/- 4.0
    LSM303_MAG_GAIN_4_7, //!< +/- 4.7
    LSM303_MAG_GAIN_5_6, //!< +/- 5.6
    LSM303_MAG_GAIN_8_1  //!< +/- 8.1
} lsm303_mag_gain_t;

/**
 * Raw acceleration measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} lsm303_acc_raw_data_t;

/**
 * Raw magnetometer measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} lsm303_mag_raw_data_t;

/**
 * Linear acceleration measurement result: g
 */
typedef struct
{
    float x;
    float y;
    float z;
} lsm303_acc_data_t;

/**
 * Magnetic field measurement result: uT
 */
typedef struct
{
    float x;
    float y;
    float z;
} lsm303_mag_data_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev_accel;
    i2c_master_dev_handle_t i2c_dev_mag;
} lsm303_dev_t;

esp_err_t lsm303_init_desc_ng(lsm303_dev_t *dev, i2c_master_bus_handle_t bus, uint16_t addr_accel, uint16_t addr_mag);
esp_err_t lsm303_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t lsm303_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t value);
esp_err_t lsm303_deinit_desc_ng(lsm303_dev_t *dev, i2c_master_bus_handle_t bus);
esp_err_t lsm303_get_accel(lsm303_dev_t *dev, int16_t *ax, int16_t *ay, int16_t *az);
esp_err_t lsm303_get_mag(lsm303_dev_t *dev, int16_t *mx, int16_t *my, int16_t *mz);
esp_err_t lsm303_acc_raw_to_g(lsm303_dev_t *dev, lsm303_acc_raw_data_t *raw, lsm303_acc_data_t *data);
esp_err_t lsm303_mag_raw_to_uT(lsm303_dev_t *dev, lsm303_mag_raw_data_t *raw, lsm303_mag_data_t *data);

#ifdef __cplusplus
}
#endif 
#endif // _LSM303_DRIVER_NG_H_
