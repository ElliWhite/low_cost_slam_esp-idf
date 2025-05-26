#ifndef _I2C_LCD_H_
#define _I2C_LCD_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "lvgl.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define PIN_NUM_SDA     GPIO_NUM_21
#define PIN_NUM_SCL     GPIO_NUM_22
#define PIN_NUM_RST     -1
#define I2C_HW_ADDR     (0x78 >> 1)
#define I2C_BUS_PORT    0

#define SCREEN_WIDTH    128 // OLED display width, in pixels
#define SCREEN_HEIGHT   64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// To use LV_COLOR_FORMAT_I1, we need an extra buffer to hold the converted data
//extern uint8_t oled_buffer[SCREEN_WIDTH * SCREEN_HEIGHT / 8];
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
extern _lock_t lvgl_api_lock;

#define LCD_PIXEL_CLOCK_HZ    (400 * 1000)

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2
#define LVGL_TICK_PERIOD_MS    5
#define LVGL_PALETTE_SIZE      8
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1000 / CONFIG_FREERTOS_HZ

void initI2C(i2c_master_bus_handle_t *i2c_bus);
void initSSD1306Panel(i2c_master_bus_handle_t i2c_bus, esp_lcd_panel_io_handle_t *io_handle, esp_lcd_panel_handle_t *panel_handle);
void initLVGL(lv_disp_t **display, esp_lcd_panel_io_handle_t *io_handle, esp_lcd_panel_handle_t panel_handle, void *buf);

#ifdef __cplusplus
}
#endif 
#endif // _I2C_LCD_H_