#include "i2c_lcd.h"
#include "chip_utils.h"
#include <sys/param.h>


// To use LV_COLOR_FORMAT_I1, we need an extra buffer to hold the converted data
static uint8_t oled_buffer[SCREEN_WIDTH * SCREEN_HEIGHT / 8];
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
_lock_t lvgl_api_lock;


static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_panel, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t *disp = (lv_disp_t *)user_ctx;
    lv_disp_flush_ready(disp);
    return false;
}

static void lvgl_flush_cb(lv_disp_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    // This is necessary because LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette. Skip the palette here
    // More information about the monochrome, please refer to https://docs.lvgl.io/9.2/porting/display.html#monochrome-displays
    px_map += LVGL_PALETTE_SIZE;

    uint16_t hor_res = lv_disp_get_hor_res(disp);

    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            /* The order of bits is MSB first
                        MSB           LSB
               bits      7 6 5 4 3 2 1 0
               pixels    0 1 2 3 4 5 6 7
                        Left         Right
            */
            bool chroma_color = (px_map[(hor_res >> 3) * y  + (x >> 3)] & 1 << (7 - x % 8));
        
            /* Write to the buffer as required for the display.
            * It writes only 1-bit for monochrome displays mapped vertically.*/
            uint8_t *buf = oled_buffer + hor_res * (y >> 3) + (x);
            if (chroma_color) {
                (*buf) &= ~(1 << (y % 8));
            } else {
                (*buf) |= (1 << (y % 8));
            }
        }
    }

    
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, oled_buffer);

}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_port_task(void *arg)
{
    COND_ESP_LOGI(TAG_LVGL, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();   
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, LVGL_TASK_MIN_DELAY_MS);
        // in case of lvgl display not ready yet
        time_till_next_ms = MIN(time_till_next_ms, LVGL_TASK_MAX_DELAY_MS);
        //COND_ESP_LOGI(TAG_LVGL, "delaying");
        vTaskDelay(pdMS_TO_TICKS(time_till_next_ms));
    }
}



void initI2C(i2c_master_bus_handle_t *i2c_bus) {

    COND_ESP_LOGI(TAG_I2C, "Initialize I2C bus");
    
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = PIN_NUM_SDA,
        .scl_io_num = PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, i2c_bus));

    return;

}


void initSSD1306Panel(i2c_master_bus_handle_t i2c_bus, esp_lcd_panel_io_handle_t *io_handle, esp_lcd_panel_handle_t *panel_handle) {

    COND_ESP_LOGI(TAG_I2C, "Install panel IO");
    
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .scl_speed_hz = LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(i2c_bus, &io_config, io_handle));       // forced to v2 (new implementation using driver_ng)

    COND_ESP_LOGI(TAG_I2C, "Install SSD1306 panel driver");
    //esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = PIN_NUM_RST,
    };

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = SCREEN_HEIGHT,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(*io_handle, &panel_config, panel_handle));
 
    ESP_ERROR_CHECK(esp_lcd_panel_reset(*panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(*panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(*panel_handle, true));

    return;
    

};

//lv_disp_t  display is **display so the pointer from main() gets updated. All references of display are now pointers
void initLVGL(lv_disp_t **display, esp_lcd_panel_io_handle_t *io_handle, esp_lcd_panel_handle_t panel_handle, void *buf){
    COND_ESP_LOGI(TAG_LVGL, "Initialize LVGL");

    lv_init();
    // create a lvgl display
    *display = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);

    // associate the i2c panel handle to the display
    lv_display_set_user_data(*display, panel_handle);
    // create draw buffer
    //void *buf = NULL;
    COND_ESP_LOGI(TAG_LVGL, "Allocate separate LVGL draw buffers");
    // LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette.
    size_t draw_buffer_sz = SCREEN_WIDTH * SCREEN_HEIGHT / 8 + LVGL_PALETTE_SIZE;
    //buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    //assert(buf);

    // LVGL9 suooprt new monochromatic format.
    lv_display_set_color_format(*display, LV_COLOR_FORMAT_I1);
    // initialize LVGL draw buffers
    lv_display_set_buffers(*display, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    // set the callback which can copy the rendered image to an area of the display
    COND_ESP_LOGI(TAG_LVGL, "Setting flush callback: %p", lvgl_flush_cb);
    lv_display_set_flush_cb(*display, lvgl_flush_cb);

    COND_ESP_LOGI(TAG_LVGL, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbas = {
       .on_color_trans_done = notify_lvgl_flush_ready,
    };
  
    /* Register done callback */
    esp_lcd_panel_io_register_event_callbacks(*io_handle, &cbas, *display);

    COND_ESP_LOGI(TAG_LVGL, "Use esp_timer as LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    COND_ESP_LOGI(TAG_LVGL, "Create LVGL task");
    xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
    
    return;
};