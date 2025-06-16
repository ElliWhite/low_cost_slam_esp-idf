#include "stdio.h"
#include "stdint.h"
#include "sdkconfig.h"
#include "string.h"
#include <sys/param.h>
#include <sys/lock.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_chip_info.h"
#include "esp_flash.h"


#include "driver/gpio.h"
#include "chip_utils.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

#include "lvgl.h"
#include "lv_conf.h"

#include "i2c_lcd.h"
#include "math.h"

#include "lsm303_driver_ng.h"
#include "l3gd20_driver_ng.h"

#define PIN_LED_BUILTIN 2



#define LIDAR_PACKET_SIZE   2520
#define LIDAR_BUFFER_SIZE   (LIDAR_PACKET_SIZE * 2) // * 2 so we can be reading from one scan whilst the other fills up and to make sure that somewhere in the buffer is one full packet
#define SYNC_BYTE_0         0xFA    // Start of 42 byte chunk
#define SYNC_BYTE_1         0xA0    // First angle (0 degrees). Angles are 0xA0->0xDB
#define LIDAR_DIST_MAX      500     // max range of LiDAR in mm (set to 500mm for scaling on the small OLED - real max distance is 3500mm)


static const char *TAG = "IMU";


void i2c_scan(i2c_master_bus_handle_t bus) {
    ESP_LOGI(TAG, "Starting I2C bus scan...");

    int devices_found = 0;
    uint8_t dummy = 0x00;
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        i2c_master_dev_handle_t dev_handle;
        i2c_device_config_t dev_cfg = {
            .device_address = addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .scl_speed_hz = 100000,
        };

        esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle);
        if (err == ESP_OK) {
            // Perform zero-length write (just start + address + stop)
            err = i2c_master_transmit(dev_handle, &dummy, 1, pdMS_TO_TICKS(100));
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
                devices_found++;
            }
            i2c_master_bus_rm_device(dev_handle);
        }
    }

    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found!");
    }
}



void app_main(void)
{

    COND_ESP_LOGI(TAG_INFO, "Started main()");
     
    lv_disp_t *display = NULL;
    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_lcd_panel_io_handle_t io_handle = NULL; 
    esp_lcd_panel_handle_t panel_handle = NULL;

    void *buf = NULL;

    initI2C(&i2c_bus);
    initSSD1306Panel(i2c_bus, &io_handle, &panel_handle);
    
    //i2c_scan(i2c_bus);

    size_t draw_buffer_sz = SCREEN_WIDTH * SCREEN_HEIGHT / 8 + LVGL_PALETTE_SIZE;
    buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(buf);

    initLVGL(&display, &io_handle, panel_handle, buf);
    

    COND_ESP_LOGI(TAG_LVGL, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    lv_obj_t *scr = lv_disp_get_scr_act(display);
    COND_ESP_LOGI(TAG_LVGL, "scr: %p", scr);
    lv_obj_t *label = lv_label_create(scr);
    COND_ESP_LOGI(TAG_LVGL, "label: %p", label);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP); // Wrap the text to new line if longer than the line
    lv_obj_set_style_text_font(label, &lv_font_montserrat_10, 0); // set smaller font. Smallest can view comfortably on the SSD1306 is 10px
    lv_label_set_text(label, "Starting scan.");
    /* Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) */
    lv_obj_set_width(label, lv_disp_get_hor_res(display));
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    // Release the mutex
    _lock_release(&lvgl_api_lock);

    // OLED chart creation
    COND_ESP_LOGI(TAG_LVGL, "Display LVGL Chart");
    _lock_acquire(&lvgl_api_lock);
    lv_obj_t *chart = lv_chart_create(scr);
    lv_obj_set_size(chart, 64, 64);         // 64x64 to keep square using maximum space on screen (height = 64px)
    lv_obj_align(chart, LV_ALIGN_CENTER, 0, 0);  // centre with no offset

    lv_obj_set_style_line_width(chart, 0, LV_PART_ITEMS);       // Remove the lines
    lv_obj_set_style_size(chart, 2, 2, LV_PART_INDICATOR);      // Make data points smaller (2 x 2px - smallest visible size on screen)
    lv_chart_set_type(chart, LV_CHART_TYPE_SCATTER);            // Scatter graph
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_X, -LIDAR_DIST_MAX, LIDAR_DIST_MAX);    
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -LIDAR_DIST_MAX, LIDAR_DIST_MAX);
    lv_chart_set_point_count(chart, 360);
    lv_chart_series_t * chart_data = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);     // add a data series
        
    _lock_release(&lvgl_api_lock);

    /* Configure UART for LiDAR */
    const uart_port_t uart_num = UART_NUM_2;

    uart_config_t lidar_uart_config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    COND_ESP_LOGI(TAG_INFO, "Configuring UART");

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &lidar_uart_config));

    // Set UART pins(TX: IO17, RX: IO16, No RTS or CTS as HW flow is disabled)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    const int uart_rx_buffer_size = LIDAR_BUFFER_SIZE; 
    //const int uart_tx_buffer_size = (128 * 2);  // Don't need a tx buffer as are just sending start and stop commands
    //QueueHandle_t uart_queue;
    COND_ESP_LOGI(TAG_INFO, "Installing UART driver");
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_rx_buffer_size, \
                                            0, 0, NULL, 0));

    // Write data to UART. Start LiDAR
    COND_ESP_LOGI(TAG_UART_TX, "Starting LiDAR");
    char* test_str = "b";
    uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));
 
    // Reset the LED pin
    gpio_reset_pin(PIN_LED_BUILTIN);

    // Set input/output so state can be read with get_level
    gpio_set_direction(PIN_LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    // Configure a buffer for the incoming data
    uint8_t *lidar_data_tmp = (uint8_t *) malloc(LIDAR_BUFFER_SIZE);      // temporary buffer for reading 2 full packets to find sync bytes
    uint8_t *lidar_data = (uint8_t *) malloc(LIDAR_BUFFER_SIZE + LIDAR_PACKET_SIZE); // rolling buffer = 3 total packets
    uint32_t lidar_buf_capacity = LIDAR_BUFFER_SIZE + LIDAR_PACKET_SIZE;
    int lidar_data_len = 0;
  
    int num_found_packets = 0;


    static lsm303_dev_t lsm303;
    
    ESP_LOGI(TAG, "Initialising LSM303");
    ESP_ERROR_CHECK(lsm303_init_desc_ng(&lsm303, i2c_bus, LSM303_ADDR_ACC, LSM303_ADDR_MAG));
    // CTRL_REG1_A - Power on, 50 Hz data rate, enable all axes
    ESP_ERROR_CHECK(lsm303_write_register(lsm303.i2c_dev_accel, 0x20, 0x27));

    // CRA_REG_M - Temperature sensor disable, 15 Hz data rate
    ESP_ERROR_CHECK(lsm303_write_register(lsm303.i2c_dev_mag, 0x00, 0x14));

    // CRB_REG_M - Gain setting, Sensor input field range +/- 1.3 Gauss
    ESP_ERROR_CHECK(lsm303_write_register(lsm303.i2c_dev_mag, 0x01, 0x20));

    // Gain setting: ±4 Gauss
    ESP_ERROR_CHECK(lsm303_write_register(lsm303.i2c_dev_mag, 0x01, 0x40));

    // MR_REG_M - Continuous-conversion mode
    ESP_ERROR_CHECK(lsm303_write_register(lsm303.i2c_dev_mag, 0x02, 0x00));


    static l3gd20_dev_t l3gd20;

    ESP_LOGI(TAG, "Initialising L3GD20");
    ESP_ERROR_CHECK(l3gd20_init_desc_ng(&l3gd20, i2c_bus, L3GD20_ADDR));

    uint8_t who_am_i;
    ESP_ERROR_CHECK(l3gd20_read_register(l3gd20.i2c_dev_gyro, L3GD20_REG_WHO_AM_I, &who_am_i, 1));
    ESP_LOGI(TAG, "L3GD20 WHO_AM_I: 0x%02X", who_am_i);

    // CTRL_REG4 - Set scale 500 dps
    ESP_ERROR_CHECK(l3gd20_write_register(l3gd20.i2c_dev_gyro, L3GD20_REG_CTRL_REG4, (L3GD20_SCALE_500 << 4)));

    // CTRL_REG1 - Set datarate and bandwidth L3GD20_DRBW_800_30 & enable all axes & enable normal power mode
    ESP_ERROR_CHECK(l3gd20_write_register(l3gd20.i2c_dev_gyro, L3GD20_REG_CTRL_REG1, (L3GD20_DRBW_800_30 << 4) | L3GD20_ENABLE_ALL_AXES | (L3GD20_ENABLE_NORMAL_POWER_MODE << 3)));


    while (1) {
        int16_t ax, ay, az, mx, my, mz;
        lsm303_acc_raw_data_t acc_raw_data;
        lsm303_mag_raw_data_t mag_raw_data;
        lsm303_acc_data_t acc_data;
        lsm303_mag_data_t mag_data;

        l3gd20_raw_data_t gyro_raw_data;
        l3gd20_data_t gyro_data;

        if (lsm303_get_accel(&lsm303, &ax, &ay, &az) == ESP_OK) {
            ESP_LOGI(TAG, "Accel: X=%d Y=%d Z=%d", ax, ay, az);

            acc_raw_data.x = ax;
            acc_raw_data.y = ay;
            acc_raw_data.z = az;
            lsm303_acc_raw_to_g(&lsm303, &acc_raw_data, &acc_data);
            ESP_LOGI(TAG, "Accel_in_g: X=%f Y=%f Z=%f", acc_data.x, acc_data.y, acc_data.z);
        } else {
            ESP_LOGE(TAG, "Failed to read accel");
        }

        if (lsm303_get_mag(&lsm303, &mx, &my, &mz) == ESP_OK) {
            ESP_LOGI(TAG, "Mag: X=%d Y=%d Z=%d", mx, my, mz);

            mag_raw_data.x = mx;
            mag_raw_data.y = my;
            mag_raw_data.z = mz;
            lsm303_mag_raw_to_uT(&lsm303, &mag_raw_data, &mag_data);
            ESP_LOGI(TAG, "Mag_in_uT: X=%f Y=%f Z=%f", mag_data.x, mag_data.y, mag_data.z);

        } else {
            ESP_LOGE(TAG, "Failed to read mag");
        }

        if (l3gd20_get_raw_data(&l3gd20, &gyro_raw_data) == ESP_OK) {

            ESP_LOGI(TAG, "Gyro: X=%d Y=%d Z=%d", gyro_raw_data.x, gyro_raw_data.y, gyro_raw_data.z);

            l3gd20_raw_to_dps(&l3gd20, &gyro_raw_data, &gyro_data);
            ESP_LOGI(TAG, "Gyro_in_dps: X=%f Y=%f Z=%f", gyro_data.x, gyro_data.y, gyro_data.z);

        } else {
            ESP_LOGE(TAG, "Failed to read gyro");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }



    

    while(1){

        vTaskDelay(10 / portTICK_PERIOD_MS); // Short delay of 10ms to allow other tasks like IDLE0 to run, otherwise we get a task watchdog trigger

        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length));
        COND_ESP_LOGI(TAG_UART_RX, "UART buffer size: %i", length);
        if (length > 0) { // buffer contains some data

            // check if amount of data in buffer is less than 2x full packets
            if(length < LIDAR_BUFFER_SIZE){
                length = uart_read_bytes(uart_num, lidar_data_tmp, length, pdMS_TO_TICKS(100));     // Read from the buffer into temporary storage
            } else {
                length = uart_read_bytes(uart_num, lidar_data_tmp, LIDAR_BUFFER_SIZE, pdMS_TO_TICKS(100));     // Read 2x packet size from the buffer into temporary storage
            }
            COND_ESP_LOGI(TAG_UART_RX, "Read %i from buffer", length);

            // Append new data to final buffer if room
            if ((lidar_data_len + length) <= lidar_buf_capacity) {
                COND_ESP_LOGI(TAG_UART_RX, "Adding %i bytes from temp buffer to final buffer", length);
                memcpy(&lidar_data[lidar_data_len], lidar_data_tmp, length);
                lidar_data_len += length;
            } else {
                // Overflow protection: drop oldest data
                int overflow = (lidar_data_len + length) - lidar_buf_capacity;
                COND_ESP_LOGI(TAG_UART_RX, "Overflow protection. Length: %i", overflow);
                memmove(lidar_data, &lidar_data[overflow], lidar_data_len - overflow);
                memcpy(&lidar_data[lidar_data_len - overflow], lidar_data_tmp, length);
                lidar_data_len = lidar_buf_capacity;
            }

            // Scan for sync byte and process packets
            COND_ESP_LOGI(TAG_UART_RX, "Finding sync bytes");
            for (int i = 0; i <= lidar_data_len - LIDAR_PACKET_SIZE; i++) {
                if (lidar_data[i] == SYNC_BYTE_0 && lidar_data[i+1] == SYNC_BYTE_1) {
                    COND_ESP_LOGI(TAG_UART_RX, "Found sync bytes");
                    if (i + LIDAR_PACKET_SIZE <= lidar_data_len) {
                        //process_packet(&lidar_data[i]);
                        num_found_packets += 1;
                        COND_ESP_LOGI(TAG_INFO, "Found: %i LiDAR packets", num_found_packets);

                        uint8_t good_packets_cnt = 0;

                        // Checksum. Add first 40 bytes of packet, then do 0xFF - chksm. If equals either byte 41 or 42 in packet, data valid.
                        for(uint16_t x = i; x < (2520 + i); x = x + 42){

                            uint8_t chksm = 0;

                            for(uint16_t j = 0; j < 40; j++){
                                chksm = chksm + lidar_data[x+j];
                            }

                            chksm = 0xFF - chksm;
                            
                            if(chksm == lidar_data[x+40]){
                                good_packets_cnt += 1;
                            }

                        }

                        // Quit if haven't received full packet of good data
                        if(good_packets_cnt != 60){
                            COND_ESP_LOGI(TAG_INFO, "Packet was bad - skipping..");
                            // Move remaining data forward
                            int remaining = lidar_data_len - (i + LIDAR_PACKET_SIZE);
                            memmove(lidar_data, &lidar_data[i + LIDAR_PACKET_SIZE], remaining);
                            lidar_data_len = remaining;
                            i = -1;  // Restart scan from beginning
                            break;
                        }


                        /* Process data for graph display */
                        
                        double lidar_angle = 0;
                        lv_point_t lidar_points[360] = {};
                        // now have full data packet, loop over each 42 byte packet
                        for(uint16_t x = i; x < (LIDAR_PACKET_SIZE + i); x = x + 42){
                            for (int j = x + 6; j < x + 40; j = j + 6) {
                                int32_t lidar_distance = (lidar_data[j+1] << 8) + lidar_data[j];
                                // Convert from "navigation" to "math" angle (converting "north" as 0 degrees to 90 in math degrees)
                                double math_angle = fmod((90.0 - lidar_angle + 360.0), 360.0);
                                
                                lidar_points[(int16_t)lidar_angle].x = (int32_t)(lidar_distance * cos(math_angle * (double)M_PI / 180.0));

                                // Cap x co-ord if calculated distance is bigger than max distance
                                if(lidar_points[(int16_t)lidar_angle].x > LIDAR_DIST_MAX){
                                    lidar_points[(int16_t)lidar_angle].x = LIDAR_DIST_MAX;
                                }

                                lidar_points[(int16_t)lidar_angle].y = (int32_t)(lidar_distance * sin(math_angle * (double)M_PI / 180.0));

                                // Cap y co-ord if calculated distance is bigger than max distance
                                if(lidar_points[(int16_t)lidar_angle].y > LIDAR_DIST_MAX){
                                    lidar_points[(int16_t)lidar_angle].y = LIDAR_DIST_MAX;
                                }
                                
                                lidar_angle += 1;
                                                        
                            }
                        }
                        

                        // Lock the mutex due to the LVGL APIs are not thread-safe
                        _lock_acquire(&lvgl_api_lock);
                        lv_label_set_text(label, ""); // clear text using same "label" as previously created
                        char text[128];
                        snprintf(text, sizeof(text), "LP %i", num_found_packets);
                        lv_label_set_text(label, text);
                        lv_chart_set_point_count(chart, 0);     // clear existing points
                        lv_chart_set_point_count(chart, 360);    // reset point coint 
                        // Add lidar data to chart
                        for(uint16_t z = 0; z < 360; z++) {
                           lv_chart_set_next_value2(chart, chart_data, lidar_points[z].x, lidar_points[z].y); 
                        };
                        // Release the mutex
                        _lock_release(&lvgl_api_lock);
    

                        // Toggle the LED (set to the opposite level)
                        gpio_set_level(PIN_LED_BUILTIN, (gpio_get_level(PIN_LED_BUILTIN) == 0) ? 1 : 0);
                        // Move remaining data forward
                        int remaining = lidar_data_len - (i + LIDAR_PACKET_SIZE);
                        memmove(lidar_data, &lidar_data[i + LIDAR_PACKET_SIZE], remaining);
                        lidar_data_len = remaining;
                        i = -1;  // Restart scan from beginning
                    }
                }

            }
        } 
    }

    restartESP(2);      // Never reached
    
}