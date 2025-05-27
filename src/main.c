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

#define PIN_LED_BUILTIN 2



#define LIDAR_PACKET_SIZE 2520
#define LIDAR_BUFFER_SIZE (LIDAR_PACKET_SIZE * 2) // * 2 so we can be reading from one scan whilst the other fills up and to make sure that somewhere in the buffer is one full packet
#define SYNC_BYTE_0 0xFA    // Start of 42 byte chunk
#define SYNC_BYTE_1 0xA0    // First angle (0 degrees). Angles are 0xA0->0xDB

/*
static uint8_t my_oled_buffer[SCREEN_WIDTH * SCREEN_HEIGHT / 8];
static _lock_t my_lvgl_api_lock;


bool my_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_panel, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t *disp = (lv_disp_t *)user_ctx;
    lv_disp_flush_ready(disp);
    return false;
}


static void my_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);

    // This is necessary because LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette. Skip the palette here
    // More information about the monochrome, please refer to https://docs.lvgl.io/9.2/porting/display.html#monochrome-displays
    px_map += LVGL_PALETTE_SIZE;

    uint16_t hor_res = lv_disp_get_physical_hor_res(disp);
    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            // The order of bits is MSB first
            //            MSB           LSB
            //   bits      7 6 5 4 3 2 1 0
            //   pixels    0 1 2 3 4 5 6 7
            //            Left         Right
            
            bool chroma_color = (px_map[(hor_res >> 3) * y  + (x >> 3)] & 1 << (7 - x % 8));

            // Write to the buffer as required for the display.
            // It writes only 1-bit for monochrome displays mapped vertically.
            uint8_t *buf = my_oled_buffer + hor_res * (y >> 3) + (x);
            if (chroma_color) {
                (*buf) &= ~(1 << (y % 8));
            } else {
                (*buf) |= (1 << (y % 8));
            }
        }
    }
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, my_oled_buffer);
}


void my_lvgl_port_task(void *arg)
{
    COND_ESP_LOGI(TAG_LVGL, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1) {
        //COND_ESP_LOGI(TAG_LVGL, "Trying to aquire lock");
        _lock_acquire(&my_lvgl_api_lock);
        //COND_ESP_LOGI(TAG_LVGL, "Aquired");
        time_till_next_ms = lv_timer_handler();
        //COND_ESP_LOGI(TAG_LVGL, "timer handled");
        _lock_release(&my_lvgl_api_lock);
        //COND_ESP_LOGI(TAG_LVGL, "released lock");
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, LVGL_TASK_MIN_DELAY_MS);
        // in case of lvgl display not ready yet
        time_till_next_ms = MIN(time_till_next_ms, LVGL_TASK_MAX_DELAY_MS);
        //COND_ESP_LOGI(TAG_LVGL, "delaying");
        vTaskDelay(pdMS_TO_TICKS(time_till_next_ms));
    }
}
*/

/*// Function to override the point size to have a radius of 2px - DOESN'T CURRENTLY WORK so disabled cb register
static void draw_event_cb(lv_event_t * e) {

    

    lv_draw_task_t *dsc = lv_event_get_draw_task(e);
    lv_draw_dsc_base_t *base_dsc = (lv_draw_dsc_base_t *)lv_draw_task_get_draw_dsc(dsc);
    
    lv_obj_t *chart = lv_event_get_target_obj(e);

    lv_chart_series_t *ser = lv_chart_get_series_next(chart, NULL);

    lv_layer_t *draw_ctx = lv_event_get_layer(e);

    if(base_dsc->part != LV_PART_INDICATOR) return;

    uint32_t point_index = base_dsc->id2;



   
    uint32_t p_cnt = lv_chart_get_point_count(chart);
    
    for (uint32_t i = 0; i < p_cnt; i++){
        lv_point_t p;
        lv_chart_get_point_pos_by_id(chart, ser, i, &p);
        
        lv_layer_t *layer = lv_event_get_layer(e);

        // Draw a small point manually
        lv_draw_rect_dsc_t *rect_dsc = (lv_draw_rect_dsc_t *)lv_draw_task_get_draw_dsc(dsc);
        lv_draw_rect_dsc_t draw_rect_dsc;
        lv_draw_rect_dsc_init(&draw_rect_dsc);

        lv_area_t chart_area;
        lv_obj_get_coords(chart, &chart_area);
        lv_area_t rect_area;
        rect_area.x1 = chart_area.x1 + p.x - 20;
        rect_area.y1 = chart_area.y1 + p.x + 20;
        rect_area.x2 = chart_area.x1 + p.y - 20;
        rect_area.y2 = chart_area.y1 + p.y + 20;
    }

    //lv_draw_rect(layer, &draw_rect_dsc, &rect_area);
        
    
    
    rect_dsc->bg_opa = LV_OPA_COVER;
    rect_dsc->radius = 20;
    rect_dsc->border_width = 0;
    rect_dsc->border_side = LV_BORDER_SIDE_NONE;
    rect_dsc->outline_width = 0;
    rect_dsc->shadow_width = 0;
    
    uint32_t start_point = lv_chart_get_x_start_point(chart, ser);
    uint32_t cnt = lv_chart_get_point_count(chart);
    uint32_t p_act = (start_point + base_dsc->id2) % cnt;
    int32_t *x_array = lv_chart_get_x_array(chart, ser);
    int32_t *y_array = lv_chart_get_y_array(chart, ser);

    uint32_t radius_x = (uint32_t)((x_array[p_act] * 40));
    uint32_t radius_y = (uint32_t)((y_array[p_act] * 2));

    rect_dsc->radius = radius_x;

    

    

    lv_area_t chart_area;
    lv_obj_get_coords(chart, &chart_area);

    lv_area_t area;
    area.x1 = chart_area.x1 - 2;
    area.y1 = chart_area.y1 - 2;
    area.x2 = chart_area.x1 + 2;
    area.y2 = chart_area.y1 + 2;
    



    //lv_draw_rect(draw_ctx, rect_dsc, &area);
    
    lv_draw_triangle_dsc_t triangle;
    lv_draw_triangle_dsc_init(&triangle);
    triangle.p[0].x = 10;
    triangle.p[0].y = 10;
    triangle.p[1].x = 80;
    triangle.p[1].y = 80;
    triangle.p[2].x = 20;
    triangle.p[2].y = 100;
    lv_draw_triangle(draw_ctx, &triangle);
    */
    
    
    /*
    if(base_dsc->part == LV_PART_ITEMS) {
        // Customize how each point is drawn
        lv_draw_rect_dsc_t draw_dsc;
        lv_draw_rect_dsc_init(&draw_dsc);
        draw_dsc.radius = LV_RADIUS_CIRCLE;
        draw_dsc.bg_opa = LV_OPA_COVER;

        ser->x_pointslv_chart_series_t
        

        // This is the actual point position from the chart
        lv_coord_t cx = (dsc->area->x1 + dsc->area->x2) / 2;
        lv_coord_t cy = (dsc->area->y1 + dsc->area->y2) / 2;

        lv_area_t chart_area;
        lv_obj_get_coords(chart, &chart_area);

        lv_area_t area;
        area.x1 = chart_area.x1 - 2;LV_EVENT_DRAW_POST_END
        area.y1 = chart_area.y1 - 2;
        area.x2 = chart_area.x1 + 2;
        area.y2 = chart_area.y1 + 2;

        lv_layer_t *layer = lv_event_get_layer(e);

        lv_draw_rect(layer, &draw_dsc, &area);
        
    }
    
}*/

void app_main(void)
{

    COND_ESP_LOGI(TAG_INFO, "Started main()");
 
/* ORIGINAL EXAMPLE CODE 

    ESP_LOGI(TAG_INFO,"Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = PIN_NUM_SDA,
        .scl_io_num = PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG_INFO,"Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .scl_speed_hz = LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(TAG_INFO,"Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = PIN_NUM_RST,
    };

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = SCREEN_HEIGHT,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));


    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


    ESP_LOGI(TAG_INFO,"Initialize LVGL");
    lv_init();
    // create a lvgl display
    lv_display_t *display = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
    // associate the i2c panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // create draw buffer
    void *buf = NULL;
    ESP_LOGI(TAG_INFO,"Allocate separate LVGL draw buffers");
    // LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette.
    size_t draw_buffer_sz = SCREEN_WIDTH * SCREEN_HEIGHT / 8 + LVGL_PALETTE_SIZE;
    buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(buf);

    // LVGL9 suooprt new monochromatic format.
    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, my_lvgl_flush_cb);

    ESP_LOGI(TAG_INFO,"Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = my_notify_lvgl_flush_ready,
    };
    //Register done callback 
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display);

    ESP_LOGI(TAG_INFO,"Use esp_timer as LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG_INFO,"Create LVGL task");
    xTaskCreate(my_lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);


*/

    
    lv_disp_t *display = NULL;
    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_lcd_panel_io_handle_t io_handle = NULL; 
    esp_lcd_panel_handle_t panel_handle = NULL;
    //esp_lcd_panel_io_callbacks_t cbs = {
    //   .on_color_trans_done = notify_lvgl_flush_ready,
    //};

    void *buf = NULL;

    initI2C(&i2c_bus);
    initSSD1306Panel(i2c_bus, &io_handle, &panel_handle);
    
    
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

    /* Disabled as not currently working */
    //lv_obj_add_event_cb(chart, draw_event_cb, LV_EVENT_DRAW_TASK_ADDED, NULL);
    //lv_obj_add_flag(chart, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);

    lv_obj_set_style_line_width(chart, 0, LV_PART_ITEMS);       // Remove the lines
    lv_chart_set_type(chart, LV_CHART_TYPE_SCATTER);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_X, 0, 3500);    // Max range is 3.5m (3500mm)
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 3500);
    lv_chart_set_point_count(chart, 360);
    lv_chart_series_t * chart_data = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);     // add a data series
    
    uint32_t i;
    for(i = 0; i < 10; i++) {
        lv_chart_set_next_value2(chart, chart_data, (int32_t)lv_rand(0, 3500), (int32_t)lv_rand(0, 3500)); // create dummy data
    };

    
    _lock_release(&lvgl_api_lock);

    

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

    //uint8_t *single_lidar_packet = (uint8_t *) malloc(LIDAR_PACKET_SIZE);      // buffer for the packet to process
    //uint8_t single_lidar_packet[LIDAR_PACKET_SIZE];

    // this method of a rolling buffer of 3 packets means we are processing 2 scans behind the latest scan (not good!)

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
                        COND_ESP_LOGI(TAG_INFO, "Found %i LiDAR packets", num_found_packets);

                         
                        // process data for graph display
                        //memcpy(&single_lidar_packet, &lidar_data[i], LIDAR_PACKET_SIZE);     // copy data from buffer into single packet buffer
                        double lidar_angle = 0;
                        lv_point_t lidar_points[360];
                        // now have full data packet, loop over each 42 byte packet
                        for(uint16_t x = 0; x < LIDAR_PACKET_SIZE; x = x + 42){
                            for (int j = i + 6; j < i + 40; j = j + 6) {
                                int32_t lidar_distance = (lidar_data[j+1] << 8) + lidar_data[j];


                                /* This doesn't fix the issue below with using cos */
                                double *tmp = (double *) malloc(sizeof(double));
                                
                                tmp[0] = (double)lidar_distance * cos(lidar_angle * (double)M_PI / 180.0);
                                lidar_points[(int16_t)lidar_angle].x = tmp[0];
                                free(tmp);
                                /* Something about the following breaks the ESP - Stack overflow */
                                //lidar_points[(int16_t)lidar_angle].x = lidar_distance * (int32_t)cos(lidar_angle * (double)M_PI / 180.0);
                                //lidar_points[lidar_angle].y = lidar_distance * sin(lidar_angle);

                                lidar_angle += 1;
                            
                            }
                        }
                        

                        // Lock the mutex due to the LVGL APIs are not thread-safe
                        _lock_acquire(&lvgl_api_lock);
                        lv_label_set_text(label, ""); // clear text using same "label" as previously created
                        char text[128];
                        snprintf(text, sizeof(text), "Found %i LiDAR packets", num_found_packets);
                        lv_label_set_text(label, text);
                        lv_chart_set_point_count(chart, 0);     // clear existing points
                        lv_chart_set_point_count(chart, 360);    // reset point coint 
                        // Add lidar data to chart
                        for(uint16_t z = 0; z < 360; z++) {
                           //lv_chart_set_next_value2(chart, chart_data, lidar_points[i].x, lidar_points[i].y); 
                           lv_chart_set_next_value2(chart, chart_data, (int32_t)lv_rand(0, 3500), (int32_t)lv_rand(0, 3500)); 
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

    restartESP(2);
    
}