/*
* Copyright © 2024 Fasani Corp.
*
* LVGL for TrueTouch 21100
* Permission is hereby granted, free of charge, to any person obtaining a copy of this 
* software and associated documentation files (the “Software”), to deal in the Software 
* without restriction, including without limitation the rights to use, copy, modify, merge, 
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
* to whom the Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
* SOFTWARE.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_touch_tt21100.h"
#include <esp_log.h>
#include "driver/i2c.h"

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include <lvgl.h>
#else
#include <lvgl/lvgl.h>
#endif
#include "tt21100.h"

#define TAG "TT21100"

#ifdef CONFIG_LV_TOUCH_I2C_PORT_0
    #define I2C_PORT I2C_NUM_0
#endif
#ifdef CONFIG_LV_TOUCH_I2C_PORT_1
    #define I2C_PORT I2C_NUM_1
#endif

esp_lcd_touch_handle_t tp;

/**
  * @brief  Initialize for TT communication via I2C
  * @retval None
  */
void tt21100_init(uint8_t dev_addr) {
    //printf("init TT I2C SDA:%d SCL:%d\n",CONFIG_LV_TOUCH_I2C_SDA, CONFIG_LV_TOUCH_I2C_SCL);
    // Setup up touch panel
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_TT21100_CONFIG();

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = 1448,
        .y_max = 1072,
        .rst_gpio_num = 9,
        .int_gpio_num = 3,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 1,
        },
    };
    
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_PORT, &tp_io_config, &tp_io_handle);
    
    esp_lcd_touch_new_i2c_tt21100(tp_io_handle, &tp_cfg, &tp);
}

/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
bool tt21100_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    esp_lcd_touch_read_data(tp);
    uint16_t touch_x[1];
        uint16_t touch_y[1];
        uint16_t touch_strength[1];
        uint8_t touch_cnt = 0;
        bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, touch_strength, &touch_cnt, 1);
    if (touchpad_pressed) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touch_x[0];
        data->point.y = touch_y[0];
        printf("x:%d y:%d\n", (int)touch_x[0], (int)touch_y[0]);
        } else {
        data->state = LV_INDEV_STATE_REL;
        data->point.x = -1;
        data->point.y = -1;
    }
    return false;
}
