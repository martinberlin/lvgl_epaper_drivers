/*
* Copyright © 2024 Fasani Corp.

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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include <esp_log.h>
#include "driver/gpio.h"
#include <driver/i2c.h>

QueueHandle_t touch_queue = NULL;

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include <lvgl.h>
#else
#include <lvgl/lvgl.h>
#endif
#include "tma445.h"

#define TAG "TMA445"

#ifdef CONFIG_LV_TOUCH_I2C_PORT_0
#define I2C_PORT I2C_NUM_0
#endif
#ifdef CONFIG_LV_TOUCH_I2C_PORT_1
#define I2C_PORT I2C_NUM_1
#endif

#define TS_INT  3
#define TS_RES  -1

#define TOUCH_ADDR 0x24
// Adjust to your desired level. Too large might record many seconds which is bad for UX
#define TOUCH_QUEUE_LENGTH 15

#define CY_REG_BASE         	0x00
#define GET_NUM_TOUCHES(x)     	((x) & 0x0F)
#define GET_TOUCH1_ID(x)       	(((x) & 0xF0) >> 4)
#define GET_TOUCH2_ID(x)       	((x) & 0x0F)
#define CY_XPOS             		0
#define CY_YPOS             		1
#define CY_MT_TCH1_IDX      		0
#define CY_MT_TCH2_IDX      		1
#define be16_to_cpu(x) (uint16_t)((((x) & 0xFF00) >> 8) | (((x) & 0x00FF) << 8))

#define CY_OPERATE_MODE    		0x00
#define CY_SOFT_RESET_MODE      0x01
#define CY_LOW_POWER         	0x04 // Non used for now
#define CY_HNDSHK_BIT           0x80

// I2C common protocol defines
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

static uint8_t sec_key[] = {0x00, 0xFF, 0xA5, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    
#define DELAY(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define GET_BOOTLOADERMODE(reg)		((reg & 0x10) >> 4)

/* TrueTouch Standard Product Gen3 (Txx3xx) interface definition */
struct cyttsp_xydata {
	uint8_t hst_mode;
	uint8_t tt_mode;
	uint8_t tt_stat;
	uint16_t x1 __attribute__ ((packed));
	uint16_t y1 __attribute__ ((packed));
	uint8_t z1;
	uint8_t touch12_id;
	uint16_t x2 __attribute__ ((packed));
	uint16_t y2 __attribute__ ((packed));
	uint8_t z2;
};

struct cyttsp_xydata xy_data;

struct cyttsp_bootloader_data {
	uint8_t bl_file;
	uint8_t bl_status;
	uint8_t bl_error;
	uint8_t blver_hi;
	uint8_t blver_lo;
	uint8_t bld_blver_hi;
	uint8_t bld_blver_lo;
	uint8_t ttspver_hi;
	uint8_t ttspver_lo;
	uint8_t appid_hi;
	uint8_t appid_lo;
	uint8_t appver_hi;
	uint8_t appver_lo;
	uint8_t cid_0;
	uint8_t cid_1;
	uint8_t cid_2;
};

static const char *LOGG = "LOG";

static void i2c_master_init() {
    i2c_config_t conf = { };
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = CONFIG_LV_TOUCH_I2C_SDA;
    conf.scl_io_num = CONFIG_LV_TOUCH_I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ), ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Read contents of a register
---------------------------------------------------------------------------*/
esp_err_t i2c_read_reg( uint8_t reg, uint8_t *pdata, uint8_t count ) {
	return( i2c_master_read_slave_reg( I2C_NUM_0, TOUCH_ADDR,  reg, pdata, count ) );
}

/* Write value to specified register
---------------------------------------------------------------------------*/
esp_err_t i2c_write_reg( uint8_t reg, uint8_t *pdata, uint8_t count ) {
	return( i2c_master_write_slave_reg( I2C_NUM_0, TOUCH_ADDR,  reg, pdata, count ) );
}

static QueueHandle_t gpio_evt_queue = NULL;

static int _cyttsp_hndshk_n_write(uint8_t write_back)
{
	int retval = -1;
	uint8_t hst_mode[1];
	uint8_t cmd[1];
	uint8_t tries = 0;
	while (retval < 0 && tries++ < 20){
		DELAY(5);
        retval = i2c_read_reg(0x00, &hst_mode, sizeof(hst_mode));
        if(retval < 0) {	
			printf("%s: bus read fail on handshake ret=%d, retries=%d\n",
				__func__, retval, tries);
			continue;
		}
        cmd[0] = hst_mode[0] & CY_HNDSHK_BIT ?
		write_back & ~CY_HNDSHK_BIT :
		write_back | CY_HNDSHK_BIT;
        retval = i2c_write_reg(CY_REG_BASE, &cmd, sizeof(cmd));
        if(retval < 0)
			printf("%s: bus write fail on handshake ret=%d, retries=%d\n",
				__func__, retval, tries);
    }
    return retval;
}

// 317 cyttsp.c
static int _cyttsp_hndshk()
{
	int retval = -1;
	uint8_t hst_mode[1];
	uint8_t cmd[1];
	uint8_t tries = 0;
	while (retval < 0 && tries++ < 20){
		DELAY(5);
        retval = i2c_read_reg(0x00, &hst_mode, sizeof(hst_mode));
        if(retval < 0) {	
			printf("%s: bus read fail on handshake ret=%d, retries=%d\n",
				__func__, retval, tries);
			continue;
		}
        cmd[0] = hst_mode[0] & CY_HNDSHK_BIT ?
		hst_mode[0] & ~CY_HNDSHK_BIT :
		hst_mode[0] | CY_HNDSHK_BIT;
        retval = i2c_write_reg(CY_REG_BASE, &cmd, sizeof(cmd));
        if(retval < 0)
			printf("%s: bus write fail on handshake ret=%d, retries=%d\n",
				__func__, retval, tries);
    }
    return retval;
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void touch_INT(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            /* get event data from CYTTSP device */
            int retval;
            retval = i2c_read_reg(CY_REG_BASE, &xy_data, sizeof(xy_data));
            if (retval < 0) {
		        printf("%s: Error, fail to read device on host interface bus\n", __func__);
            }
            /* provide flow control handshake */
            _cyttsp_hndshk();
            int touch = GET_NUM_TOUCHES(xy_data.tt_stat);
            if (touch) {
                xy_data.x1 = be16_to_cpu(xy_data.x1);
                xy_data.y1 = be16_to_cpu(xy_data.y1);
            if (xQueueSend( touch_queue, &xy_data, ( TickType_t ) 0 ) == pdTRUE) {
                // The message was sent successfully
            } else {
                ESP_LOGW("touch INT", "Could not add xy_data in queue");
            }
            }
        }
    }
}

void resetTouch() {
    gpio_set_level(TS_RES, 1);
    DELAY(20);
    gpio_set_level(TS_RES, 0);
    DELAY(40);
    gpio_set_level(TS_RES, 1);
    DELAY(20);
}

void touchStuff() {
    // soft reset
    esp_err_t err;
    uint8_t softres[] = {0x01};
    err = i2c_write_reg(CY_REG_BASE, &softres, 1);
    if (err != ESP_OK) {
        printf("i2c_write_reg CY_SOFT_RESET_MODE FAILED \n");
    }
    DELAY(50);
    // write sec_key
    err = i2c_write_reg(CY_REG_BASE, &sec_key, sizeof(sec_key));
    if (err != ESP_OK) {
        printf("i2c_write_reg sec_key FAILED \n");
    }
    printf("i2c_write_reg sec_key OK \n");
    DELAY(88);
    
    // Before this there is a part that reads sysinfo and sets some registers
    // Maybe that is the key to start the engines faster?
    uint8_t tries = 0;
    struct cyttsp_bootloader_data  bl_data = {};
    do {
		DELAY(20);
        i2c_read_reg(CY_REG_BASE, &bl_data, sizeof(bl_data));
        uint8_t *bl_data_p = (uint8_t *)&(bl_data);
        /* for (int i = 0; i < sizeof(struct cyttsp_bootloader_data); i++) {
			printf("bl_data[%d]=0x%x\n", i, bl_data_p[i]);
        }
        printf("bl_data status=0x%x\n", bl_data.bl_status); */
	} while (GET_BOOTLOADERMODE(bl_data.bl_status) && tries++ < 10);

    printf("bl_data status=0x%x\n", GET_BOOTLOADERMODE(bl_data.bl_status));

    // Set OP mode
    int retval;
    uint8_t cmd = CY_OPERATE_MODE;
    struct cyttsp_xydata xy_data;
    printf("%s set operational mode\n",__func__);
	memset(&(xy_data), 0, sizeof(xy_data));
    /* wait for TTSP Device to complete switch to Operational mode */
	DELAY(20);
    retval = i2c_read_reg(CY_REG_BASE, &xy_data, sizeof(xy_data));
    printf("%s: hstmode:0x%x tt_mode:0x%x tt_stat:0x%x ", __func__, xy_data.hst_mode, xy_data.tt_mode, xy_data.tt_stat);
}

/**
 * @brief  Initialize for TMA445 communication via I2C
 * @retval None
 * IMPORTANT: Adjust here to your panel resolution this is by default
 *            set to the Kindle 6" paperwhite x_max/y_max
 */
void tma445_init(uint8_t dev_addr)
{
    // It seems there is no need to RESET
    #if TS_RES != -1
    gpio_set_direction(TS_RES, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(TS_RES, GPIO_PULLUP_ONLY);
    resetTouch();
    #endif
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = TS_INT;
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(TOUCH_QUEUE_LENGTH, sizeof(uint8_t));
    touch_queue = xQueueCreate(TOUCH_QUEUE_LENGTH, sizeof(xy_data));

    //i2c_master_init(); // I2C is already started by LVGL

    // Setup interrupt for this IO that goes low on the interrupt
    gpio_set_intr_type(TS_INT, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TS_INT, gpio_isr_handler, (void *)TS_INT);
    // INT detection task
    xTaskCreatePinnedToCore(touch_INT, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
     
    // Start I2C scanner task
    //xTaskCreatePinnedToCore(i2cscanner, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    touchStuff();
}

/**
 * @brief  Get the touch screen X and Y positions values. Ignores multi touch
 * @param  drv:
 * @param  data: Store data here
 * @retval Always false
 */
bool tma445_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    /* Read from the touch Queue */
    if( xQueueReceive(touch_queue, &xy_data, ( TickType_t ) 0 ) ) {
        data->state = LV_INDEV_STATE_PR;
        data->point.y = 768 - xy_data.x1;
        data->point.x = xy_data.y1;
        printf("qR X:%d Y:%d\n", data->point.x, data->point.y);
    } else {
        data->state = LV_INDEV_STATE_REL;
        data->point.x = -1;
        data->point.y = -1;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    return false;
}
    

