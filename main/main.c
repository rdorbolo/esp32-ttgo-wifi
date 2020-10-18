/*

   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <driver/adc.h>
#include "esp_timer.h"
#include <string.h>

#include "../../esp32-ttgo-display/main/ttgo.h"

#define UART_RX 27
#define UART_TX 26

extern uint8_t bgRed, bgGreen, bgBlue;
unsigned test = 0;

struct AirData
{
    unsigned pm1;
    unsigned pm2_5;
    unsigned pm10;
    unsigned count_3;
    unsigned count_5;
    unsigned count1;
    unsigned count2_5;
    unsigned count5;
    unsigned count10;
    int64_t sampleTime;
} airData;

int displayMode = 0;

//The driver samples the SDA (input data) at rising edge of SCL,
//but shifts SDA (output data) at the falling edge of SCL

//After the read status command has been sent, the SDA line must be set to tri-state no later than at the
//falling edge of SCL of the last bit.

unsigned calcAqi(unsigned pm25)
{

    int interval_y0;
    int interval_y1;
    int interval_x0;
    int interval_x1;

    if (pm25 <= 12)
    {
        interval_y0 = 0;
        interval_y1 = 50;
        interval_x0 = 0;
        interval_x1 = 12;
    }
    else if (pm25 <= 35)
    {
        interval_y0 = 51;
        interval_y1 = 100;
        interval_x0 = 13;
        interval_x1 = 35;
    }
    else if (pm25 <= 55)
    {
        interval_y0 = 101;
        interval_y1 = 150;
        interval_x0 = 35;
        interval_x1 = 55;
    }
    else if (pm25 <= 150)
    {
        interval_y0 = 151;
        interval_y1 = 200;
        interval_x0 = 55;
        interval_x1 = 150;
    }
    else if (pm25 <= 250)
    {
        interval_y0 = 201;
        interval_y1 = 300;
        interval_x0 = 150;
        interval_x1 = 250;
    }
    else if (pm25 <= 350)
    {
        interval_y0 = 301;
        interval_y1 = 400;
        interval_x0 = 250;
        interval_x1 = 350;
    }
    else if (pm25 <= 500)
    {
        interval_y0 = 401;
        interval_y1 = 500;
        interval_x0 = 350;
        interval_x1 = 500;
    }
    else
    {
        return pm25;
    }
    int retval = interval_y0 + ((pm25 - interval_x0) * (interval_y1 - interval_y0) / (interval_x1 - interval_x0));

    return retval + test;
}

void test1()
{

    //TTGO Logo is top
    char s[30];
    int count = 0;
    int oldColorNumber = -1;
    int64_t oldSampleTime = 0;
    int animationCnt = 0;

    while (1)
    {

        if (displayMode == 0)
        {
            bgRed = 0x00;
            bgGreen = 0x00;
            bgBlue = 0x00;
            clearScreen(bgRed, bgGreen, bgBlue);
        }

        while (1)
        {

            if (displayMode == 0)
            {

                unsigned aqi = calcAqi(airData.pm2_5);
                unsigned colorNumber;

                if (aqi < 51)
                {
                    colorNumber = 0; //green
                    bgRed = 0x00;
                    bgGreen = 0xcf;
                    bgBlue = 0x00;
                }
                else if (aqi < 101)
                {
                    colorNumber = 1; //yellow
                    bgRed = 0xdf;
                    bgGreen = 0xdf;
                    bgBlue = 0x00;
                }
                else if (aqi < 151)
                {
                    colorNumber = 2; // Orange;
                    bgRed = 0xdf;
                    bgGreen = 0xb0;
                    bgBlue = 0x00;
                }
                else if (aqi < 201)
                {
                    colorNumber = 3; // Red;
                    bgRed = 0xef;
                    bgGreen = 0x00;
                    bgBlue = 0x00;
                }
                else if (aqi < 301)
                {
                    colorNumber = 4; // purple;
                    bgRed = 0xd0;
                    bgGreen = 0x00;
                    bgBlue = 0xd0;
                }
                else
                {
                    colorNumber = 5; // Maroon;
                    bgRed = 0xa0;
                    bgGreen = 0x00;
                    bgBlue = 0x30;
                }

                gpio_set_intr_type(35, GPIO_PIN_INTR_POSEDGE);

                if (oldColorNumber != colorNumber)
                    clearScreen(bgRed, bgGreen, bgBlue);
                oldColorNumber = colorNumber;

                if (oldSampleTime != airData.sampleTime)
                {

                    int x = 40;
                    // int y = 10;
                    const int yMid = 135 / 2;
                    count++;

                    snprintf(s, 30, "AQI");
                    displayStr(s, x, yMid - 16, 0xf0, 0xf0, 0xf0, 32);
                    snprintf(s, 30, "%d   ", aqi);
                    displayStr(s, x + 60, yMid - 28, 0xf0, 0xf0, 0xf0, 64);

                    //snprintf(s, 30, "PM2.5");
                    //displayStr(s, x+90, y,0xf0, 0xf0, 0xf0, 32);

                    //snprintf(s, 30, "%d  ", airData.pm2_5 );
                    //displayStr(s, x+5+90, y,0xf0, 0xf0, 0xf0, 64);

                    //snprintf(s, 30, "pm2.5:  %d   ", airData.pm2_5);
                    //displayStr(s, x, y,0xf0, 0xf0, 0xf0);
                    //y = y + 32;
                    //snprintf(s, 30, "0.3 qty:  %d   ", airData.count_3);
                    //displayStr(s, x, y,0xf0, 0xf0, 0xf0);
                    //y = y + 32;

                    const int v = 3818 + (adc1_get_raw(ADC1_CHANNEL_6) * 1000 - 2080 * 1000) / 625;
                    snprintf(s, 30, "%d.%d V  ", v / 1000, ((v % 1000) + 50) / 100);
                    //snprintf(s, 30, "%lld   ", airData.sampleTime / 1000000);
                    displayStr(s, 10, 135 - 32, 0xf0, 0xf0, 0xf0, 32);

                    //fillBox(210, 15, 14, 14, 0xff, 0xff, 0xff);

                    animationCnt = 0;
                    //printf("1\n");
                }

                if (animationCnt < 10)
                {

                    const u_int8_t color = 0xff - animationCnt*0x0f;
                    fillBox(210, 15, 14, 14, color, color, color);
                    animationCnt++;
                }

                oldSampleTime = airData.sampleTime;
            }
            else
            {

                uint8_t random;
                random = (uint8_t)esp_random();
                bgRed = random;
                bgGreen = random;
                bgBlue = random;
                clearScreen(random, random, random >> 2);
                vTaskDelay(50 / portTICK_PERIOD_MS);

                //wrCmmd(ST7789_MADCTL);
                //wrData(0b01101000);

                int x = 20;
                int y = 20;

                snprintf(s, 30, "This ");
                x = displayStr(s, x, y, 0xff, 0xff, 0xff, 32);
                y = y + 32;
                snprintf(s, 30, "is a ");
                x = displayStr(s, x, y, 0xff, 0xff, 0xff, 32);
                y = y + 32;
                snprintf(s, 30, "Test ");
                x = displayStr(s, x, y, 0xff, 0xff, 0xff, 32);
                y = y + 32;
            }

            //vTaskDelay(1000 / portTICK_PERIOD_MS);
            //count++;

            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}

void gpio_int(void *arg)
{
    gpio_set_intr_type(35, GPIO_PIN_INTR_DISABLE);
    test = (test + 50) % 500;
}

static const int RX_BUF_SIZE = 1024;

void init(void)
{

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); //channel 6 is gpio 34

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_set_direction(35, GPIO_MODE_INPUT);
    gpio_set_intr_type(35, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(35, gpio_int, NULL);
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, 32, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            if ((data[0] == 'B') &&
                (data[1] == 'M') &&
                (rxBytes == 32) &&
                (data[2] == (uint8_t)0) &&
                (data[3] == (uint8_t)28))
            {

                airData.pm1 = (((unsigned)data[4]) << 8) + (unsigned)data[5];
                airData.pm2_5 = (((unsigned)data[6]) << 8) + (unsigned)data[7];
                airData.pm10 = (((unsigned)data[8]) << 8) + (unsigned)data[9];
                airData.count_3 = (((unsigned)data[16]) << 8) + (unsigned)data[17];

                //airData.count_3 =
                airData.sampleTime = esp_timer_get_time();
            }

            //printf();
            // ESP_LOGI(RX_TASK_TAG, "pm2.5 = %d .3um Count = %d ts=%lld", airData.pm2_5,
            //          airData.count_3, airData.sampleTime);
        }

        //else printf("no data\n");
    }
    free(data);
}

void app_main(void)
{

    initTTGO();
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);

    test1();
}
