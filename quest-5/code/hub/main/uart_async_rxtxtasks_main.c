/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/rmt.h"

#include <string.h>
#include <stdio.h>
#include <sys/param.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

static const char *RMT_TX_TAG = "RMT Tx";

#define HOST_IP_ADDR "192.168.1.113"
#define PORT 8080

static const char *TAG = "Client";

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define SAMPLE_CNT  (10)

#define BIT_0 21

#define LED_B 14
#define LED_G 32
#define LED_R 15
#define RMT_TX_GPIO 26 // A0
#define TXD_PIN 25 //A1
#define RXD_PIN 34 //A2
#define BUTTON 4 //A5

#define RMT_CLK_DIV      100

static const int RX_BUF_SIZE = 1024;
int pressed_flag = 0, counter = 0, count = 0;
char fobid[10];
char code[10];
char *HUB_ID = "hub1";

void init(void) {
        const uart_config_t uart_config = {
                .baud_rate = 2400,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };
        uart_param_config(UART_NUM_1, &uart_config);
        uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
        uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        // We won't use a buffer for sending data.
        uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int sendData(const char* logName, const char* data)
{
        const int len = strlen(data);
        const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
        ESP_LOGI(logName, "Wrote %d bytes", txBytes);
        return txBytes;
}

static void tx_task(void *arg)
{
        static const char *TX_TASK_TAG = "TX_TASK";
        esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
        char* return_disp = "f";
        // ############## change payload
        while(1) {
                if (count == 2 || count == 3) {
                        return_disp = (count == 2) ? "f" : "s";
                        sendData(TX_TASK_TAG, return_disp);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                        sendData(TX_TASK_TAG, return_disp);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                        sendData(TX_TASK_TAG, return_disp);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                        if (count == 3)
                                vTaskDelay(1000 / portTICK_PERIOD_MS);
                        count = 0;
                }
                vTaskDelay(20 / portTICK_PERIOD_MS);
        }
}

static void rx_task(void *arg)
{
        static const char *RX_TASK_TAG = "RX_TASK";
        esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
        uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
        char message[80];
        while (1) {
                if (count == 0) {
                        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 500 / portTICK_RATE_MS);
                        if (rxBytes > 0) {
                                data[rxBytes] = 0;
                                ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
                                // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
                                sprintf(message, "%s ", data);
                                if ((int)message[0] >= 65 && (int)message[0] <= 122) {
                                        char *ptr = strtok(message, " ");
                                        strcat(fobid, ptr);
                                        ptr = strtok(NULL, " ");
                                        strcat(code, ptr);
                                        if (strlen(fobid) >= 2 && strlen(code) >= 2)
                                                count = 1;
                                        printf("VALID\n");
                                }
                                vTaskDelay(20);
                        }
                }
                vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        free(data);
}

static void udp_client_task(void *pvParameters) {
        // Initialize variables
        char rx_buffer[128];
        char addr_str[128];
        int addr_family;
        int ip_protocol;

        while (1) {
                // Socket address settings
                struct sockaddr_in dest_addr;
                dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_port = htons(PORT);
                addr_family = AF_INET;
                ip_protocol = IPPROTO_IP;
                inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

                // Tries to create socket, if fail, retry in while loop
                int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
                if (sock < 0) {
                        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                        break;
                }
                ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
                char payload[30];
                while (1) {
                        if (count == 1) {
                                strcat(payload, fobid);
                                strcat(payload, " ");
                                strcat(payload, HUB_ID);
                                strcat(payload, " ");
                                strcat(payload, code);
                                printf("Modified Payload: %s\n", payload);
                                fobid[0] = '\0';
                                code[0] = '\0';
                                // Uses UART to get input from user, uses this control UDP client
                                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                                payload[0] = '\0';

                                if (err < 0) {
                                        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                                        break;
                                }
                                ESP_LOGI(TAG, "Message sent");
                                vTaskDelay(500 / portTICK_PERIOD_MS);
                                count = 2;
                        }
                        vTaskDelay(750 / portTICK_PERIOD_MS);
                }

                if (sock != -1) {
                        ESP_LOGE(TAG, "Shutting down socket and restarting...");
                        shutdown(sock, 0);
                        close(sock);
                }
        }
        vTaskDelete(NULL);
}

//Convert uint8_t type of data to rmt format data.
static void IRAM_ATTR u8_to_rmt(const void* src, rmt_item32_t* dest, size_t src_size,
                                size_t wanted_num, size_t* translated_size, size_t* item_num)
{
        if(src == NULL || dest == NULL) {
                *translated_size = 0;
                *item_num = 0;
                return;
        }
        const rmt_item32_t bit0 = {{{ 32767, 1, 15000, 0 }}}; //Logical 0
        const rmt_item32_t bit1 = {{{ 32767, 1, 32767, 0 }}}; //Logical 1
        size_t size = 0;
        size_t num = 0;
        uint8_t *psrc = (uint8_t *)src;
        rmt_item32_t* pdest = dest;
        while (size < src_size && num < wanted_num) {
                for(int i = 0; i < 8; i++) {
                        if(*psrc & (0x1 << i)) {
                                pdest->val =  bit1.val;
                        } else {
                                pdest->val =  bit0.val;
                        }
                        num++;
                        pdest++;
                }
                size++;
                psrc++;
        }
        *translated_size = size;
        *item_num = num;
}

static void rmt_tx_int(void)
{
        rmt_config_t config;
        config.rmt_mode = RMT_MODE_TX;
        config.channel = RMT_TX_CHANNEL;
        config.gpio_num = RMT_TX_GPIO;
        config.mem_block_num = 1;
        config.tx_config.loop_en = 0;
        // enable the carrier to be able to hear the Morse sound
        // if the RMT_TX_GPIO is connected to a speaker
        config.tx_config.carrier_en = 1;
        config.tx_config.idle_output_en = 1;
        config.tx_config.idle_level = 1;
        config.tx_config.idle_output_en = true;
        config.tx_config.carrier_duty_percent = 50;
        // set audible career frequency of 611 Hz
        // actually 611 Hz is the minimum, that can be set
        // with current implementation of the RMT API
        config.tx_config.carrier_freq_hz = 38000;
        config.tx_config.carrier_level = 1;
        // set the maximum clock divider to be able to output
        // RMT pulses in range of about one hundred milliseconds
        config.clk_div = RMT_CLK_DIV;

        ESP_ERROR_CHECK(rmt_config(&config));
        ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
        ESP_ERROR_CHECK(rmt_translator_init(config.channel, u8_to_rmt));
}

static void control(void *arg) {

        /*
           int press_mem[2];
           int user = 1;
           gpio_pad_select_gpio(BIT_0);
           gpio_set_direction(BIT_0, GPIO_MODE_OUTPUT);
           gpio_pad_select_gpio(BIT_1);
           gpio_set_direction(BIT_1, GPIO_MODE_OUTPUT);
           while (1) {
                pressed_flag = (gpio_get_level(BUTTON)) ? 0 : 1;
                press_mem[0] = pressed_flag;
                if (pressed_flag && press_mem[1] == 0) {
                        printf("PRESSED\n");
                        count = 1;

                        user = (user == 3) ? 1 : user + 1;
                        switch (user) {
                        case 1: FOB_ID = "fob3";
                                gpio_set_level(BIT_0,1);
                                gpio_set_level(BIT_1,1);
                                break;
                        case 2: FOB_ID = "fob1";
                                gpio_set_level(BIT_0,1);
                                gpio_set_level(BIT_1,0);
                                break;
                        case 3: FOB_ID = "fob2";
                                gpio_set_level(BIT_0,0);
                                gpio_set_level(BIT_1,1);
                                break;
                        }
                        vTaskDelay(1000/portTICK_RATE_MS);
                }
                vTaskDelay(100/portTICK_RATE_MS);
                press_mem[1] = press_mem[0];
           }
         */
        int press_mem[2];
        int hubs = 1;
        gpio_pad_select_gpio(BIT_0);
        gpio_set_direction(BIT_0, GPIO_MODE_OUTPUT);
        while (1) {
                pressed_flag = (gpio_get_level(BUTTON)) ? 0 : 1;
                press_mem[0] = pressed_flag;
                if (pressed_flag && press_mem[1] == 0) {
                        printf("PRESSED\n");
                        count = 0;

                        hubs = (hubs == 2) ? 1 : hubs + 1;
                        switch (hubs) {
                        case 1: HUB_ID = "hub1";
                                gpio_set_level(BIT_0,0);
                                break;
                        case 2: HUB_ID = "hub2";
                                gpio_set_level(BIT_0,1);
                                break;
                        }
                        vTaskDelay(1000/portTICK_RATE_MS);
                }
                vTaskDelay(100/portTICK_RATE_MS);
                press_mem[1] = press_mem[0];
        }
}

void app_main(void)
{
        init();
        rmt_tx_int();

        ESP_ERROR_CHECK(nvs_flash_init());
        tcpip_adapter_init();
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(example_connect());

        xTaskCreate(udp_client_task, "udp_client_task", 4096, NULL, configMAX_PRIORITIES-1, NULL);
        xTaskCreate(control, "control", 1024*2, NULL, configMAX_PRIORITIES+1, NULL);
        xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
        xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES+1, NULL);
        // xTaskCreate(rmt_tx, "rmt_tx", 1024*2, NULL, configMAX_PRIORITIES, NULL);
        gpio_pad_select_gpio(LED_R);
        gpio_set_direction(LED_R, GPIO_MODE_OUTPUT);
        gpio_pad_select_gpio(LED_G);
        gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);
        gpio_pad_select_gpio(LED_B);
        gpio_set_direction(LED_B, GPIO_MODE_OUTPUT);

        gpio_pad_select_gpio(BUTTON);
        gpio_set_direction(BUTTON, GPIO_MODE_INPUT);

        gpio_set_level(LED_R,0);
        gpio_set_level(LED_G,0);
        gpio_set_level(LED_B,0);
        while(1) {
                switch (count) {
                case 0: gpio_set_level(LED_R,0);
                        gpio_set_level(LED_G,0);
                        gpio_set_level(LED_B,0);
                        break;
                case 1: gpio_set_level(LED_R,0);
                        gpio_set_level(LED_G,0);
                        gpio_set_level(LED_B,1);
                        break;
                case 2: gpio_set_level(LED_R,0);
                        gpio_set_level(LED_G,1);
                        gpio_set_level(LED_B,1);
                        break;
                case 3: gpio_set_level(LED_R,1);
                        gpio_set_level(LED_G,0);
                        gpio_set_level(LED_B,0);
                        break;
                }
                vTaskDelay(20);
        }
}
