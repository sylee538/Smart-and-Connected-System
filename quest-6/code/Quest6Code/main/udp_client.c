#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

#include "string.h"
#include <stdlib.h>
#include <sys/param.h>
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include "lwip/err.h"

#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <math.h>

#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"

#include "driver/pcnt.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

#include "soc/uart_struct.h"
#include "driver/timer.h"
#include "soc/gpio_sig_map.h"
#include "esp_types.h"
//Eddies Fob
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/rmt.h"


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"

// #define STEERERROR 18

// ADXL343
#define SLAVE_ADDR_LID                   0x62 // 0x53
#define ACQ_COMMAND                 (0x00)    /**< Assumes ALT address pin low */
#define ACQ_STATUS                   (0x01)
#define ACQ_LOW                 (0x10)
#define ACQ_HIGH                 (0x0f)


#define MAX_PRIORITY 6

//Turning
//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

//Timer
#define T_INTERVAL (.500)
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TEST_WITH_RELOAD      1

//Display
// --------------------------------------
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   256          //Multisampling

#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd/

#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

#define NO_OF_SAMPLES   256          //Multisampling

static const char *RMT_TX_TAG = "RMT Tx";

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define SAMPLE_CNT  (10)

#define LID_TX 27
#define LID_RX 26
#define IR_TX 32
#define IR_RX 34




// #define LED_B 14
// #define LED_G 32
// #define LED_R 15
// #define RMT_TX_GPIO   25
// #define TXD_PIN 26 //A1
// #define RXD_PIN 34 //A2
// #define BUTTON 4 //A5

#define RMT_CLK_DIV      100

#define SET_SPEED 1.0

#define HOST_IP_ADDR "192.168.1.118"
#define PORT 8080
static const char *TAG = "example";

static const int RX_BUF_SIZE = 1024;
bool automatic = true;
bool drive = false;
float micro_dist = 0;
float ultra_dist = 0;

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC1_CHANNEL_3;     //A3, 39
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


bool checkCheckSum(uint8_t *p, int len) {
        char temp = (char) 0;
        bool isValid;
        for (int i = 0; i < len-1; i++) {
                temp = temp^p[i];
        }

        return isValid = (temp == p[len-1]) ? true : false;
}

static void rx_task_ir(void *arg) {
        const uart_config_t uart_config = {
                .baud_rate = 1200,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };
        uart_param_config(UART_NUM_1, &uart_config);
        uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
        uart_set_pin(UART_NUM_1, IR_TX, IR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);


        static const char *RX_TASK_TAG = "RX_TASK";
        esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
        uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);

        while (1) {
                uint8_t payload[4];
                const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 20 / portTICK_RATE_MS);

                // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
                if (rxBytes > 0) {
                        data[rxBytes] = 0;
                        ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
                        for(int i = 0; i < rxBytes; i++) {
                                if (data[i] == 0x1B) {
                                        payload[0] = data[i];
                                        payload[1] = data[i+1];
                                        payload[2] = data[i+2];
                                        payload[3] = data[i+3];
                                        printf("%x %x %x %x\n",payload[0], payload[1], payload[2], payload[3]);
                                        if (checkCheckSum(payload,4)) {
                                                printf("CHECKSUM VALID\n");
                                                // printf("%c %d\n", payload[1], payload[2]);
                                                if (payload[1] == 'G') {
                                                        printf("%c %d\n", payload[1], payload[2]);
                                                        drive = true;
                                                }
                                                else if (payload[1] == 'R') {
                                                        printf("%c %d\n", payload[1], payload[2]);
                                                        drive = false;
                                                        if (payload[2] == 3)
                                                                automatic = false;
                                                }
                                        }
                                        break;
                                }
                        }
                }
                vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        free(data);
}

int display_number (int num) {
        int result = 0;

        switch(num) {
        case 0:
                result = 0b0000110000111111; // 0
                break;
        case 1:
                result = 0b0000000000000110; // 1
                break;
        case 2:
                result = 0b0000000011011011; // 2
                break;
        case 3:
                result = 0b0000000010001111; // 3
                break;
        case 4:
                result = 0b0000000011100110; // 4
                break;
        case 5:
                result = 0b0010000001101001; // 5
                break;
        case 6:
                result = 0b0000000011111101; // 6
                break;
        case 7:
                result = 0b0000000000000111; // 7
                break;
        case 8:
                result = 0b0000000011111111; // 8
                break;
        case 9:
                result = 0b0000000011101111; // 9
                break;

        }

        return result;
}

static void i2c_master_init(){
        // Debug
        printf("\n>> i2c Config\n");
        int err;

        // Port configuration
        int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

        /// Define I2C configurations
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;                          // Master mode
        conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;          // Default SDA pin
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;              // Internal pullup
        conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;          // Default SCL pin
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;              // Internal pullup
        conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;   // CLK frequency
        err = i2c_param_config(i2c_master_port, &conf);       // Configure
        if (err == ESP_OK) {printf("- parameters: ok\n");}

        // Install I2C driver
        err = i2c_driver_install(i2c_master_port, conf.mode,
                                 I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                                 I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
        // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
        if (err == ESP_OK) {printf("- initialized: yes\n\n");}

        // Dat in MSB mode
        i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

static void i2c_LIDAR_master_init(){
        // Debug
        printf("\n>> i2c Config\n");
        int err;

        // Port configuration
        int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

        /// Define I2C configurations
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;                        // Master mode
        conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;        // Default SDA pin
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
        conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;        // Default SCL pin
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
        conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
        err = i2c_param_config(i2c_master_port, &conf);     // Configure
        if (err == ESP_OK) {printf("- parameters: ok\n");}

        // Install I2C driver
        err = i2c_driver_install(i2c_master_port, conf.mode,
                                 I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                                 I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
        if (err == ESP_OK) {printf("- initialized: yes\n");}

        // Data in MSB mode
        i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

void print_err(int ret){
        if (ret == ESP_OK)
                printf("Success");
        else if(ret ==ESP_FAIL  )
                printf("fail");
        else if (ret == ESP_ERR_INVALID_ARG )
                printf("param");
        else if (ret == ESP_ERR_INVALID_STATE )
                printf("inv state");
        else if (ret == ESP_ERR_TIMEOUT )
                printf("timeout");

}

int alpha_oscillator() {
        int ret;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        vTaskDelay(200 / portTICK_RATE_MS);
        return ret;
}

// Set blink rate to off
int no_blink() {
        int ret;
        i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
        i2c_master_start(cmd2);
        i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
        i2c_master_stop(cmd2);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd2);
        vTaskDelay(200 / portTICK_RATE_MS);
        return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
        int ret;
        i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
        i2c_master_start(cmd3);
        i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
        i2c_master_stop(cmd3);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd3);
        vTaskDelay(200 / portTICK_RATE_MS);
        return ret;
}

int testConnection(uint8_t devAddr, int32_t timeout) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return err;
}

static void udp_client_task(void *pvParameters) {
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
                char payload[10];
                while (1) {
                        // Uses UART to get input from user, uses this control UDP client
                        // int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                        // if (err < 0) {
                        //         ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        //         break;
                        // }
                        // ESP_LOGI(TAG, "Message sent");

                        struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
                        socklen_t socklen = sizeof(source_addr);
                        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

                        // Error occurred during receiving
                        if (len < 0) {
                                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                                break;
                        }
                        // Data received
                        else {
                                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                                ESP_LOGI(TAG, "%s", rx_buffer);
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

static void i2c_scanner() {
        int32_t scanTimeout = 1000;
        printf("\n>> I2C scanning ..."  "\n");
        uint8_t icount = 0;
        for (uint8_t i = 1; i < 127; i++) {
                // printf("0x%X%s",i,"\n");
                if (testConnection(i, scanTimeout) == ESP_OK) {
                        printf( "- Device found at address: 0x%X%s", i, "\n");
                        icount++;
                }
        }
        if (icount == 0)
                printf("- No I2C devices found!" "\n");
        printf("\n");
}

static void test_alpha_display() {
        // Debug
        int ret;
        printf(">> Test Alphanumeric Display: \n");

        // Set up routines
        // Turn on alpha oscillator
        ret = alpha_oscillator();
        if(ret == ESP_OK) {printf("- oscillator: ok \n");}
        // Set display blink off
        ret = no_blink();
        if(ret == ESP_OK) {printf("- blink: off \n");}
        ret = set_brightness_max(0xF);
        if(ret == ESP_OK) {printf("- brightness: max \n");}

        // Write to characters to buffer
        uint16_t displaybuffer[8];
        displaybuffer[0] = 0b0101001000000001; // T.
        displaybuffer[1] = 0b0101001000001111; // D.
        displaybuffer[2] = 0b0100000000111001; // C.
        displaybuffer[3] = 0b0100000000111000; // L.

        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i=0; i<8; i++) {
                i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
                i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd4);

        if(ret == ESP_OK) {
                printf("Startup successful. Loading default display ...\n\n");
        }
}

static void check_efuse(void) {
        //Check TP is burned into eFuse
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
                printf("eFuse Two Point: Supported\n");
        } else {
                printf("eFuse Two Point: NOT supported\n");
        }

        //Check Vref is burned into eFuse
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
                printf("eFuse Vref: Supported\n");
        } else {
                printf("eFuse Vref: NOT supported\n");
        }
}

static void mcpwm_example_gpio_initialize(void) {
        printf("initializing mcpwm servo control gpio......\n");
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18); //Set GPIO 18 as PWM0A, to which servo is connected

}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
        uint32_t cal_pulsewidth = 0;
        cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
        return cal_pulsewidth;
}

void esc_start(void *arg) {
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);
        //int count;
        //2. initial mcpwm configuration
        printf("Configuring Initial Parameters of mcpwm......\n");
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        pwm_config.cmpr_a = 0; //duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0; //duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings

        vTaskDelay(3000 / portTICK_PERIOD_MS); // Give yourself time to turn on crawler
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100);  // HIGH signal in microseconds
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // LOW signal in microseconds
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        while (1) {
                if (drive) {
                        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200);

                        vTaskDelay(1000/portTICK_PERIOD_MS);
                }
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);

                vTaskDelay(20/portTICK_PERIOD_MS);
        }
}

void servo_start(void *arg){

        uint32_t angle = 0;

        int count = 0;
        while (1) {
                if (count <= SERVO_MAX_DEGREE && count >= 1) {
                        angle = servo_per_degree_init(count);
                        printf("Angle of rotation: %d\n", count);
                }
                else {
                        if (count > SERVO_MAX_DEGREE)
                                angle = (count < 100) ? servo_per_degree_init(SERVO_MAX_DEGREE) : servo_per_degree_init(45);
                        else
                                angle = (count > -100) ? servo_per_degree_init(0) : servo_per_degree_init(45);

                }
                // printf("ANGLE: %d\n", angle);
                mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
                vTaskDelay(100/portTICK_PERIOD_MS);

        }
}

static float bin2cm(uint32_t adc_reading) {
        float dist = (float)adc_reading * 5;
        return dist / 10;
}

static float volt2cm(uint32_t voltage) {
        float volts = (float)voltage/1000;
        // Datasheet shows 2 slopes
        if (volts < 0.45)
                return 200;
        else if (volts > 0.45 || volts <= 2) { // Volt change constantly, slope roughly y = 44.1176x + 0.3676
                return pow((volts-0.3676)/44.1176,-1);
        }
        else if (volts > 2 || volts <= 2.5) { // Volts greater than 2 has steeper slope means distance changes little at this point
                // Slope roughly y = 30.8824x + 0.9808
                return pow((volts-0.9808)/30.8824,-1);
        }
        else if (volts > 2.5 || volts <= 2.75) {
                return pow((volts-2.5)/14.706,-1);
        }
        else if (volts > 2.75)
                return 30;
        else
                return 0;

}
// ==============================================================// ==============================================================
//Sensors
// ==============================================================// ==============================================================

static void sensors(){
        check_efuse();

        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);

        //Characterize ADC
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        // print_char_val_type(val_type);

        while (1) {

                uint32_t adc_reading = 0;
                //Multisampling
                for (int i = 0; i < NO_OF_SAMPLES; i++) {
                        adc_reading += adc1_get_raw((adc1_channel_t)channel);
                }
                adc_reading /= NO_OF_SAMPLES;
                //Convert adc_reading to voltage in mV
                uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
                ultra_dist = bin2cm(adc_reading);

                if(automatic)
                        drive = (ultra_dist < 85) ? false : true;


                printf("Ultra distance: %f cm\n",ultra_dist);
                vTaskDelay(500/portTICK_RATE_MS);
        }
}

int writeRegister(uint8_t reg, uint8_t data) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR_LID << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return ret;
}

// Read register
uint8_t readRegister(uint8_t reg) {
        uint8_t data;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR_LID << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR_LID << 1 ) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS); //problem this doesnt read regist 0x01
        i2c_master_stop(cmd);
        int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return data;
}
//read 16 bits
int16_t read16() {
        uint8_t highbyte = 0, lowbyte = 0;
        lowbyte = readRegister(ACQ_LOW);
        highbyte = readRegister(ACQ_HIGH);
        //printf("FRONT LIDAR Bytes: %x %x\n", highbyte, lowbyte);
        return ((uint16_t)highbyte)<< 8 | lowbyte;
}

// Task to continuously poll acceleration and calculate roll and pitch
static void test_lidar() {
        printf("\n>> Polling ADAXL343\n");
        uint8_t lidar_active = 1;
        while(1) {
                uint16_t distance = read16(0x8f);
                printf("FRONT LIDAR Distance: %d\n", distance);
                vTaskDelay(500 / portTICK_RATE_MS);
        }
}

static void front_lid() {
        int error_ret;
        const uart_config_t uart_config = {
                .baud_rate = 115200,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };

        uart_param_config(UART_NUM_0, &uart_config);
        uart_set_pin(UART_NUM_0, LID_TX, LID_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        error_ret = uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
        print_err(error_ret);


        char *RX_TASK_TAG = "RX_TASK";
        esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
        uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
        uint16_t test = 0x59;

        while (1) {
                uint16_t lowbyte, highbyte;

                const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 750 / portTICK_RATE_MS);
                if (rxBytes > 0) {
                        data[rxBytes] = 0;
                        ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);

                        for(int i = 0; i < RX_BUF_SIZE; i++) {
                                //printf("%x\n", data[i]);
                                if((data[i] == test) && (data[i+1] == test)) {
                                        lowbyte = (uint16_t) data[i+2];
                                        highbyte = (uint16_t) data[i+3];

                                        if ((highbyte <= (uint16_t)4)) {
                                                micro_dist = (((uint32_t)highbyte) << 8 | lowbyte) + 16;
                                                printf("Micro Lidar: %f\n ", micro_dist);
                                                break;
                                        }
                                }
                        }
                }
        }
        free(data);
}


//static void PID() {

// static const char *RX_TASK_TAG = "RX_TASK";
// esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
// uint8_t* fdata = (uint8_t*) malloc(RX_BUF_SIZE+1);
// uint8_t* bdata = (uint8_t*) malloc(RX_BUF_SIZE+1);
// uint16_t test = 0x59;
// while (1) {
//         uint16_t flowbyte, fhighbyte;
//         uint16_t blowbyte, bhighbyte;
//         const int frxBytes = uart_read_bytes(UART_NUM_0, fdata, RX_BUF_SIZE, 750 / portTICK_RATE_MS);
//         const int brxBytes = uart_read_bytes(UART_NUM_1, bdata, RX_BUF_SIZE, 750 / portTICK_RATE_MS);
//         if (frxBytes > 0) {
//                 fdata[frxBytes] = 0;
//                 ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", frxBytes, fdata);
//
//                 for(int i = 0; i < RX_BUF_SIZE; i++) {
//                         //printf("%x\n", data[i]);
//                         if((fdata[i] == test) && (fdata[i+1] == test)) {
//                                 flowbyte = (uint16_t) fdata[i+2];
//                                 fhighbyte = (uint16_t) fdata[i+3];
//
//                                 if ((fhighbyte <= (uint16_t)4)) {
//                                         left_dist[0] = (((uint32_t)fhighbyte) << 8 | flowbyte) + 16;
//                                         left_dist[0] += 10;
//                                         printf("Front Lidar: %d\n ", left_dist[0]);
//                                         break;
//                                 }
//                         }
//                 }
//         }
//         if (brxBytes > 0) {
//                 bdata[brxBytes] = 0;
//                 //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", brxBytes, bdata);
//
//                 for(int i = 0; i < RX_BUF_SIZE; i++) {
//                         //printf("%x\n", data[i]);
//                         if((bdata[i] == test) && (bdata[i+1] == test)) {
//                                 blowbyte = (uint16_t) bdata[i+2];
//                                 bhighbyte = (uint16_t) bdata[i+3];
//
//                                 if ((bhighbyte <= (uint16_t)4)) {
//                                         left_dist[1] = (((uint32_t)bhighbyte) << 8 | blowbyte) + 16;
//                                         printf("Back Lidar: %d\n ", left_dist[1]);
//                                         break;
//                                 }
//                         }
//                 }
//         }
// }//runs 200 tests per second;
// free(fdata);
// free(bdata);
//}
//
static void motor_stop() {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
}

static void motor_turn_left(int count) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(count));
}

static void motor_turn_right(int count) {
        //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1300);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, servo_per_degree_init(count));


}

static void motor_forward() {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1300);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, servo_per_degree_init(45));

}

static void motor_backward(){
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1550);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, servo_per_degree_init(45));
}


void app_main(void){
        //Initializes UART
        // ESP_ERROR_CHECK(nvs_flash_init());
        // tcpip_adapter_init();
        // ESP_ERROR_CHECK(esp_event_loop_create_default());
        // ESP_ERROR_CHECK(example_connect());
        //pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
        //timer_queue = xQueueCreate(10, sizeof(timer_event_t));
        //pcnt_example_init();
        i2c_master_init();
        i2c_scanner();

        test_alpha_display();

        i2c_LIDAR_master_init();
        printf("Reading\n");
        uint8_t data;
        uint16_t reading;

        xTaskCreate(rx_task, "rx_task", 2048, NULL, MAX_PRIORITY - 1, NULL);
        xTaskCreate(front_lid, "front_lid", 4096, NULL, MAX_PRIORITY-1, NULL);
        xTaskCreate(sensors, "sensors", 2048, NULL, MAX_PRIORITY-2, NULL);


        //miniL_init();
        xTaskCreate(rx_task_ir, "rx_task_ir", 2048*2, NULL, MAX_PRIORITY-2, NULL);
        //xTaskCreate(PID, "PID", 2048, NULL, MAX_PRIORITY-1, NULL);

        printf("Testing servo motor.......\n");

        xTaskCreate(servo_start, "servo_start", 2048, NULL, 5, NULL);
        xTaskCreate(esc_start, "esc_start", 2048, NULL, MAX_PRIORITY, NULL);

        // xTaskCreate(udp_client_task, "udp_client_task", 2048, NULL, MAX_PRIORITY, NULL);
        // xTaskCreate(PID, "PID", 2048, NULL, 3, NULL);
        //For Eddies Fob
        //init();
        //xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES - 1, NULL);


        //  uint32_t angle = 0;
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, 21);
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        pwm_config.cmpr_a = 0; //duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0; //duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        printf("Configuring Initial Parameters of steering......\n");
        mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings

        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, servo_per_degree_init(45));
        bool collision = false;
        while(1) {

                // 0 right 90 left //45 straight
                writeRegister(ACQ_COMMAND, 0x04);
                //printf("%x\n", readRegister(ACQ_COMMAND));
                while(1) {
                        data = readRegister(ACQ_STATUS);
                        if ((data & 1) != 1)
                                break;
                        vTaskDelay(20/portTICK_RATE_MS);
                }
                reading = read16();
                // if (reading < 60) {
                //         drive = false;
                // }


                // printf("LIDAR dist: %d \nMicro dist: %3.0f\n Ultra dist: %3.0f\n\n", reading, micro_dist, ultra_dist);
                vTaskDelay(500/portTICK_RATE_MS);


        }
}
