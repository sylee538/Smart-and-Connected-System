#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "driver/i2c.h"
#include "ADXL343.h"

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

#define MAX_PRIO 7

// Pin Setups
#define LEDPIN 13 // Onboard LED
#define WATER_LED 26 // A0
#define LED2 21
#define ALERT_LED 33

// SETUP For Vibration Switch

#define DEBOUNCETIME 10
#define GPIO_OUTPUT_IO_0    18 // MOSI
#define GPIO_OUTPUT_PIN_SEL  1ULL<<GPIO_OUTPUT_IO_0
#define GPIO_INPUT_IO_0     4 // A5
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_OUTPUT_IO_0
#define ESP_INTR_FLAG_DEFAULT 0

// SETUP For Thermistor
#define CONST_B 3435 // B constant
#define CONST_K 298.15 // Room temp in K
#define CONST_R 15000

// SETUP For ADC
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

// SETUP For PWMled
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TEST_CH_NUM       (1)
#define LEDC_TEST_DUTY         8192

// SETUP For Socket
#define HOST_IP_ADDR "192.168.1.106"
#define PORT 8080

// Master I2C
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

// ADXL343
#define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53

static const char *TAG = "Client";

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //A2, GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

uint32_t debounceTimeout = 0;
int numberOfButtonInterrupts = 0;
int second = 0;
int water_time = 3599;

int look_flag = 0; // If user is here
int water_flag = 1; // if time for water, base setting is 60 min
int control_flag = 1;

// Passed Values
float battery = 8;
float temp = 0;
int steps = 0;


// ---------------------------------------
// Setup Functions
// ---------------------------------------
static void IRAM_ATTR gpio_isr_handler(void* arg) {
        debounceTimeout = xTaskGetTickCount(); // Gets current time when interrupt is triggered
        numberOfButtonInterrupts++;            // Adds to interrupt count
}

static void check_efuse() {
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

static void i2c_master_init(){
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


static void print_char_val_type(esp_adc_cal_value_t val_type) {
        if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
                printf("Characterized using Two Point Value\n");
        } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
                printf("Characterized using eFuse Vref\n");
        } else {
                printf("Characterized using Default Vref\n");
        }
}

static float ohm2C(uint32_t voltage) {
        float device_v = (float)voltage/1000; // Change mV to V
        //float therm_res = (5 * CONST_R)/device_v - CONST_R;
        float therm_res = CONST_R/((5/device_v) - 1); // Forumla to find resistance of Thermistor
        float inv_T = (1 / CONST_K) + log(therm_res/10000)/CONST_B; // Use formula to change Resistance to C
        return pow(inv_T, -1) - 273.15;
}

// Get Device ID
int getDeviceID(uint8_t *data) {
        int ret;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return ret;
}


// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
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
        i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
        uint8_t highbyte, lowbyte;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &highbyte, ACK_VAL);
        i2c_master_read_byte(cmd, &lowbyte, ACK_CHECK_DIS);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return ((uint16_t)lowbyte)<< 8 | highbyte;
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

// Utility function to scan for i2c device
static void i2c_scanner() {
        int32_t scanTimeout = 1000;
        printf("\n>> I2C scanning ..."  "\n");
        uint8_t count = 0;
        for (uint8_t i = 1; i < 127; i++) {
                // printf("0x%X%s",i,"\n");
                if (testConnection(i, scanTimeout) == ESP_OK) {
                        printf( "- Device found at address: 0x%X%s", i, "\n");
                        count++;
                }
        }
        if (count == 0) {printf("- No I2C devices found!" "\n");}
}

void setRange(range_t range) {
        /* Red the data format register to preserve bits */
        uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

        /* Update the data rate */
        format &= ~0x0F;
        format |= range;

        /* Make sure that the FULL-RES bit is enabled for range scaling */
        format |= 0x08;

        /* Write the register back to the IC */
        writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
        /* Red the data format register to preserve bits */
        return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
        return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

void getAccel(float * xp, float *yp, float *zp) {
        *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
        *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
        *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
        // printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

// function to print roll and pitch
int calcRP(float x, float y, float z){
        int step = 0;
        float roll = atan2(y,z) * 57.3;
        float pitch = -atan2((-x),sqrt(y*y+z*z))*57.3;
        printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
        if(z < 5.0) {
                if( y > 0 ) {
                        if ( pitch > -30.0 ) {
                                step++;
                        }
                }else if ( y < 0) {
                        if ( pitch > -60.0 ) {
                                step++;
                        }
                }
        }
        return step;
}

// ---------------------------------------
static void control(int ins, int val) {
        switch (ins) {
        case 1: water_time = val;
                printf("Water Time Change to %d\n", val);
                break;
        case 2: control_flag = 1;
                break;

        }
}
// Task to continuously poll acceleration and calculate roll and pitch
static void task_accel() {
        printf("\n>> Polling ADAXL343\n");
        int x_off = readRegister(ADXL343_REG_OFSX);
        int y_off = readRegister(ADXL343_REG_OFSY);
        int z_off = readRegister(ADXL343_REG_OFSZ);

        printf("xoff:%d, yoff:%d, zoff:%d\n", x_off, y_off, z_off);

        printf("\n>> Polling ADAXL343\n");
        while (1) {
                float xVal, yVal, zVal;
                getAccel(&xVal, &yVal, &zVal);
                xVal+=(int)x_off;
                yVal+=(int)y_off;
                //zVal+=(int)z_off;
                steps += calcRP(xVal, yVal, zVal);
                printf("Steps taken : %d\n", steps);
                vTaskDelay(500 / portTICK_RATE_MS);
        }
}

static void task_vib() {
        gpio_config_t io_conf;
        //disable interrupt
        io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
        //set as output mode
        io_conf.mode = GPIO_MODE_INPUT;
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 1;
        //configure GPIO with the given settings
        gpio_config(&io_conf);

        gpio_pad_select_gpio(LEDPIN);
        gpio_set_direction(LEDPIN, GPIO_MODE_INPUT_OUTPUT);
        gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_POSEDGE);

        // initiates ISR services
        gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
        gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

        // Software Debounce variables initialize
        uint32_t saveDebounceTimeout;     // Last time switch triggered
        int saveLastState;                // Last state of LED
        int save;                         // # of times triggered
        int flag = 0;                     // Flag for next state

        while (1) {
                save  = numberOfButtonInterrupts;                          // Checks for interrupts
                saveDebounceTimeout = xTaskGetTickCount() - debounceTimeout; // Check for intervals of last interrupt
                saveLastState = flag;                                      // Saves next supposed state of trigger

                int currentState = gpio_get_level(13);                     // Gets current state

                if ((save != 0)  && (currentState == saveLastState) && (saveDebounceTimeout > DEBOUNCETIME )) {
                        printf("Button Interrupt Triggered %d times, current State=%d, time since last trigger %dms\n", save, currentState, saveDebounceTimeout);
                        flag ^= 1;                      // Flips LED state
                        look_flag = 1;
                        gpio_set_level(LEDPIN,flag);    // Set LED state
                        numberOfButtonInterrupts = 0;   // Reset
                        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay each noise signal
                }
                vTaskDelay(50 / portTICK_PERIOD_MS);    // Delay each check
        }
}

static void task_temperature() {
        check_efuse();

        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);

        //Characterize ADC
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);

        while (1) {
                uint32_t adc_reading = 0;

                for (int i = 0; i < NO_OF_SAMPLES; i++)
                        adc_reading += adc1_get_raw((adc1_channel_t)channel);

                adc_reading /= NO_OF_SAMPLES;
                //Convert adc_reading to voltage in mV
                uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
                temp = ohm2C(voltage);
                printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
                printf("%3.1f C\n", temp); // Prints

                vTaskDelay(pdMS_TO_TICKS(2000));
        }

}

static void task_time() {
        while (1) {
                if (second == water_time) {
                        second = 0;
                        if (!water_flag) // if 1, doesnt trigger again
                                water_flag ^= 1;
                }
                else
                        second = second + 1;
                vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
}

static void task_waterTimer() {
        gpio_pad_select_gpio(WATER_LED);
        gpio_set_direction(WATER_LED, GPIO_MODE_OUTPUT);

        int mode = 0;
        while (1) {
                if (water_flag) {
                        if (look_flag) {
                                look_flag = 0;
                                water_flag = 0;
                                gpio_set_level(WATER_LED, 0);
                        }
                        else if (!look_flag) {
                                for (int i = 0; i < 7; i++) {
                                        mode ^= 1;
                                        gpio_set_level(WATER_LED, mode);
                                        vTaskDelay(500 / portTICK_PERIOD_MS);
                                }
                        }

                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
        }
}

static void task_alert() {
        gpio_pad_select_gpio(ALERT_LED);
        gpio_set_direction(ALERT_LED, GPIO_MODE_OUTPUT);

        int mode = 0;
        while (1) {
                if (control_flag) {
                        if (look_flag) {
                                look_flag = 0;
                                control_flag = 0;
                                gpio_set_level(ALERT_LED, 0);
                        }
                        else if (!look_flag) {
                                for (int i = 0; i < 8; i++) {
                                        mode ^= 1;
                                        gpio_set_level(ALERT_LED, mode);
                                        vTaskDelay(500 / portTICK_PERIOD_MS);
                                }
                        }

                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
        }
}

static void task_battery() {

        ledc_timer_config_t ledc_timer = {
                .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
                .freq_hz = 5000,        // frequency of PWM signal
                .speed_mode = LEDC_HS_MODE, // timer mode
                .timer_num = LEDC_HS_TIMER, // timer index
                .clk_cfg = LEDC_AUTO_CLK, // Auto select the source clock
        };
        // Set configuration of timer0 for high speed channels
        ledc_timer_config(&ledc_timer);

        ledc_timer.speed_mode = LEDC_LS_MODE;
        ledc_timer.timer_num = LEDC_LS_TIMER;
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
                {
                        .channel    = LEDC_HS_CH0_CHANNEL,
                        .duty       = 0,
                        .gpio_num   = LEDC_HS_CH0_GPIO,
                        .speed_mode = LEDC_HS_MODE,
                        .hpoint     = 0,
                        .timer_sel  = LEDC_HS_TIMER
                }
        };

        ledc_channel_config(&ledc_channel[0]);

        while (1) {
                for (int m = 4; m >= 0; m--) {
                        int num = (int) pow(LEDC_TEST_DUTY, (1/10.0)*m);

                        ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, num); // logscale steps
                        ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
                        // printf("Duty Res: %d\n", num);
                        vTaskDelay(800 / portTICK_PERIOD_MS);
                }
        }
}

static void task_UDPcomm(void *pvParameters) {

        char rx_buffer[128];
        char addr_str[128];
        int addr_family;
        int ip_protocol;

        char payload[30];
        while (1) {

                struct sockaddr_in dest_addr;
                dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_port = htons(PORT);
                addr_family = AF_INET;
                ip_protocol = IPPROTO_IP;
                inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);


                int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
                if (sock < 0) {
                        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                        break;
                }
                ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

                while (1) {
                        int ins = 0;
                        int ins_val = 0;

                        sprintf(payload, "%d %2.2f %2.2f", steps, temp, battery);
                        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                        if (err < 0) {
                                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                                break;
                        }
                        payload[0] = '0';

                        struct sockaddr_in source_addr;         // Large enough for both IPv4 or IPv6
                        socklen_t socklen = sizeof(source_addr);
                        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

                        // Error occurred during receiving
                        if (len < 0) {
                                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                                break;
                        }
                        // Data received
                        else {
                                rx_buffer[len] = 0;         // Null-terminate whatever we received and treat like a string
                                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                                ESP_LOGI(TAG, "%s", rx_buffer);
                                sscanf(rx_buffer, "Ins: %d %d", &ins, &ins_val);
                                printf("%d %d\n", ins, ins_val);
                                control(ins, ins_val);
                        }

                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                }

                if (sock != -1) {
                        ESP_LOGE(TAG, "Shutting down socket and restarting...");
                        shutdown(sock, 0);
                        close(sock);
                }
        }
        vTaskDelete(NULL);
}


void app_main() {
        // Routine
        i2c_master_init();
        i2c_scanner();

        // Check for ADXL343
        uint8_t deviceID;
        getDeviceID(&deviceID);
        if (deviceID == 0xE5) {
                printf("\n>> Found ADAXL343\n");
        }

        // Disable interrupts
        writeRegister(ADXL343_REG_INT_ENABLE, 0);

        // Set range
        setRange(ADXL343_RANGE_2_G);
        // Display range
        printf  ("- Range:         +/- ");
        switch(getRange()) {
        case ADXL343_RANGE_16_G:
                printf  ("16 ");
                break;
        case ADXL343_RANGE_8_G:
                printf  ("8 ");
                break;
        case ADXL343_RANGE_4_G:
                printf  ("4 ");
                break;
        case ADXL343_RANGE_2_G:
                printf  ("2 ");
                break;
        default:
                printf  ("?? ");
                break;
        }
        printf(" g\n");

        // Display data rate
        printf ("- Data Rate:    ");
        switch(getDataRate()) {
        case ADXL343_DATARATE_3200_HZ:
                printf  ("3200 ");
                break;
        case ADXL343_DATARATE_1600_HZ:
                printf  ("1600 ");
                break;
        case ADXL343_DATARATE_800_HZ:
                printf  ("800 ");
                break;
        case ADXL343_DATARATE_400_HZ:
                printf  ("400 ");
                break;
        case ADXL343_DATARATE_200_HZ:
                printf  ("200 ");
                break;
        case ADXL343_DATARATE_100_HZ:
                printf  ("100 ");
                break;
        case ADXL343_DATARATE_50_HZ:
                printf  ("50 ");
                break;
        case ADXL343_DATARATE_25_HZ:
                printf  ("25 ");
                break;
        case ADXL343_DATARATE_12_5_HZ:
                printf  ("12.5 ");
                break;
        case ADXL343_DATARATE_6_25HZ:
                printf  ("6.25 ");
                break;
        case ADXL343_DATARATE_3_13_HZ:
                printf  ("3.13 ");
                break;
        case ADXL343_DATARATE_1_56_HZ:
                printf  ("1.56 ");
                break;
        case ADXL343_DATARATE_0_78_HZ:
                printf  ("0.78 ");
                break;
        case ADXL343_DATARATE_0_39_HZ:
                printf  ("0.39 ");
                break;
        case ADXL343_DATARATE_0_20_HZ:
                printf  ("0.20 ");
                break;
        case ADXL343_DATARATE_0_10_HZ:
                printf  ("0.10 ");
                break;
        default:
                printf  ("???? ");
                break;
        }
        printf(" Hz\n\n");

        // Enable measurements
        writeRegister(ADXL343_REG_POWER_CTL, 0x08);
        xTaskCreate(task_accel, "task_accel",1024*4, NULL, MAX_PRIO, NULL);

        ESP_ERROR_CHECK(nvs_flash_init());
        tcpip_adapter_init();
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(example_connect());
        xTaskCreate(task_UDPcomm, "task_UDPcomm", 4096, NULL, MAX_PRIO - 2, NULL);

        xTaskCreate(task_time, "task_time",1024, NULL, MAX_PRIO - 1, NULL);
        xTaskCreate(task_temperature, "task_temperature",1024*2, NULL, MAX_PRIO, NULL);
        xTaskCreate(task_alert, "task_alert",1024*2, NULL, MAX_PRIO, NULL);
        xTaskCreate(task_vib, "task_vib",1024*2, NULL, MAX_PRIO - 1, NULL);
        xTaskCreate(task_waterTimer, "task_waterTimer",1024, NULL, MAX_PRIO + 1, NULL);
        xTaskCreate(task_battery, "task_battery",1024, NULL, MAX_PRIO + 1, NULL);

        // while (1) {
        //         // if (look_flag)
        //         //         printf("TEST\n");
        //         vTaskDelay(10 / portTICK_RATE_MS);
        // }
}
