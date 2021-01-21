/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"


#define configMAX_PRIORITIES 6

#define SERVO_GPIO_0 12
#define SERVO_GPIO_1 27

// SECOND SERVO
#define SERVO_MIN_PULSEWIDTH_S 350 //Minimum pulse width in microsecond try 500
#define SERVO_MAX_PULSEWIDTH_S 2550 //Maximum pulse width in microsecond try 2400
#define SERVO_MAX_DEGREE_S 60 //Maximum angle in degree upto which servo can rotate


// MINUTE SERVO
#define SERVO_MIN_PULSEWIDTH_M 250 //Minimum pulse width in microsecond try 500
#define SERVO_MAX_PULSEWIDTH_M 2600 //Maximum pulse width in microsecond try 2400
#define SERVO_MAX_DEGREE_M 60 //Maximum angle in degree upto which servo can rotate

#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

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

// Startup function for Adafruit Display, GPIO, Servo
// ==========================================================================================

static void i2c_example_master_init(){
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

int testConnection(uint8_t devAddr, int32_t timeout) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return err;
}

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
        if (count == 0)
                printf("- No I2C devices found!" "\n");
        printf("\n");
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
}

static void mcpwm_example_gpio_initialize(void)
{
        printf("initializing mcpwm servo control gpio......\n");
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_GPIO_0);
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, SERVO_GPIO_1);
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation, uint32_t timedeg, uint32_t min, uint32_t max)
{
        uint32_t cal_pulsewidth = 0;
        cal_pulsewidth = min + (((max - min) * degree_of_rotation) / timedeg);
        return cal_pulsewidth;
}

// ==========================================================================================

// Function changes any int n into binary code to fit display
static int intprint(int n) {
        int i2cprint;

        switch (n) {
        case 0: i2cprint = 0b0000110000111111;
                break;
        case 1: i2cprint = 0b0000000000000110;
                break;
        case 2: i2cprint = 0b0000000011011011;
                break;
        case 3: i2cprint = 0b0000000010001111;
                break;
        case 4: i2cprint = 0b0000000011100110;
                break;
        case 5: i2cprint = 0b0010000001101001;
                break;
        case 6: i2cprint = 0b0000000011111101;
                break;
        case 7: i2cprint = 0b0000000000000111;
                break;
        case 8: i2cprint = 0b0000000011111111;
                break;
        case 9: i2cprint = 0b0000000011101111;
                break;
        default: i2cprint = 0b0000110000111111;
        }
        return i2cprint;
}

// Defining global variables to manage time, pausing and alarm
static int second = 0;
static int pause_flag = 0;
static int alarmtime = 0;
static int tripped = 0;

// Main task, in charge of ticking time & pausing
static void task_A()
{
        int minute = 0, mytime[4], mode = 0; // Initializing variables
        uint16_t displaybuffer[8]; // Define buffer to print

        // Initialize pin for alarm display
        static const uint8_t A0 = 26;

        gpio_pad_select_gpio(A0); // LED
        gpio_set_direction(A0, GPIO_MODE_OUTPUT);


        while (1) {

                // Timing logic for seconds, will not run if paused
                if (pause_flag== 0) {
                        second = (second == 3599) ? 0 : second + 1; // Loops time from 0 ~ 3599
                        minute = floor(second/60); // Minute is just seconds / 60 round down

                        if (second >= alarmtime && !tripped) { // Blinks while not tripped
                                mode = (mode == 0) ? 1 : 0;
                                gpio_set_level(A0, mode);
                        }
                        else if (second < alarmtime && tripped) // Tripped but before alarm was ready
                                tripped = 0;
                        else if (second > alarmtime && tripped) // Stops blinking when tripped
                                gpio_set_level(A0, 0);

                        // Change time into binary code for display, each digit of time is taken into consideration
                        mytime[0] = floor(minute / 10);
                        mytime[1] = minute % 10;
                        mytime[2] = floor((second % 60) / 10);
                        mytime[3] = second % 10;

                        for (int i = 0; i < 4; i++)
                                displaybuffer[i] = intprint(mytime[i]); // Updates display

                        // Send commands characters to display over I2C
                        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
                        i2c_master_start(cmd4);
                        i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
                        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
                        for (uint8_t i=0; i<8; i++) {
                                i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
                                i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
                        }
                        i2c_master_stop(cmd4);
                        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
                        i2c_cmd_link_delete(cmd4);

                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        printf("Minute: %d Second: %d", minute, second);
                }
        }
}

static void button()      // Define your first task here
{
        static const uint8_t A8 = 15;
        static const uint8_t A6 = 14;
        int level_pause[2] = {0,0};
        int level_alarm[2] = {0,0};

        gpio_pad_select_gpio(A6); // Alarm Off Button
        gpio_set_direction(A6, GPIO_MODE_INPUT);
        gpio_pad_select_gpio(A8);
        gpio_set_direction(A8, GPIO_MODE_INPUT);

        while(1) {
                level_alarm[1] = gpio_get_level(A6);
                level_pause[1] = gpio_get_level(A8);

                // counter = (counter == 0) ? 1 : 0;
                if ((level_pause[1] ^ level_pause[0]) == 1) {
                        if (level_pause[1] == 1)
                                pause_flag= (pause_flag== 0) ? 1 : 0;
                }
                if ((level_alarm[1] ^ level_alarm[0]) == 1) {
                        if (level_alarm[1] == 1) {
                                tripped = 1;
                                printf("TEST");
                        }

                }

                level_alarm[0] = level_alarm[1];
                level_pause[0] = level_pause[1];

                if (level_pause[1] == 1 || level_alarm[1] == 1) // Delay inbetween clicks
                        vTaskDelay(100 / portTICK_PERIOD_MS);
        }
}


void servo_1(void *arg)
{
        uint32_t angle, count;
        int mytime[2] = {0,0};
        //1. mcpwm gpio initialization
        mcpwm_example_gpio_initialize();

        //2. initial mcpwm configuration
        printf("Configuring Initial Parameters of mcpwm......\n");
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        pwm_config.cmpr_a = 0; //duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0; //duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);

        while (1) {
                count = second % 60;
                mytime[1] = count;

                if (pause_flag== 0) {
                        angle = servo_per_degree_init(count, SERVO_MAX_DEGREE_S, SERVO_MIN_PULSEWIDTH_S, SERVO_MAX_PULSEWIDTH_S);

                        if (mytime[1] != mytime[0])
                                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);

                        vTaskDelay(20);         //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
                }
                mytime[0] = mytime[1];
        }
}

void servo_2(void *arg)
{
        uint32_t angle, count;
        int mytime[2] = {0,0};
        //1. mcpwm gpio initialization
        mcpwm_example_gpio_initialize();

        //2. initial mcpwm configuration
        printf("Configuring Initial Parameters of mcpwm......\n");
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        pwm_config.cmpr_a = 0; //duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0; //duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0);

        while (1) {
                count = (int) floor(second/ 60);
                mytime[1] = count;

                if (pause_flag== 0) {
                        angle = servo_per_degree_init(count, SERVO_MAX_DEGREE_M, SERVO_MIN_PULSEWIDTH_M, SERVO_MAX_PULSEWIDTH_M);

                        if (mytime[1] != mytime[0])
                                mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, angle);

                        vTaskDelay(20);         //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
                }
                mytime[0] = mytime[1];
        }
}

void app_main(void)
{
        // Initiate UArt Driver
        ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );
        esp_vfs_dev_uart_use_driver(UART_NUM_0);

        i2c_example_master_init();
        i2c_scanner();
        test_alpha_display();

        while(1) {
                int min, sec, check;

                printf("Time Set: ");
                check = scanf("%d %d", &min, &sec);
                second = 60 * min + sec;


                if (check == 2){
                      printf("\nTime at %d %d\n",min, sec);
                        break;
                      }
        }

        while(1) {
                int min, sec, check;

                printf("Alarm Set: ");
                check = scanf("%d %d", &min, &sec);
                alarmtime = 60 * min + sec;


                if (check == 2) {
                        printf("\nAlarm at %d %d\n",min, sec);
                        break;
        }
      }

        xTaskCreate(button, "button",1024*2, NULL, configMAX_PRIORITIES-2, NULL);
        xTaskCreate(task_A, "task_A",4096, NULL, configMAX_PRIORITIES-2, NULL);
        xTaskCreate(servo_1, "servo_1",4096, NULL, configMAX_PRIORITIES-2, NULL);
        xTaskCreate(servo_2, "servo_2",4096, NULL, configMAX_PRIORITIES-2, NULL);
}
