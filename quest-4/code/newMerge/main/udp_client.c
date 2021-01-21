#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include <stdio.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

////Kelly edit///
#include <stdlib.h>
#include <math.h>
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "soc/gpio_sig_map.h"
#include "esp_types.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"


#include <stdio.h>
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_types.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "soc/uart_struct.h"
#include "string.h"

#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "driver/i2c.h"
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      1000
#define PCNT_THRESH1_VAL    500
#define PCNT_THRESH0_VAL   -500
#define PCNT_INPUT_SIG_IO   4  // Pulse Input GPIO A5

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (0.5) // sample test interval for the first timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload


// WHAT IS THIS ================================================================
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd/
// WHAT IS THIS ================================================================

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
// --------------------------------------
// Master I2C Lidar


// // ADXL343
#define SLAVE_ADDR_LID                   (0x62) // 0x63
#define ACQ_COMMAND                 (0x00)    /**< Assumes ALT address pin low */
#define ACQ_STATUS                   (0x01)
#define ACQ_LOW                 (0x10)
#define ACQ_HIGH                 (0x0f)
#define I2C_EXAMPLE_MASTER_NUM_LID             I2C_NUM_1  // i2c port
#define ACK_VAL_LID                            0x01 // i2c ack value
#define NACK_VAL_LID                           0xFF // i2c nack value

#define MAX_PRIORITY 6

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

#define F_TXD_PIN (14) // green 14
#define F_RXD_PIN (5) // white SCK
#define B_TXD_PIN 27  // green 27
#define B_RXD_PIN 15  //white 15

#define T_INTERVAL (.500)
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TEST_WITH_RELOAD      1

#define LED_R         12
#define LED_G         27
#define LED_B         33
// --------------------------------------
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   256          //Multisampling

//----------------
//#define LIDAR_OFFSET 30
//#define DISTANCE 15




// static esp_adc_cal_characteristics_t *adc_chars;
// static const adc_channel_t bchannel = ADC_CHANNEL_5;     //GPIO34 if ADC1, GPIO14 if ADC2
// static const adc_channel_t fchannel = ADC_CHANNEL_4;     //GPIO34 if ADC1, GPIO14 if ADC2
// static const adc_atten_t atten = ADC_ATTEN_DB_11;
// static const adc_unit_t unit = ADC_UNIT_1;


#define HOST_IP_ADDR "192.168.1.106"
#define PORT 8080

static const char *TAG = "Client";
char *payload = "Request";

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle
xQueueHandle timer_queue;

int cycles = 0, cycles_mem = 0;
double timeI, timeF = 0;
int ins = 1;
//double time = 0;
int nomoreI, nomoreF = 0;
double timeDifference = 0;
double speed = 0;
unsigned long counterPulses;
unsigned long counter_mem[4];

// ==============================================================
static const int RX_BUF_SIZE = 256;
//uint32_t b_dist = 0;
//uint32_t f_dist = 0;
uint32_t lidar_reading = 0;
uint32_t count = 45;
int dt_complete = 0;
int calibration = 0;
uint32_t dv_speed = 0;

// ==============================================================



typedef struct {
        int unit; // the PCNT unit that originated an interrupt
        uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

typedef struct {
        int type; // the type of timer's event
        int timer_group;
        int timer_idx;
        uint64_t timer_counter_value;
} timer_event_t;


static void IRAM_ATTR pcnt_example_intr_handler(void *arg) {
        uint32_t intr_status = PCNT.int_st.val;
        int i;
        pcnt_evt_t evt;
        portBASE_TYPE HPTaskAwoken = pdFALSE;

        for (i = 0; i < PCNT_UNIT_MAX; i++) {
                if (intr_status & (BIT(i))) {
                        evt.unit = i;
                        /* Save the PCNT event type that caused an interrupt
                           to pass it to the main program */
                        evt.status = PCNT.status_unit[i].val;
                        PCNT.int_clr.val = BIT(i);
                        xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
                        if (HPTaskAwoken == pdTRUE) {
                                portYIELD_FROM_ISR();
                        }
                }
        }
}

static void pcnt_example_init(void) {
        /* Prepare configuration for the PCNT unit */
        pcnt_config_t pcnt_config = {
                // Set PCNT input signal and control GPIOs
                .pulse_gpio_num = PCNT_INPUT_SIG_IO, // GPIO4
                .channel = PCNT_CHANNEL_0,
                .unit = PCNT_TEST_UNIT,
                // What to do on the positive / negative edge of pulse input?
                .pos_mode = PCNT_COUNT_INC, // Count up on the positive edge
                .neg_mode = PCNT_COUNT_DIS, // Keep the counter value on the negative edge
                // Set the maximum limit value
                .counter_h_lim = PCNT_H_LIM_VAL,
        };
        /* Initialize PCNT unit */
        pcnt_unit_config(&pcnt_config);

        /* Configure and enable the input filter */
        pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
        pcnt_filter_enable(PCNT_TEST_UNIT);

        /* Set threshold 0 and 1 values and enable events to watch */
        pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
        pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
        /* Enable events on zero, maximum and minimum limit values */
        pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
        pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
        // pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

        /* Initialize PCNT's counter */
        pcnt_counter_pause(PCNT_TEST_UNIT);
        pcnt_counter_clear(PCNT_TEST_UNIT);

        /* Register ISR handler and enable interrupts for PCNT unit */
        pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
        pcnt_intr_enable(PCNT_TEST_UNIT);

        /* Everything is set up, now go to counting */
        pcnt_counter_resume(PCNT_TEST_UNIT);
}

void IRAM_ATTR timer_group0_isr(void *para) {
        int timer_idx = (int) para;

        /* Retrieve the interrupt status and the counter value
           from the timer that reported the interrupt */
        timer_intr_t timer_intr = timer_group_intr_get_in_isr(TIMER_GROUP_0);
        uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

        /* Prepare basic event data
           that will be then sent back to the main program task */
        timer_event_t evt;
        evt.timer_group = 0;
        evt.timer_idx = timer_idx;
        evt.timer_counter_value = timer_counter_value;

        /* Clear the interrupt
           and update the alarm time for the timer with without reload */
        if (timer_intr & TIMER_INTR_T0) {
                evt.type = TEST_WITHOUT_RELOAD;
                timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_0);
                timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
                timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
        }

        /* After the alarm has been triggered
           we need enable it again, so it is triggered the next time */
        timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

        /* Now just send the event data back to the main program task */
        xQueueSendFromISR(timer_queue, &evt, NULL);
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

static void i2c_master_init_lid(){
        // Debug
        printf("\n>> i2c Config\n");
        int err;

        // Port configuration
        int i2c_master_port = I2C_EXAMPLE_MASTER_NUM_LID;

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



// Turn on oscillator for alpha display
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


int writeRegister(uint8_t reg, uint8_t data) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR_LID << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM_LID, cmd, 10 / portTICK_RATE_MS);
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
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM_LID, cmd, 10 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( SLAVE_ADDR_LID << 1 ) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS); //problem this doesnt read regist 0x01
        i2c_master_stop(cmd);
        int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM_LID, cmd, 10 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return data;
}
//read 16 bits
int16_t read16() {
        uint8_t highbyte = 0, lowbyte = 0;
        lowbyte = readRegister(ACQ_LOW);
        highbyte = readRegister(ACQ_HIGH);
        return ((uint16_t)highbyte)<< 8 | lowbyte;
}

// ==============================================================
int left_dist[2];

static void front_lid() {
        static const char *RX_TASK_TAG = "RX_TASK";
        esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
        uint8_t* fdata = (uint8_t*) malloc(RX_BUF_SIZE+1);
        uint8_t* bdata = (uint8_t*) malloc(RX_BUF_SIZE+1);
        uint16_t test = 0x59;
        while (1) {
                uint16_t flowbyte, fhighbyte;
                uint16_t blowbyte, bhighbyte;
                const int frxBytes = uart_read_bytes(UART_NUM_0, fdata, RX_BUF_SIZE, 750 / portTICK_RATE_MS);
                const int brxBytes = uart_read_bytes(UART_NUM_1, bdata, RX_BUF_SIZE, 750 / portTICK_RATE_MS);
                if (frxBytes > 0) {
                        fdata[frxBytes] = 0;
                        ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", frxBytes, fdata);

                        for(int i = 0; i < RX_BUF_SIZE; i++) {
                                //printf("%x\n", data[i]);
                                if((fdata[i] == test) && (fdata[i+1] == test)) {
                                        flowbyte = (uint16_t) fdata[i+2];
                                        fhighbyte = (uint16_t) fdata[i+3];

                                        if ((fhighbyte <= (uint16_t)4)) {
                                                left_dist[0] = (((uint32_t)fhighbyte) << 8 | flowbyte) + 16;
                                                left_dist[0] += 10;
                                                printf("Front Lidar: %d\n ", left_dist[0]);
                                                break;
                                        }
                                }
                        }
                }
                if (brxBytes > 0) {
                        bdata[brxBytes] = 0;
                        //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", brxBytes, bdata);

                        for(int i = 0; i < RX_BUF_SIZE; i++) {
                                //printf("%x\n", data[i]);
                                if((bdata[i] == test) && (bdata[i+1] == test)) {
                                        blowbyte = (uint16_t) bdata[i+2];
                                        bhighbyte = (uint16_t) bdata[i+3];

                                        if ((bhighbyte <= (uint16_t)4)) {
                                                left_dist[1] = (((uint32_t)bhighbyte) << 8 | blowbyte) + 16;
                                                printf("Back Lidar: %d\n ", left_dist[1]);
                                                break;
                                        }
                                }
                        }
                }
        }//runs 200 tests per second;
        free(fdata);
        free(bdata);
}

static void example_tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec) {
        /* Select and initialize basic parameters of the timer */
        timer_config_t config;
        config.divider = TIMER_DIVIDER;
        config.counter_dir = TIMER_COUNT_UP;
        config.counter_en = TIMER_PAUSE;
        config.alarm_en = TIMER_ALARM_EN;
        config.intr_type = TIMER_INTR_LEVEL;
        //config.auto_reload = auto_reload;
        timer_init(TIMER_GROUP_0, timer_idx, &config);

        /* Timer's counter will initially start from value below.
           Also, if auto_reload is set, this value will be automatically reload on alarm */
        timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

        /* Configure the alarm value and the interrupt on alarm. */
        timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
        timer_enable_intr(TIMER_GROUP_0, timer_idx);
        timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
                           (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

        timer_start(TIMER_GROUP_0, timer_idx);
}

double calcWheelSpeed(double interval, double cycle) {
        //printf("Cycle: %f\n", cycle);
        return 0.62 * cycle / interval;
}

void displaySpeed(double spd) {
        // printf("4.  SPEED  : %.4f m/s\n\n\n\n", spd);
        uint16_t displaybuffer[8];

        int temp = (int) (spd * 1000);
        //printf("temp: %d\n", temp);

        int d1 = temp % 10;
        int d2 = temp /10 % 10;
        int d3 = temp /100 % 10;
        int d4 = temp /1000 % 10;

        //printf("d1: %d\nd2: %d\nd3: %d\nd4: %d\n", d1, d2, d3, d4);

        int real1 = display_number(d1);
        int real2 = display_number(d2);
        int real3 = display_number(d3);
        int real4 = display_number(d4);

        // Write to characters to buffer
        displaybuffer[3] = real1;
        displaybuffer[2] = real2;
        displaybuffer[1] = real3;
        displaybuffer[0] = real4 | 0b0100000000000000;

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

        vTaskDelay(100/portTICK_RATE_MS);
}

static void timer_example_evt_task(void *arg) {
        double time = 0;
        while (1) {
                int cycle_diff = 0;
                timer_event_t evt;
                xQueueReceive(timer_queue, &evt, portMAX_DELAY);

                time = (double) evt.timer_counter_value / TIMER_SCALE;
                // printf("Time   : %.3f s\n", time);

                // printf("cycles: %d nomoreI: %d nomoreF: %d\n", cycles, nomoreI, nomoreF);

                if (counterPulses < 7 && nomoreI == 0) {
                        timeI = (double) evt.timer_counter_value / TIMER_SCALE;
                        nomoreI = 1;

                }
                if (counterPulses  >= 7 && nomoreF == 0) {
                        timeF = (double) evt.timer_counter_value / TIMER_SCALE;
                        nomoreF = 1;
                }

                timeDifference = timeF - timeI;

                cycle_diff = cycles - cycles_mem;
                // printf("3.  %d CYCLE!\n",cycle_diff);
                speed = calcWheelSpeed(timeDifference, cycle_diff);

                if (counterPulses == counter_mem[3] && counter_mem[3] == counter_mem[0]) {
                        displaySpeed(0);
                        timeI = (double) evt.timer_counter_value / TIMER_SCALE;
                        cycles_mem = cycles;
                }
                else
                        displaySpeed(speed);

                timeDifference = 0;
                speed = 0;
                nomoreI = 0;
                nomoreF = 0;

                // uint64_t task_counter_value;
                // timer_get_counter_value(evt.timer_group, evt.timer_idx, &task_counter_value);
        }
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

/*static void mcpwm_example_gpio_initialize(void) {
        printf("initializing mcpwm servo control gpio......\n");
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18); //Set GPIO 18 as PWM0A, to which servo is connected
   }*/

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
        uint32_t cal_pulsewidth = 0;
        cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
        return cal_pulsewidth;
}

void init() {
        const uart_config_t uart_config = {
                .baud_rate = 115200,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };
        uart_param_config(UART_NUM_1, &uart_config);
        uart_set_pin(UART_NUM_1, B_TXD_PIN, B_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);

        uart_param_config(UART_NUM_0, &uart_config);
        uart_set_pin(UART_NUM_0, F_TXD_PIN, F_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);

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

/*static void print_char_val_type(esp_adc_cal_value_t val_type) {
        if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
                printf("Characterized using Two Point Value\n");
        } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
                printf("Characterized using eFuse Vref\n");
        } else {
                printf("Characterized using Default Vref\n");
        }
   }

   static float bin2cm(uint32_t adc_reading) {
        float dist = (float)adc_reading * 5;
        return (300 + dist)/10;
   }
 */
static void countPuleses() {
        //int16_t count = 0;
        unsigned long multPulses = 0;
        pcnt_evt_t evt;
        portBASE_TYPE res;
        while (1) {
                /* Wait for the event information passed from PCNT's interrupt handler.
                 * Once received, decode the event type and print it on the serial monitor.
                 */
                //printf("TIME in main: %.8f\n", time);
                res = xQueueReceive(pcnt_evt_queue, &evt, 500 / portTICK_PERIOD_MS);
                if (res == pdTRUE) {
                        pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
                        printf("Event PCNT unit[%d]; cnt: %d\n", evt.unit, count);
                        if (evt.status & PCNT_STATUS_THRES1_M) {
                                printf("THRES1 EVT\n");
                        }
                        if (evt.status & PCNT_STATUS_H_LIM_M) {

                                printf("H_LIM EVT\n");
                        }
                        if (evt.status & PCNT_STATUS_ZERO_M) {
                                multPulses++;
                                printf("ZERO EVT\n");
                        }
                } else {
                        for (int i = 0; i < 3; i++) {
                                counter_mem[i + 1] = counter_mem[i];
                        }
                        counter_mem[0] = counterPulses;
                        // printf("2.  current counter_mem :%lu %lu %lu %lu\n", counter_mem[0],counter_mem[1],counter_mem[2],counter_mem[3] );
                        pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
                        counterPulses = multPulses * PCNT_H_LIM_VAL + count;
                        // printf("1.  Current counter pulse :%lu\n", counterPulses);

                        //initialTime = counterPulses - counter_mem;

                        if (counterPulses >= 7) {
                                cycles = (int)(counterPulses / 7);
                                //counter_mem+= cycles * 12;
                                //counter_mem = counterPulses;
                        } else{
                                cycles = 0;
                                //printf("its not fast enough XD\n" );
                        }


                }
        }
}

///////////Kelly edit done/////////

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

                while (1) {
                        // Uses UART to get input from user, uses this control UDP client
                        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                        if (err < 0) {
                                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                                break;
                        }
                        ESP_LOGI(TAG, "Message sent");

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
                                sscanf(rx_buffer, "%d", &ins);
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

void esc_start(void *arg) {
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);
        //2. initial mcpwm configuration
        printf("Configuring Initial Parameters of ESC......\n");
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        pwm_config.cmpr_a = 0; //duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0; //duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings

        vTaskDelay(3000 / portTICK_PERIOD_MS); // Give yourself time to turn on crawler
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // LOW signal in microseconds
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value

        while (1) {

                //printf("Lidar Reading: %d\n", (int)lidar_reading);


                if(ins == 1) {
                        if (lidar_reading > 90)
                                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (uint32_t)1280); // LOW signal in microseconds 1310
                        else
                                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (uint32_t)1400);
                }
                else
                        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (uint32_t)1400); // LOW signal in microseconds



                vTaskDelay(100/portTICK_PERIOD_MS);
        }
}

void servo_start(void *arg) {
        uint32_t angle = 0;
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, 21);
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        pwm_config.cmpr_a = 0; //duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0; //duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        printf("Configuring Initial Parameters of steering......\n");
        mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);         //Configure PWM0A & PWM0B with above settings

        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, servo_per_degree_init(45));
        vTaskDelay(500 / portTICK_PERIOD_MS);
        while (1) {

                angle = servo_per_degree_init(count);


                //printf("Angle of rotation: %d\n", angle);

                mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
                vTaskDelay(10);
        }
}

float right_dist[2];

/*void ultra() {

        adc1_config_width(ADC_WIDTH_BIT_10);         // 10 bits
        adc1_config_channel_atten(fchannel, atten);
        adc1_config_channel_atten(bchannel, atten);

        //Characterize ADC
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);

        //Continuously sample ADC1
        while (1) {
                uint32_t fadc_reading = 0;
                uint32_t badc_reading = 0;

                for (int i = 0; i < NO_OF_SAMPLES; i++) {
                        fadc_reading += adc1_get_raw((adc1_channel_t)fchannel);
                        badc_reading += adc1_get_raw((adc1_channel_t)bchannel);
                }
                fadc_reading /= NO_OF_SAMPLES;
                badc_reading /= NO_OF_SAMPLES;
                //Convert adc_reading to voltage in mV
                right_dist[0] = bin2cm(fadc_reading);
                right_dist[1] = bin2cm(badc_reading);
                vTaskDelay(pdMS_TO_TICKS(750));
        }
   }*/

static uint32_t F_Lidar(){
// Routine
        i2c_master_init_lid();

        uint8_t data;

        while(1) {
                writeRegister(ACQ_COMMAND, 0x04);
                while (1) {
                        data = readRegister(ACQ_STATUS);
                        if ((data & 1) != 1)
                                break;
                        vTaskDelay(20/portTICK_RATE_MS);
                }
                lidar_reading = read16();
                printf("BIGGIE: %d\n", (int)lidar_reading);
                vTaskDelay(200/portTICK_RATE_MS);
        }
        return lidar_reading;
}

void app_main(void)
{
        // Initializes UART
        ESP_ERROR_CHECK(nvs_flash_init());
        tcpip_adapter_init();
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(example_connect());

        pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
        timer_queue = xQueueCreate(10, sizeof(timer_event_t));

        pcnt_example_init();
        i2c_master_init();
        i2c_scanner();
        init();
        test_alpha_display();
        check_efuse();

        //xTaskCreate(udp_client_task, "udp_client_task", 4096, NULL, MAX_PRIORITY -2, NULL);
        example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
        xTaskCreate(timer_example_evt_task, "timer_example_evt_task", 2048, NULL, MAX_PRIORITY, NULL);
        xTaskCreate(countPuleses, "countPuleses", 2048, NULL, MAX_PRIORITY, NULL);


        //xTaskCreate(ultra, "ultra", 2048, NULL, MAX_PRIORITY - 1, NULL);
        xTaskCreate(front_lid, "front_lid", 2048, NULL, MAX_PRIORITY-1, NULL);

        xTaskCreate(F_Lidar, "F_Lidar", 2048, NULL, MAX_PRIORITY-2, NULL);


        printf("Testing servo motor.......\n");

        xTaskCreate(servo_start, "servo_start", 2048, NULL, 5, NULL);
        xTaskCreate(esc_start, "esc_start", 2048, NULL, 3, NULL);


        //float output = 0

        //float forwardOutput = 0, previous_forward_error = 0, forwardIntegral = 0, forwardError = 0;

        uint32_t angle = 0;
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, 21);
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        pwm_config.cmpr_a = 0; //duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0; //duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        printf("Configuring Initial Parameters of steering......\n");
        mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);         //Configure PWM0A & PWM0B with above settings

        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, servo_per_degree_init(45));

        float previous_error = 0, integral = 0; //moved it
        int error = 0; //moved it

        while(1) {
                //ins to 1: start or 0: stop
                //lidar_reading: front lidar
                //left_dist[0]: front lidar; left_dist[1]: back lidar;

                //float Kp = 0.5, Ki = 0.01, Kd = 1;
                float dt = (float)T_INTERVAL;

                error = (left_dist[0] - left_dist[1]);

                integral += error * dt;
                previous_error = error;
                if (error < 0) {
                        if ((45 + error) > 0)
                                count = 45 + error;

                        else
                                count = 45;
                }
                else if (error > 0) {
                        if ((45 + error) < SERVO_MAX_DEGREE )
                                count = 45 + error;
                        else
                                count = SERVO_MAX_DEGREE;
                }
                else {
                        count = 45;
                }

                angle = servo_per_degree_init(count);


                //printf("Angle of rotation: %d\n", angle);

                mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, angle);

                vTaskDelay(100/portTICK_RATE_MS);
        }


}
