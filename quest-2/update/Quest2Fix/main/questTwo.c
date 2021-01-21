#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"

#define CONST_B 3435 // B constant
#define CONST_K 298.15 // Room temp in K
#define CONST_R 15000

#define MAX_PRIORITY 6

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0;     //GPIO36 IR
static const adc_channel_t channel2 = ADC_CHANNEL_3;     //GPIO39 Ultasonic
static const adc_channel_t channel3 = ADC_CHANNEL_4;     //GPIO32 Thermistor
static const adc_channel_t channel4 = ADC_CHANNEL_5;     //GPIO33 battery
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// Startup function for Adafruit Display, GPIO, Servo
// ==========================================================================================

static void check_efuse(void)
{
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

// ==========================================================================================

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
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

static float bin2cm(uint32_t adc_reading) {
        float dist = (float)adc_reading * 5;
        return (300 + dist)/10; // return cm
}

// ==============================
float temp = 0;
float cm = 0;
float centimeter = 0;
float volts = 0;

static void therm() {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel3, atten);

        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);

        vTaskDelay(pdMS_TO_TICKS(1000));
        while (1) {
                uint32_t adc_reading = 0;

                for (int i = 0; i < NO_OF_SAMPLES; i++) {
                        if (unit == ADC_UNIT_1) {
                                adc_reading += adc1_get_raw((adc1_channel_t)channel3);
                        }
                }
                adc_reading /= NO_OF_SAMPLES;
                //Convert adc_reading to voltage in mV
                uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
                temp = ohm2C(voltage);
                vTaskDelay(pdMS_TO_TICKS(1000));
        }
}

static void ultra() {

        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel2, atten);

        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);

        vTaskDelay(pdMS_TO_TICKS(1000));
        while (1) {
                uint32_t adc_reading = 0;

                for (int i = 0; i < NO_OF_SAMPLES; i++) {
                        if (unit == ADC_UNIT_1) {
                                adc_reading += adc1_get_raw((adc1_channel_t)channel2);
                        }
                }
                adc_reading /= NO_OF_SAMPLES;
                cm = bin2cm(adc_reading);
                vTaskDelay(pdMS_TO_TICKS(1000));
        }
}

static void infrared(){
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel2, atten);

        //Characterize ADC
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);

        vTaskDelay(pdMS_TO_TICKS(1000));
        while (1) {
                uint32_t adc_reading = 0;

                for (int i = 0; i < NO_OF_SAMPLES; i++) {
                        if (unit == ADC_UNIT_1) {
                                adc_reading += adc1_get_raw((adc1_channel_t)channel);
                        }
                }
                adc_reading /= NO_OF_SAMPLES;
                //Convert adc_reading to voltage in mV
                uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

                centimeter = volt2cm(voltage);
                vTaskDelay(pdMS_TO_TICKS(1000));
        }
}

static void battery() {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel4, atten);

        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);

        vTaskDelay(pdMS_TO_TICKS(1000));
        while (1) {
                uint32_t adc_reading = 0;

                for (int i = 0; i < NO_OF_SAMPLES; i++) {
                        if (unit == ADC_UNIT_1) {
                                adc_reading += adc1_get_raw((adc1_channel_t)channel4);
                        }
                }
                adc_reading /= NO_OF_SAMPLES;
                //Convert adc_reading to voltage in mV
                uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
                volts = (float)(voltage / 1000);
                vTaskDelay(pdMS_TO_TICKS(1000));
        }
}

void app_main(void)
{
        check_efuse();

        xTaskCreate(therm, "therm", 4096, NULL, MAX_PRIORITY-1, NULL);
        xTaskCreate(ultra, "ultra", 4096, NULL, MAX_PRIORITY-3, NULL);
        xTaskCreate(infrared, "infrared", 4096, NULL, MAX_PRIORITY, NULL);
        xTaskCreate(battery, "battery", 4096, NULL, MAX_PRIORITY-3, NULL);

        while(1) {
                if (volts == 0 || centimeter == 0 || cm == 0 || temp == 0) {
                        vTaskDelay(pdMS_TO_TICKS(500));
                        continue;
                }
                else {
                        printf("Volt: %2.2f IR: %2.2f Ultra: %2.2f Temp: %2.2f\n", volts,centimeter,cm,temp);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                }
        }
}
