#ifndef HUMIDITY_SENSOR_SPECIFIC_H
#define HUMIDITY_SENSOR_SPECIFIC_H

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "stdint.h"
#include "esp_http_client.h"
#include <vector>

#define DONE_PIN          GPIO_NUM_3
#define RESET_PIN         GPIO_NUM_9
#define LED_PIN           GPIO_NUM_10

#define HUMIDIDY_ADC_ATTEN  ADC_ATTEN_DB_12
#define BATT_ADC_ATTEN    ADC_ATTEN_DB_12
#define HUMIDIDY_ADC_CH   ADC_CHANNEL_0 // The soil humidity sensor must be connected to IO0
#define BATTERY_ADC_CH    ADC_CHANNEL_1 // The battery voltage divider must be connected to IO1

#define MAC_SIZE 6

#define I2C_MASTER_SCL_IO           GPIO_NUM_4      // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO           GPIO_NUM_5      // GPIO number for I2C master data
#define I2C_MASTER_NUM              I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ          400000   // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0        // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0        // I2C master doesn't need buffer

#define THINGSPEAK_API_KEY "SH5HQKJ0K659OG43"
#define THINGSPEAK_URL "http://api.thingspeak.com/update"

#define WIFI_SSID "your_ssid"
#define WIFI_PASS "your_password"

#define WIFI_CONNECTED_BIT BIT0

//Enums
enum running_state {SETTING_UP, SERVER, READING, RESET};

static EventGroupHandle_t s_wifi_event_group;

typedef struct sensors_data {
    running_state operation;
    uint8_t soil_humidity_percentage;
    uint8_t air_humidity_percentage;
    int16_t internal_temperature_x10;
    int16_t ambient_temperature_x10;
    uint16_t battery_level;          //millivolts
    uint16_t heartBeat;
} sensors_data;

typedef struct humidity_sensor_saved_values{
  uint8_t mac[MAC_SIZE];
  sensors_data data;
} humidity_sensor_saved_values;

uint16_t scale_adc_millivolts_to_battery_voltage(int adc_millivolts);
uint8_t scale_adc_millivolts_to_humidity_percentage(int adc_millivolts, uint16_t millivolts_water, uint16_t millivolts_air);

void init_nvs();
void wifi_init();
void wifi_auth_init();
void init_littlefs();

bool write_uint8_array_to_file(const char* filename, const uint8_t* data, size_t length);
bool append_uint8_array_to_file(const char* filename, const uint8_t* data, size_t length);
std::vector<uint8_t> read_from_file_as_uint8_vector(const char* filename);

void config_gpio(gpio_num_t gpio_num, gpio_mode_t mode);

float get_temperature(temperature_sensor_handle_t* ptemp_handle);
void init_temp_sens(temperature_sensor_handle_t* ptemp_handle);

esp_err_t i2c_master_init(i2c_master_bus_handle_t *i2c_bus);
esp_err_t i2c_add_HTU21D_sensor(i2c_master_bus_handle_t i2c_bus, i2c_master_dev_handle_t *i2c_dev);

void init_adc(adc_oneshot_unit_handle_t *padc_handle);
bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
void adc_calibration_deinit(adc_cali_handle_t handle);

float get_humidity_from_HTU21D(i2c_master_dev_handle_t i2c_dev);
float get_temperature_from_HTU21D(i2c_master_dev_handle_t i2c_dev);
void soft_reset_HTU21D(i2c_master_dev_handle_t i2c_dev);

void send_to_thingspeak(sensors_data *data);

#endif // GENERIC_FUNCTIONS_H