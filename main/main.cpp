#include <string>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_littlefs.h"
#include "esp_sleep.h"
#include "Support.h"

//Defines
#define ADC_SAMPLE_COUNT 30

//Logging tags
static const char* TAG = "ADC_TAG";
static const char* TAG_FAIL = "FAIL_SEND";
static const char* TAG_I2C = "I2C";

//Adc variables
static int adc_raw[2][10], humidity_millivolts;
static int voltage[2][10];


int32_t adc_raw_sum = 0;

adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t battery_cali_handle = NULL;
adc_cali_handle_t humidity_cali_handle = NULL;

bool calibration_battery_executed = false;
bool calibration_humidity_executed = false;
bool calibration_temperature_executed = false;

uint16_t millivolts_water = 220;  //2200  1800
uint16_t millivolts_air = 1000;  //3250  3150

i2c_master_bus_handle_t i2c_bus = NULL;
i2c_master_dev_handle_t i2c_dev = NULL;

running_state local_operation = SETTING_UP;
sensors_data myData;

//Temperature sensor variables
temperature_sensor_handle_t temp_handle = NULL;
float temperature = 255.0;

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(250)); // to give the sensor enough time to start
    init_nvs();
    init_littlefs();
    init_adc(&adc_handle);
    config_gpio(DONE_PIN, GPIO_MODE_OUTPUT);  
    config_gpio(LED_PIN, GPIO_MODE_INPUT);
    gpio_set_level(DONE_PIN, 0);
    calibration_battery_executed = adc_calibration_init(ADC_UNIT_1, BATTERY_ADC_CH, BATT_ADC_ATTEN, &battery_cali_handle); 
    calibration_humidity_executed = adc_calibration_init(ADC_UNIT_1, HUMIDIDY_ADC_CH, HUMIDIDY_ADC_ATTEN, &humidity_cali_handle); 

/*    if (i2c_master_init(&i2c_bus) != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialization failed");
    }
    if (i2c_add_HTU21D_sensor(i2c_bus, &i2c_dev) != ESP_OK) {
        ESP_LOGE(TAG, "I2C slave initialization failed");
    }
*/
    std::vector<uint8_t> read_operation = read_from_file_as_uint8_vector("/storage/opMode.bin");
    if (!read_operation.empty()) {
        local_operation = (running_state)read_operation[0];
        ESP_LOGI(TAG, "Operation mode: %d", local_operation);
    } else {
        local_operation = SETTING_UP;
        write_uint8_array_to_file("/storage/opMode.bin", (uint8_t*)&local_operation, 1);
        ESP_LOGE(TAG, "Failed to read operation mode from file");
    }    

    std::vector<uint8_t> read_heartbeat = read_from_file_as_uint8_vector("/storage/heartbeat.bin");
    if (!read_heartbeat.empty()) {
        if(read_heartbeat.size() > 1){
            myData.heartBeat = read_heartbeat[0] << 8 | read_heartbeat[1];
        }
        else{
            myData.heartBeat = read_heartbeat[0];
        }
        ESP_LOGI(TAG, "Heartbeat: %d", myData.heartBeat);
    } 
    else {
        myData.heartBeat = 91;
        ESP_LOGE(TAG, "Failed to read heartbeat from file");
    }    

    myData.heartBeat++;
    {
        uint8_t temp_heartbeat[2];
        temp_heartbeat[0] = myData.heartBeat >> 8;
        temp_heartbeat[1] = myData.heartBeat & 0xFF;
        write_uint8_array_to_file("/storage/heartbeat.bin", &temp_heartbeat[0], 2);
    }

    init_temp_sens(&temp_handle);
    temperature = get_temperature(&temp_handle);
    myData.internal_temperature_x10 = (int16_t)(temperature * 10);
    ESP_LOGI(TAG, "Internal temperature: %.02f", temperature);

    //Do all ADC measurements before starting Wifi to save some energy
    // Measure battery voltage
    adc_raw_sum = 0;
    for(int i=0;i<ADC_SAMPLE_COUNT;i++){
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, BATTERY_ADC_CH, &adc_raw[0][0]));
        adc_raw_sum += adc_raw[0][0];
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (calibration_battery_executed) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(battery_cali_handle, (adc_raw_sum/ADC_SAMPLE_COUNT), &voltage[0][0]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BATTERY_ADC_CH, voltage[0][0]);
        myData.battery_level = scale_adc_millivolts_to_battery_voltage(voltage[0][0]);
        ESP_LOGI(TAG, "Battery voltage is %d millivolts", myData.battery_level);
    }

    // Measure humidity
    adc_raw_sum = 0;
    for(int i=0;i<ADC_SAMPLE_COUNT;i++){
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, HUMIDIDY_ADC_CH, &adc_raw[0][0]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, HUMIDIDY_ADC_CH, adc_raw[0][0]);
        adc_raw_sum += adc_raw[0][0];
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (calibration_humidity_executed) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(humidity_cali_handle, (adc_raw_sum/ADC_SAMPLE_COUNT), &voltage[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, HUMIDIDY_ADC_CH, voltage[0][0]);
        humidity_millivolts = voltage[0][0];
        myData.soil_humidity_percentage = scale_adc_millivolts_to_humidity_percentage(voltage[0][0], millivolts_water, millivolts_air); 
        ESP_LOGI(TAG, "Scaled value is %d", myData.soil_humidity_percentage);
    }

/*
    soft_reset_HTU21D(i2c_dev);
    myData.ambient_temperature_x10 = (int16_t)(get_temperature_from_HTU21D(i2c_dev) * 10);
    myData.soil_humidity_percentage = (uint8_t)get_humidity_from_HTU21D(i2c_dev);
    ESP_LOGI(TAG, "Humidity: %d %%", myData.soil_humidity_percentage);
    ESP_LOGI(TAG, "Temperature: %d°C", myData.ambient_temperature_x10);
*/
    //Moved this here to save some energy
    wifi_init();

    local_operation = SETTING_UP;
    
    while(1) {
        switch(local_operation){
            case SETTING_UP:
                wifi_init_ap();
                start_http_server();
                vTaskDelay(pdMS_TO_TICKS(100000));
                vTaskDelay(pdMS_TO_TICKS(100000));
                local_operation = SERVER;
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            case SERVER:
                local_operation = READING;
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            
            case READING:
                adc_raw_sum = 0;
                for(int i=0;i<ADC_SAMPLE_COUNT;i++){
                    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, HUMIDIDY_ADC_CH, &adc_raw[0][0]));
                    //ESP_LOGI(TAG_I2C, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, HUMIDIDY_ADC_CH, adc_raw[0][0]);
                    adc_raw_sum += adc_raw[0][0];
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                if (calibration_humidity_executed) {
                    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(humidity_cali_handle, (adc_raw_sum/ADC_SAMPLE_COUNT), &voltage[0][0]));
                    ESP_LOGI(TAG_I2C, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, HUMIDIDY_ADC_CH, voltage[0][0]);
                    humidity_millivolts = voltage[0][0];
                    myData.soil_humidity_percentage = scale_adc_millivolts_to_humidity_percentage(voltage[0][0], millivolts_water, millivolts_air); 
                    ESP_LOGI(TAG_I2C, "Scaled value is %d", myData.soil_humidity_percentage);
                }   

                // Measure battery voltage
                adc_raw_sum = 0;
                for(int i=0;i<ADC_SAMPLE_COUNT;i++){
                    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, BATTERY_ADC_CH, &adc_raw[0][0]));
                    adc_raw_sum += adc_raw[0][0];
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                if (calibration_battery_executed) {
                    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(battery_cali_handle, (adc_raw_sum/ADC_SAMPLE_COUNT), &voltage[0][0]));
                    ESP_LOGI(TAG_I2C, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BATTERY_ADC_CH, voltage[0][0]);
                    myData.battery_level = scale_adc_millivolts_to_battery_voltage(voltage[0][0]);
                    ESP_LOGI(TAG_I2C, "Battery voltage is %d millivolts", myData.battery_level);
                }
                
                send_to_thingspeak(&myData);
                vTaskDelay(pdMS_TO_TICKS(20000));
                break;
            case RESET:
                static int reset_counter = 0;
                config_gpio(LED_PIN, GPIO_MODE_OUTPUT);
                gpio_set_level(LED_PIN, 1); //LED is inverted mode  
                
                if(gpio_get_level(RESET_PIN) == 0)
                {
                    ESP_LOGI(TAG_FAIL, "Reset pin: %d", gpio_get_level(RESET_PIN));
                    ESP_LOGI(TAG_FAIL, "Reset Counter value: %d", reset_counter);
                    reset_counter++;
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    if(reset_counter > 3){
                        local_operation = SETTING_UP;
                        myData.heartBeat = 0;
                        write_uint8_array_to_file("/storage/opMode.bin", (uint8_t*)&local_operation, 1);
                        write_uint8_array_to_file("/storage/heartbeat.bin", (uint8_t*)&myData.heartBeat, 1);

                        reset_counter = 0;
                        gpio_set_level(LED_PIN, 0); //LED ON
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        gpio_set_level(LED_PIN, 1); //LED is inverted mode   
                        config_gpio(LED_PIN, GPIO_MODE_INPUT); 
                    }
                }
                else{
                    reset_counter = 0;
                    local_operation = SERVER;
                    config_gpio(LED_PIN, GPIO_MODE_INPUT);
                }
                break;
            default:
                // Should never reach here
            break;

        }
    }
}