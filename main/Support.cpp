#include "Support.h"
#include "esp_wifi.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include <vector>
#include "nvs_flash.h"
#include "driver/temperature_sensor.h"

#define R1_VOL_DIV 2080 //2000 * 1.04 which is compensation for voltage drops
#define R2_VOL_DIV 3000

#define ESP_WIFI_SSID "ESP-WIFI"
#define ESP_WIFI_PASS "123456789"
#define ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char* TAG_FILE = "FILESYSTEM";
static const char* ADCTAG = "ADC";
static const char* IIC_TAG = "I2C";
static const char* TAG_WIFI = "WIFI";
static const char* TAG_HTTP = "HTTP_SERVER";

static EventGroupHandle_t wifi_event_group;

static int s_retry_num = 0;
uint8_t local_ssid[32] = {0};
uint8_t local_password[64] = {0};
uint8_t local_api_key[17] = {0};
running_state current_operation = SETTING_UP;
bool connected_to_internet = false;

void set_current_op_mode(running_state mode){
    current_operation = mode;
}

running_state get_current_operation_mode(){
    return current_operation;
}

bool get_connection_status(){
    return connected_to_internet;
}

uint8_t scale_adc_millivolts_to_humidity_percentage(int adc_millivolts, uint16_t millivolts_water, uint16_t millivolts_air){
    if(adc_millivolts < millivolts_water){
        adc_millivolts = millivolts_water;
    }
    else if(adc_millivolts > millivolts_air){
        adc_millivolts = millivolts_air;
    }
    return (uint8_t) (99 - (((adc_millivolts - millivolts_water) * 99) / (millivolts_air - millivolts_water))); // Made it 99 to avoid fit value in 2 digits
}

uint16_t scale_adc_millivolts_to_battery_voltage(int adc_millivolts){
    return (uint16_t)((adc_millivolts * (R1_VOL_DIV + R2_VOL_DIV)) / R2_VOL_DIV);
}

// Wifi related functions
void init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

}

void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        //esp_wifi_connect();
        if (s_retry_num < ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG_WIFI, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        // Add a delay to ensure the connection is fully established
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void wifi_auth_init(void)
{
    s_wifi_event_group = xEventGroupCreate(); // Initialize the event group
    wifi_mode_t current_mode;
    ESP_ERROR_CHECK(esp_wifi_get_mode(&current_mode));
    if (current_mode != WIFI_MODE_NULL) {
        ESP_ERROR_CHECK(esp_wifi_stop());
    }
    else{
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
    }

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config;

    strncpy((char*)wifi_config.sta.ssid, (char*)local_ssid, sizeof(wifi_config.sta.ssid) - 1);
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0'; // Ensure null-termination

    strncpy((char*)wifi_config.sta.password, (char*)local_password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0'; // Ensure null-termination

    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false; // Disable PMF requirement

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    connected_to_internet = true;
}

void wifi_init_ap() {
    s_wifi_event_group = xEventGroupCreate(); // Initialize the event group
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_mode_t current_mode;
    ESP_ERROR_CHECK(esp_wifi_get_mode(&current_mode));
    if (current_mode != WIFI_MODE_NULL) {
        ESP_ERROR_CHECK(esp_wifi_stop());
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    wifi_config_t ap_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = 1, // Set WiFi channel to 1
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .max_connection = 4, // Adjust the max connection number
            .beacon_interval = 100 // Increase beacon interval to 100ms
        },
    };
    if (strlen(ESP_WIFI_SSID) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Disable power save mode
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Configure the DHCP server
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(netif));

    // Set IP address, netmask, and gateway
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));

    // Start the DHCP server
    ESP_ERROR_CHECK(esp_netif_dhcps_start(netif));

    // Log the IP configuration
    ESP_LOGI(TAG_WIFI, "AP IP Address: " IPSTR, IP2STR(&ip_info.ip));
    ESP_LOGI(TAG_WIFI, "AP Gateway: " IPSTR, IP2STR(&ip_info.gw));
    ESP_LOGI(TAG_WIFI, "AP Netmask: " IPSTR, IP2STR(&ip_info.netmask));

    // Disable SA Query procedure (last resort)
    wifi_sta_config_t sta_config = {};
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_AP, (wifi_config_t*)&sta_config));
    sta_config.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    sta_config.threshold.rssi = -127;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, (wifi_config_t*)&sta_config));
}


static esp_err_t http_get_handler(httpd_req_t *req) {
    const char* resp_str = "<!DOCTYPE html><html><body>"
                           "<h1>ESP32 Configuration</h1>"
                           "<form action=\"/submit\" method=\"post\">"
                           "SSID: <input type=\"text\" name=\"ssid\"><br>"
                           "Password: <input type=\"text\" name=\"password\"><br>"
                           "API Key ThingSpeak: <input type=\"text\" name=\"ApiKey\"><br>"
                           "<input type=\"submit\" value=\"Submit\">"
                           "</form>"
                           "</body></html>";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_post_handler(httpd_req_t *req) {
    char buf[120];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        remaining -= ret;
    }

    buf[req->content_len] = '\0';
    ESP_LOGI(TAG_HTTP, "Received data: %s", buf);

    // Parse the received data
    sscanf(buf, "ssid=%31[^&]&password=%63[^&]&ApiKey=%16s", local_ssid, local_password, local_api_key);

    ESP_LOGI(TAG_HTTP, "Parsed SSID: %s", local_ssid);
    ESP_LOGI(TAG_HTTP, "Parsed Password: %s", local_password);
    ESP_LOGI(TAG_HTTP, "Parsed API Key: %s", local_api_key);

    // Save the received SSID, password, and API key to NVS or file system as needed

    const char* resp_str = "Configuration saved";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    current_operation = READING;
    return ESP_OK;
}

static esp_err_t http_connectivity_handler(httpd_req_t *req) {
    const char* resp_str = "HTTP/1.1 204 No Content\r\nContent-Length: 0\r\n\r\n";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_uri_t uri_get = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = http_get_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_post = {
    .uri      = "/submit",
    .method   = HTTP_POST,
    .handler  = http_post_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_connectivity_check = {
    .uri      = "/generate_204",
    .method   = HTTP_GET,
    .handler  = http_connectivity_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_connectivity_check2 = {
    .uri      = "/hotspot-detect.html",
    .method   = HTTP_GET,
    .handler  = http_connectivity_handler,
    .user_ctx = NULL
};

void start_http_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_post);
        httpd_register_uri_handler(server, &uri_connectivity_check);
        httpd_register_uri_handler(server, &uri_connectivity_check2);
    }
}

// LittleFS functions
void init_littlefs() {
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/storage",
        .partition_label = "storage",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    
    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG_FILE, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG_FILE, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG_FILE, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_FILE, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG_FILE, "Partition size: total: %d, used: %d", total, used);
    }
}

bool write_uint8_array_to_file(const char* filename, const uint8_t* data, size_t length) {
    //ESP_LOGI(TAG, "Writing uint8_t array to file: %s", filename);
    FILE* f = fopen(filename, "wb");  // Note the "wb" for writing in binary mode
    if (f == NULL) {
        ESP_LOGE(TAG_FILE, "Failed to open file for writing");
        return false;
    }
    size_t written = fwrite(data, sizeof(uint8_t), length, f);
    if (written != length) {
        ESP_LOGE(TAG_FILE, "Failed to write to file: wrote %d of %d bytes", written, length);
        fclose(f);
        return false;
    }
    fclose(f);
    return true;
}

bool append_uint8_array_to_file(const char* filename, const uint8_t* data, size_t length) {
    //ESP_LOGI(TAG, "Writing uint8_t array to file: %s", filename);
    FILE* f = fopen(filename, "ab");  // Note the "wb" for writing in binary mode
    if (f == NULL) {
        ESP_LOGE(TAG_FILE, "Failed to open file for appending");
        f = fopen(filename, "wb");
        if (f == NULL) {
            ESP_LOGE(TAG_FILE, "Failed to create file");
            return false;
        }
    }
    
    size_t written = fwrite(data, sizeof(uint8_t), length, f);
    if (written != length) {
        ESP_LOGE(TAG_FILE, "Failed to write to file: wrote %d of %d bytes", written, length);
        fclose(f);
        return false;
    }

    fclose(f);
    return true;
}

std::vector<uint8_t> read_from_file_as_uint8_vector(const char* filename) {
    //ESP_LOGI(TAG, "Reading uint8_t array from file: %s", filename);
    FILE* f = fopen(filename, "rb");  // Note the "rb" for reading in binary mode
    if (f == NULL) {
        ESP_LOGE(TAG_FILE, "Failed to open file for reading");
        return std::vector<uint8_t>();
    }
    
    // Get file size
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    // Read file contents into vector
    std::vector<uint8_t> data(fsize);
    size_t read = fread(data.data(), sizeof(uint8_t), fsize, f);
    if (read != fsize) {
        ESP_LOGE(TAG_FILE, "Failed to read from file: read %d of %d bytes", read, (int)fsize);
    }
    fclose(f);
    
    return data;
}

// Function configure the GPIO as output or input
void config_gpio(gpio_num_t gpio_num, gpio_mode_t mode) {
    // Configure the GPIO as output
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;           // Disable interrupt
    io_conf.mode = mode;                             // Set mode
    io_conf.pin_bit_mask = (1ULL << gpio_num);       // Bit mask of the pin
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;    // Disable pull-down mode
    if(gpio_num != LED_PIN){
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;        // Disable pull-up mode
    }
    else{
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;        // Enable pull-up mode
    }   
    gpio_config(&io_conf);                           // Configure GPIO with the given settings
}

// Temperature sensor related functions
void init_temp_sens(temperature_sensor_handle_t* ptemp_handle)
{
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-30, 40);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, ptemp_handle));
}

float get_temperature(temperature_sensor_handle_t* ptemp_handle)
{
    float local_temperature = 0.0;
    float old_local_temperature = 999.0;
    // Enable temperature sensor
    ESP_ERROR_CHECK(temperature_sensor_enable(*ptemp_handle));
    // Get converted sensor data
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(*ptemp_handle, &local_temperature));

    while(abs(local_temperature - old_local_temperature) > 0.1){
        old_local_temperature = local_temperature;
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(*ptemp_handle, &local_temperature));
    }
    // Disable the temperature sensor if it is not needed and save the power
    ESP_ERROR_CHECK(temperature_sensor_disable(*ptemp_handle));
    return local_temperature;
}

esp_err_t i2c_master_init(i2c_master_bus_handle_t *i2c_bus)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = -1,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .flags = {
            .enable_internal_pullup = true
        }
    };
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, i2c_bus);
    return err;
}

esp_err_t i2c_add_HTU21D_sensor(i2c_master_bus_handle_t i2c_bus, i2c_master_dev_handle_t *i2c_dev){
    i2c_device_config_t i2c_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x40,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 1000,
        .flags = {
            .disable_ack_check = false
        }
    };

    esp_err_t err = i2c_master_bus_add_device(i2c_bus, &i2c_dev_config, i2c_dev);
    return err;
}

// ADC related functions
void init_adc(adc_oneshot_unit_handle_t *padc_handle){
    adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, padc_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    adc_oneshot_chan_cfg_t humidity_config = {
        .atten = HUMIDIDY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(*padc_handle, BATTERY_ADC_CH, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*padc_handle, HUMIDIDY_ADC_CH, &humidity_config));
    //For calibration check https://github.com/espressif/esp-idf/blob/v5.3/examples/peripherals/adc/oneshot_read/main/oneshot_read_main.c
}

bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    if (!calibrated) {
        ESP_LOGI(ADCTAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(ADCTAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(ADCTAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(ADCTAG, "Invalid arg or no memory");
    }
    return calibrated;
}

void adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_LOGI(ADCTAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
}

float get_humidity_from_HTU21D(i2c_master_dev_handle_t i2c_dev){
    uint8_t cmd = 0xE5;  // Command for temperature measurement
    uint8_t data[3];     // Buffer for received data (2 bytes temp + 1 byte CRC)
    esp_err_t err;
    uint16_t raw_data = 0;
    float scaled_data;

    err = i2c_master_transmit(i2c_dev, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(IIC_TAG, "Failed to send command: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(20));  // Wait for measurement

    err = i2c_master_receive(i2c_dev, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(IIC_TAG, "Failed to read data: %s", esp_err_to_name(err));
    }

    raw_data = (data[0] << 8) | data[1];
    scaled_data = -6.0 + (125.0 * raw_data / 65536.0);
    return scaled_data;
}


float get_temperature_from_HTU21D(i2c_master_dev_handle_t i2c_dev){
    uint8_t cmd = 0xE3;  // Command for temperature measurement
    uint8_t data[3];     // Buffer for received data (2 bytes temp + 1 byte CRC)
    esp_err_t err;
    uint16_t raw_data = 0;
    float scaled_data;

    err = i2c_master_transmit(i2c_dev, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(IIC_TAG, "Failed to send command: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // Wait for measurement

    err = i2c_master_receive(i2c_dev, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(IIC_TAG, "Failed to read data: %s", esp_err_to_name(err));
    }

    raw_data = (data[0] << 8) | data[1];
    scaled_data = -46.85 + (175.72 * raw_data / 65536.0);
    return scaled_data;
}

void soft_reset_HTU21D(i2c_master_dev_handle_t i2c_dev){
    uint8_t cmd = 0xFE;  // Command for temperature measurement
    esp_err_t err;
    
    err = i2c_master_transmit(i2c_dev, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(IIC_TAG, "Failed to send command: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // Wait for measurement
}

void send_to_thingspeak(sensors_data *data) {
    char url[128];
    snprintf(url, sizeof(url),
        "http://api.thingspeak.com/update?"
        "api_key=%s"
        "&field1=%d"
        "&field2=%.1f",
        local_api_key,
        data->soil_humidity_percentage,
        data->internal_temperature_x10 / 10.0f);

    ESP_LOGI(TAG_HTTP, "Sending data to ThingSpeak: %s", url);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_HTTP, "HTTP GET request sent successfully");
    } else {
        ESP_LOGE(TAG_HTTP, "HTTP GET request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}