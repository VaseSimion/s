idf_component_register(SRCS "Support.cpp" "main.cpp" 
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_http_client esp_adc esp_littlefs nvs_flash esp_driver_tsens esp_driver_gpio esp_driver_i2c)