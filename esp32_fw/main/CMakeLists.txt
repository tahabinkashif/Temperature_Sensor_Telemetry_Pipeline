idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_uart
        esp_driver_gpio
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        mqtt
)

