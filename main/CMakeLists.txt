set(srcs "scientisst.c" "bt.c" "timer.c" "adc.c" "gpio.c" "com.c" "spi.c" "wifi.c" "wifi_rest_server.c" "tcp.c" "udp.c" "uart.c" "ws.c" "ble.c" "sd_card.c")
set(requires esp_adc_cal nvs_flash json mdns bt esp_http_server fatfs spiffs esp_https_server)

set(SRCS_PATH "scientisst-sense-firmware/main")

#Add main.c if scientisst is intended to be a standalone firmware
if (STANDALONE_SCIENTISST)
    list(APPEND srcs "main.c")
else ()
    list(TRANSFORM srcs PREPEND "${SRCS_PATH}/")
endif ()

idf_component_register(SRCS ${srcs}
        INCLUDE_DIRS "." "../../"
        REQUIRES ${requires}
        EMBED_TXTFILES "${CMAKE_CURRENT_LIST_DIR}/certs/cacert.pem" "${CMAKE_CURRENT_LIST_DIR}/certs/prvtkey.pem"
        )


set(WEB_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../www")
if (EXISTS ${WEB_SRC_DIR})
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT)
else ()
    message(FATAL_ERROR "${WEB_SRC_DIR} doesn't exit")
endif ()
