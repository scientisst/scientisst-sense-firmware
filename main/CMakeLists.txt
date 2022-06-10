idf_component_register(SRCS "main.c" "bt.c" "timer.c" "adc.c" "gpio.c" "com.c" "i2c.c" "spi.c" "wifi.c" "wifi_rest_server.c" "tcp.c" "udp.c" "ws.c" "uart.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "certs/cacert.pem"
                                   "certs/prvtkey.pem")

set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../www")
if(EXISTS ${WEB_SRC_DIR})
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT)
else()
    message(FATAL_ERROR "${WEB_SRC_DIR} doesn't exit")
endif()
