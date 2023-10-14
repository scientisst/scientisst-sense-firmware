file(GLOB_RECURSE COMMUNICATION_SOURCES "communication/*.c")
file(GLOB_RECURSE SENSORS_SOURCES "sensors/*.c")
file(GLOB_RECURSE DRIVERS_SOURCES "drivers/*.c")
file(GLOB_RECURSE TASKS_SOURCES "tasks/*.c")
set(srcs "sci_scientisst.c" ${COMMUNICATION_SOURCES} ${DRIVERS_SOURCES} ${TASKS_SOURCES} ${SENSORS_SOURCES})

set(requires esp_adc_cal nvs_flash json mdns bt esp_http_server fatfs spiffs esp_https_server driver sdmmc)

set(SRCS_PATH "scientisst-sense-firmware/main")

#Add main.c if scientisst is intended to be a standalone firmware
if (STANDALONE_SCIENTISST)
    list(APPEND srcs "main.c")
else ()
    list(TRANSFORM srcs PREPEND "${SRCS_PATH}/")
endif ()

idf_component_register(SRCS ${srcs}
        INCLUDE_DIRS "." ".." "communication/include" "communication/wifi_rest_server" "drivers/include" "sensors/include" "tasks/include"
        REQUIRES ${requires}
        EMBED_TXTFILES "${CMAKE_CURRENT_LIST_DIR}/certs/cacert.pem" "${CMAKE_CURRENT_LIST_DIR}/certs/prvtkey.pem"
)


set(WEB_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../www")
if (EXISTS ${WEB_SRC_DIR})
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT)
else ()
    message(FATAL_ERROR "${WEB_SRC_DIR} doesn't exit")
endif ()

add_compile_options(-Wall -Wextra -Wundef -Wunused -Warray-bounds -Wbuiltin-macro-redefined)
