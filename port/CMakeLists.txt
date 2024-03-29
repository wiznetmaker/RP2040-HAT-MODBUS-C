# ioLibrary_Driver
add_library(IOLIBRARY_FILES STATIC)

target_sources(IOLIBRARY_FILES PUBLIC
        ${PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
        ${PORT_DIR}/ioLibrary_Driver/src/w5x00_gpio_irq.c
        )

target_include_directories(IOLIBRARY_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${PORT_DIR}/ioLibrary_Driver/inc
        ${PORT_DIR}
        )

target_link_libraries(IOLIBRARY_FILES PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_clocks
        )

# timer
add_library(TIMER_FILES STATIC)

target_sources(TIMER_FILES PUBLIC
        ${PORT_DIR}/timer/timer.c
        )

target_include_directories(TIMER_FILES PUBLIC
        ${PORT_DIR}/timer
        )

target_link_libraries(TIMER_FILES PRIVATE
        pico_stdlib      
        )


add_library(MODBUS_FILES STATIC)

target_sources(MODBUS_FILES PUBLIC
        ${PORT_DIR}/modbus/src/mb.c
        ${PORT_DIR}/modbus/src/mbascii.c
        ${PORT_DIR}/modbus/src/mbcrc.c
        ${PORT_DIR}/modbus/src/mbrtu.c
        ${PORT_DIR}/modbus/src/mbtcp.c
        ${PORT_DIR}/modbus/src/mbtimer.c
        ${PORT_DIR}/modbus/src/mbserial.c
        )

target_include_directories(MODBUS_FILES PUBLIC
        ${PORT_DIR}
        ${PORT_DIR}/modbus/inc
        )

target_link_libraries(MODBUS_FILES PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        IOLIBRARY_FILES
        )