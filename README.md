# RP2040-HAT-MODBUS-C
This project is a Modbus TCP RTU/ASCII example code that runs on the W5500-EVB-Pico or W5100S-EVB-Pico.

https://docs.wiznet.io/Product/iEthernet/W5100S/w5100s-evb-pico
https://docs.wiznet.io/Product/iEthernet/W5500/w5500-evb-pico

## Set the CMakeLists
Set the PICO_SDK_PATH in RP2040-HAT-MODBUS-C/CMakeLists.txt   
Refer to the documentation below for more information.  
https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html
```cmake
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
    message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
endif()
```

If you are using the W5100S-EVB-Pico board, make the following changes. The default is W5500-EVB-Pico.

```cmake
# Set ethernet chip
set(WIZNET_CHIP W5100S)
```

## Set Timer
If you have a different MCU or a different clock, you will need to set the Timer. This example is set to a timer of 20khz.  
RP2040-HAT-MODBUS-C\port\modbus\src\mbtimer.c

1/20khz = 50us

```cpp
add_repeating_timer_us(50, vMBPortTimersCallback, NULL, &g_mb_timer);
```

## Set Modbus Serial
In this example, the UART1 TX : 4, RX : 5 Pins are converted to Modbus Serial.  
Baudrate is 19200.
RP2040-HAT-MODBUS-C\port\modbus\inc\mbserial.h
```cpp
#define UART_MODBUS uart1
#define UART_MODBUS_TX 4
#define UART_MODBUS_RX 5
#define UART_MODBUS_BAUDRATE 19200
```

## Build
RP2040-HAT-MODBUS-C
```terminal
mkdir build
cd build
cmake -G "NMake Makefiles" ..
nmake
```

I tested using modbus poll/slave tools.  
modbus poll : https://www.modbustools.com/modbus_poll.html  
modbus slave : https://www.modbustools.com/modbus_slave.html

Modbus Pool Connect :  
![Modbus Poll connect.png](https://github.com/wiznetmaker/W5X00_STM32F411_Modbus/blob/main/Images/Modbus%20Poll%20connect.png)  

Modbus Slave Connect :  
![Modbus Slave connect.png](https://github.com/wiznetmaker/W5X00_STM32F411_Modbus/blob/main/Images/Modbus%20Slave%20connect.png)  

Test Result :  
![Test Result.png](https://github.com/wiznetmaker/W5X00_STM32F411_Modbus/blob/main/Images/Test%20Result.png)  