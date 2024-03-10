
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "mbconfig.h"
#include "mbserial.h"
#include "mbrtu.h"
#include "mbascii.h"

BUFFER_DEFINITION(uart_rx, UART_RX_SIZE);

/*****************************************************************************
 * Private functions
 ****************************************************************************/

void xMBPortUARTInit(void)
{
    uart_deinit(UART_MODBUS);
    uart_init(UART_MODBUS, UART_MODBUS_BAUDRATE);

    gpio_set_function(UART_MODBUS_TX, GPIO_FUNC_UART);
    gpio_set_function(UART_MODBUS_RX, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_MODBUS, false, false);
    uart_set_format(UART_MODBUS, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_MODBUS, false);

    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(UART_MODBUS, true, false);
}

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */

void on_uart_rx(void)
{
    uint8_t ch;

    while (uart_is_readable(UART_MODBUS)) {
        ch = uart_getc(UART_MODBUS);

        if(is_uart_buffer_full() == FALSE)
            put_byte_to_uart_buffer(ch);
    }
}

int UART_read(void *data, int bytes)
{
  uint32_t i;
  uint8_t *data_ptr = data;
  if(IS_BUFFER_EMPTY(uart_rx)) return RET_NOK;
  
  for(i=0; i<bytes; i++)
    data_ptr[i] = (uint8_t)BUFFER_OUT(uart_rx);
  BUFFER_OUT_MOVE(uart_rx, i);
  return i;
}

uint32_t UART_write(void *data, int bytes)
{
  uint32_t i;
  uint8_t *data_ptr = data;

  for(i=0; i<bytes; i++)
    uart_putc(UART_MODBUS, data_ptr[i]);
  return bytes;
}

int32_t platform_uart_getc(void)
{
    int32_t ch;

    while(IS_BUFFER_EMPTY(uart_rx));
    ch = (int32_t)BUFFER_OUT(uart_rx);
    BUFFER_OUT_MOVE(uart_rx, 1);

    return ch;
}

int32_t platform_uart_getc_nonblk(void)
{
    int32_t ch;

    if(IS_BUFFER_EMPTY(uart_rx)) return RET_NOK;
    ch = (int32_t)BUFFER_OUT(uart_rx);
    BUFFER_OUT_MOVE(uart_rx, 1);

    return ch;
}

int32_t platform_uart_gets(uint8_t* buf, uint16_t bytes)
{
    uint16_t lentot = 0, len1st = 0;

    lentot = bytes = MIN(BUFFER_USED_SIZE(uart_rx), bytes);
    if(IS_BUFFER_OUT_SEPARATED(uart_rx) && (len1st = BUFFER_OUT_1ST_SIZE(uart_rx)) < bytes) {
        memcpy(buf, &BUFFER_OUT(uart_rx), len1st);
        BUFFER_OUT_MOVE(uart_rx, len1st);
        bytes -= len1st;
    }
    memcpy(buf+len1st, &BUFFER_OUT(uart_rx), bytes);
    BUFFER_OUT_MOVE(uart_rx, bytes);

    return lentot;
}

void uart_rx_flush(void)
{
    BUFFER_CLEAR(uart_rx);
}

void put_byte_to_uart_buffer(uint8_t ch)
{
    BUFFER_IN(uart_rx) = ch;
    BUFFER_IN_MOVE(uart_rx, 1);
}


uint16_t get_uart_buffer_usedsize(void)
{
    uint16_t len = 0;

    len = BUFFER_USED_SIZE(uart_rx);
    return len;
}

uint16_t get_uart_buffer_freesize(void)
{
    uint16_t len = 0;

    len = BUFFER_FREE_SIZE(uart_rx);
    return len;
}

int8_t is_uart_buffer_empty(void)
{
    int8_t ret = 0;

    ret = IS_BUFFER_EMPTY(uart_rx);
    return ret;
}

int8_t is_uart_buffer_full(void)
{
    int8_t ret = 0;

    ret = IS_BUFFER_FULL(uart_rx);
    return ret;
}

