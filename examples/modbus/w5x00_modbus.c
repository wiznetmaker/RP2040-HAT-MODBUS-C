/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include "port_common.h"

#include "wizchip_conf.h"
#include "socket.h"
#include "w5x00_spi.h"
#include "dhcp.h"

#include "timer.h"

#include "mb.h"
#include "mbrtu.h"
#include "mbascii.h"
#include "mbserial.h"
#include "mbconfig.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_MODBUS 0
#define SOCKET_DHCP 1

/* Port */
#define PORT_MODBUS 5000

/* Retry count */
#define DHCP_RETRY_COUNT 5

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 11, 2},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 11, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .dhcp = NETINFO_DHCP                         // DHCP enable/disable
};
static uint8_t g_ethernet_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
}; // common buffer

/* Timer */
static volatile uint16_t g_msec_cnt = 0;

/* Modbus */
extern volatile uint8_t mb_state_rtu_finish;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void);

/* DHCP */
static void wizchip_dhcp_init(void);
static void wizchip_dhcp_assign(void);
static void wizchip_dhcp_conflict(void);

/* Timer */
static void repeating_timer_callback(void);

/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */
int main()
{
    /* Initialize */
    uint8_t dhcp_retval = 0;
    uint8_t dhcp_retry = 0;
    uint16_t reg_val;
    uint8_t state;
    uint8_t destip[4];
    uint16_t destport;

    set_clock_khz();
    stdio_init_all();

    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    wizchip_1ms_timer_initialize(repeating_timer_callback);
    eMBRTUInit(UART_MODBUS_BAUDRATE);
    xMBPortUARTInit();

    if (g_net_info.dhcp == NETINFO_DHCP) // DHCP
        wizchip_dhcp_init();
    else // static
    {
        network_initialize(g_net_info);
        /* Get network information */
        print_network_information(g_net_info);
    }

    /* Infinite loop */
    while (1)
    {
        /* Assigned IP through DHCP */
        if (g_net_info.dhcp == NETINFO_DHCP)
        {
            dhcp_retval = DHCP_run();
            if (dhcp_retval == DHCP_FAILED)
            {
                dhcp_retry++;
                if (dhcp_retry <= DHCP_RETRY_COUNT)
                    printf(" DHCP timeout occurred and retry %d\n", dhcp_retry);
            }
            if (dhcp_retry > DHCP_RETRY_COUNT)
            {
                printf(" DHCP failed\n");
                DHCP_stop();
                while (1);
            }
            wizchip_delay_ms(1000); // wait for 1 second
        }

        if (dhcp_retval == DHCP_IP_LEASED)
        {
            state = getSn_SR(SOCKET_MODBUS);
            switch(state)
            {
                case SOCK_INIT:
                    //listen(sock); //Function call Immediately after socket open operation
                    break;
                
                case SOCK_LISTEN:
                    break;
                
                case SOCK_ESTABLISHED:
                    if(getSn_IR(SOCKET_MODBUS) & Sn_IR_CON)
                    {
                        reg_val = SIK_CONNECTED & 0x00FF;
                        ctlsocket(SOCKET_MODBUS, CS_CLR_INTERRUPT, (void *)&reg_val);
                        getsockopt(SOCKET_MODBUS, SO_DESTIP, &destip);
                        getsockopt(SOCKET_MODBUS, SO_DESTPORT, &destport);
                        printf("CONNECTED FROM - %d.%d.%d.%d : %d\r\n",destip[0], destip[1], destip[2], destip[3], destport);
                    }
                    
        #if (MODBUS_PROTOCOL == MODBUS_RTU)
                    RTU_Uart_RX();
                    if(mb_state_rtu_finish == TRUE){
                        mb_state_rtu_finish = FALSE;
                        mbRTUtoTCP(SOCKET_MODBUS);
                    }
                    if(getSn_RX_RSR(SOCKET_MODBUS))
                        mbTCPtoRTU(SOCKET_MODBUS);
        #else
                    ASCII_Uart_RX();
                    if(mb_state_ascii_finish == TRUE) {
                        mb_state_ascii_finish = FALSE;
                        mbASCIItoTCP(SOCKET_MODBUS);
                    }
                    if(getSn_RX_RSR(SOCKET_MODBUS))
                        mbTCPtoASCII(SOCKET_MODBUS);
        #endif
                    break;

                case SOCK_CLOSE_WAIT:
                    break;
                
                case SOCK_FIN_WAIT:
                case SOCK_CLOSED:
                    close(SOCKET_MODBUS);
                    if(socket(SOCKET_MODBUS, Sn_MR_TCP, PORT_MODBUS, (SF_TCP_NODELAY | SF_IO_NONBLOCK)) == SOCKET_MODBUS)
                    {                
                        // TCP Server listen
                        listen(SOCKET_MODBUS);
                        printf("TCP_SERVER_MODE:SOCKOPEN\r\n");
                    }
                    break;
                    
                default:
                    break;
            }
        }

    }
}

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

/* DHCP */
static void wizchip_dhcp_init(void)
{
    printf(" DHCP client running\n");

    DHCP_init(SOCKET_DHCP, g_ethernet_buf);

    reg_dhcp_cbfunc(wizchip_dhcp_assign, wizchip_dhcp_assign, wizchip_dhcp_conflict);
}

static void wizchip_dhcp_assign(void)
{
    getIPfromDHCP(g_net_info.ip);
    getGWfromDHCP(g_net_info.gw);
    getSNfromDHCP(g_net_info.sn);
    getDNSfromDHCP(g_net_info.dns);

    g_net_info.dhcp = NETINFO_DHCP;

    /* Network initialize */
    network_initialize(g_net_info); // apply from DHCP

    print_network_information(g_net_info);
    printf(" DHCP leased time : %ld seconds\n", getDHCPLeasetime());
}

static void wizchip_dhcp_conflict(void)
{
    printf(" Conflict IP from DHCP\n");

    // halt or reset or any...
    while (1)
        ; // this example is halt.
}

/* Timer */
static void repeating_timer_callback(void)
{
    g_msec_cnt++;

    if (g_msec_cnt >= 1000 - 1)
    {
        g_msec_cnt = 0;

        DHCP_time_handler();
        DNS_time_handler();
    }
}
