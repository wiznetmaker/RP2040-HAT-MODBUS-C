#ifndef _MBCONFIG_H_
#define _MBCONFIG_H_

#define MODBUS_RTU   0
#define MODBUS_ASCII 1

#define MODBUS_PROTOCOL MODBUS_RTU

#if !defined(MAX)
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#if !defined(MIN)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

enum {
    FALSE = 0,
    TRUE
};

#endif