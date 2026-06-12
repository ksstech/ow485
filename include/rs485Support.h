// RS485 support

#pragma once

#include "platform-ow485.h"

// format content control options
#define PO_RUNTIME                 	0x0001
#define PO_UPTIME                 	0x0002
#define PO_ADDR                   	0x0004
#define PO_ONEWIRE                	0x0008
#define PO_RLY_LED                  0x0010
#define PO_EEPROM                 	0x0020
#define PO_USERROW                	0x0040
#define PO_FIRMWARE               	0x0080
#define PO_CMDBUF                 	0x0100
#define PO_SYSSTAT                	0x0200

#define PRINT_BUFSIZE				128

void rs485Setup(void);

void rs485Status(bool);

void serialWrite(const char *);

void serialPrintFOptions(uint16_t, const char *, ...);
