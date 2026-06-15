// rs485Support.h — RS485 direction control, serial output and debug formatter
// Copyright (c) 2026 Andre M. Maree / KSS Technologies (Pty) Ltd.

#pragma once

// format content control options
#define PO_EEPROM                 	0x0001
#define PO_USERROW                	0x0002
#define PO_CMDBUF                 	0x0004
#define PO_UPTIME                 	0x0008
#define PO_RLY_LED                  0x0010
#define PO_SYSSTAT                	0x0020
#define PO_FIRMWARE               	0x0040
#define PO_1W_BUTTON                0x0080
#define PO_1W_DS1820                0x0100

#define PRINT_BUFSIZE				128

void rs485Setup(void);

void rs485Status(bool);

void serialWrite(const char *);

void serialPrintRomID(uint8_t *, bool);

void serialPrintF(const char * fmt, ...);

void serialPrintFOptions(uint16_t, const char *, ...);

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-05-18
 *   - New file: RS485 direction control, serialWrite, serialPrintFOptions
 *     with PO_xxx option flags
 *
 * 2026-06-12
 *   - PO_1W_BUTTON added for iButton ROM + status output
 *
 * 2026-06-15
 *   - PO_1W_DS1820 added for DS18xx ROM + temperature output
 *   - serialPrintRomID() declared (shared ROM formatter for iButton and DS18xx)
 */
