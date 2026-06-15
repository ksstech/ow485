// platform-ow485.h — central build configuration and hardware pin map
// Copyright (c) 2026 Andre M. Maree / KSS Technologies (Pty) Ltd.

/*	Phy	    Num	    Log
 *	1	    --	    ---   --    Vcc
 *  2	    0	    PA4   X0    1Wire     In/Out  Waveform Output 4 (WO4) on HCMP1
 *  3	    1	    PA5   X1    LED       Out     Waveform Output 5 (WO5) on HCMP2
 *  4	    2	    PA6   X2    Unused
 *  5       3	    PA7   X3    Unused
 *  6   		    PB3   D5    RS485_RX  In
 *  7	    	    PB2   D4    RS485_TX  Out
 *  8		        PB1   X7    Unused		       RESET
 *  9		        PB0   X6    RS485_EN  Out
 *  10	    11	    PA0   X11   UPDI      In/Out
 *  11	    8	    PA1   X8    Unused
 *  12	    9	    PA2   X9    Relay     Out
 *  13	    10	    PA3   X10   Unused            Waveform Output 3 (WO3) on HCMP0
 *  14      --      ---   ---   GND
 */
 
#pragma once

#define _toSTR(x)			#x
#define toSTR(x)			_toSTR(x)

#define DEV_MFGR            "KSS"
#define DEV_MODEL           "OW485"
#define DEV_MCU             "ATtiny3224"
#define DEV_HW_INFO         DEV_MFGR "_" DEV_MODEL "_" DEV_MCU

#define DEV_FW_NAME         "Irmacos"
#define DEV_FW_MAJ          0
#define DEV_FW_MIN          5
#define DEV_FW_FIX          0
#define DEV_FW_INFO         DEV_FW_NAME " v" toSTR(DEV_FW_MAJ) "." toSTR(DEV_FW_MIN) "." toSTR(DEV_FW_FIX)

// Firmware BUILD options.
#define OW_DS18TEMP         1       // include DS18xx temperature sensor support

// 1-Wire timing
#define OW_SCAN_MS          500UL   // iButton bus scan interval
#define OW_DS18_INTERVAL_MS 5000UL  // DS18xx conversion interval (temp changes slowly)

// Debug BUILD options
#define DEBUG_LEVEL         0x00FF    // combination of flags below
#define DEBUG_TRACK         0x0001
#define DEBUG_CMDBUF        0x0002
#define DEBUG_1WIRE         0x0004
#define DEBUG_PWM           0x0008
#define DEBUG_ADDRESS       0x0010
#define DEBUG_CMD_ECHO      0x0020  // character echo + RS485 flush (ow485-comms.h)
#define DEBUG_CMD_EDITING   0x0040  // VT100 line editing  (requires DEBUG_CMD_ECHO)
#define DEBUG_CMD_HISTORY   0x0080  // 8-deep history ring (requires DEBUG_CMD_EDITING)

// UART Buffer sizes
//#undef SERIAL_TX_BUFFER_SIZE
//#undef SERIAL_RX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 32
#define SERIAL_RX_BUFFER_SIZE 32

// USERROW usage
#define UR_IDX_DEVID 0  // first byte in the array

// Pin definitions
const uint8_t pinUPDI  = PIN_PA0;
// Unused pins: PA1
const uint8_t pinRelay = PIN_PA2;
// Unused pins: PA3
const uint8_t pin1Wire = PIN_PA4;
const uint8_t pinLED   = PIN_PA5;
// Unused pins: PA6, PA7
const uint8_t pin485EN = PIN_PB0;
// Unused pins: PB1 (RESET)
const uint8_t pin485TX = PIN_PB2;
const uint8_t pin485RX = PIN_PB3;

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-06-12
 *   - New file: pin definitions extracted from ow485.cpp
 *
 * 2026-06-12
 *   - Device identity macros: DEV_MFGR / DEV_MODEL / DEV_MCU / DEV_FW_*
 *   - toSTR() stringify helper
 *   - Debug flag definitions (DEBUG_LEVEL, DEBUG_TRACK … DEBUG_CMD_HISTORY)
 *     moved here from ow485.cpp
 *
 * 2026-06-15
 *   - Promoted to sole central config header
 *   - OW_DS18TEMP feature flag (conditional DS18xx support)
 *   - OW_SCAN_MS / OW_DS18_INTERVAL_MS timing constants
 *   - UART buffer sizes and UR_IDX_DEVID moved here from ow485.cpp
 */
