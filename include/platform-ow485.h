// platform-ow485.h

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

#define _toSTR(x)				#x
#define toSTR(x)				_toSTR(x)

#define DEV_MFGR                "KSS"
#define DEV_MODEL               "OW485"
#define DEV_MCU                 "ATtiny3224"
#define DEV_HW_INFO             DEV_MFGR "_" DEV_MODEL "_" DEV_MCU

#define DEV_FW_NAME             "Irmacos"
#define DEV_FW_MAJ              0
#define DEV_FW_MIN              2
#define DEV_FW_FIX              0
#define DEV_FW_INFO             DEV_FW_NAME " v" toSTR(DEV_FW_MAJ) "." toSTR(DEV_FW_MIN) "." toSTR(DEV_FW_FIX)


// Global constants
const uint8_t pinUPDI = PIN_PA0;
const uint8_t pinRelay = PIN_PA2;
const uint8_t pinOneWire = PIN_PA4;
const uint8_t pinLED = PIN_PA5;

const uint8_t pin485EN = PIN_PB0;
const uint8_t pin485TX = PIN_PB2;
const uint8_t pin485RX = PIN_PB3;
