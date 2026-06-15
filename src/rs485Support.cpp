// rs485Support.cpp — RS485 direction control, serial output and debug formatter
// Copyright (c) 2026 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include <Arduino.h>

#include <megaTinyCore.h>
#include <EEPROM.h>
#include <USERSIG.h>

#include "platform-ow485.h"

#include "OneWireBus.h"
#include "OneWireTag.h"
#if OW_DS18TEMP
#include "DS18Temp.h"
#endif
#include "rs485Support.h"

// Global variables
uint32_t RunTime;
uint8_t UPhr, UPmin, UPsec;
uint16_t UPdays, UPmsec;

extern uint8_t DevID;
extern const char* hostCmdStr(void);
extern uint8_t     hostCmdLen(void);

// External variables
extern int __heap_start, *__brkval;

/**
 * Basic RS485 support functions
 */
void rs485Setup(void) {
  pinMode(pin485EN, OUTPUT);
	digitalWrite(pin485EN, HIGH);
	Serial.begin(19200);
	Serial.flush();
	digitalWrite(pin485EN, LOW);
}

void rs485Status(bool On) {
  if (On == 0) Serial.flush();
  digitalWrite(pin485EN, On ? HIGH : LOW);
}

static void updateUpTime(void) {
  uint32_t T = RunTime = millis();
  UPmsec = T % 1000UL;
  T /= 1000;
  UPsec = T % 60UL;
  T /= 60;
  UPmin = T % 60UL;
  T /= 60;
  UPhr = T % 24UL;
  T /= 24;
  UPdays = T;
}

/**
 * For now just a wrapper, make provision for redirecting to UPDI in future
 */
void serialWrite(const char * pMsg) { Serial.write(pMsg); }

void serialVPrintF(const char * fmt, va_list vaList) {
  char MsgBuf[PRINT_BUFSIZE];
  vsnprintf(MsgBuf, PRINT_BUFSIZE, fmt, vaList);
  serialWrite(MsgBuf);
}

void serialPrintF(const char * fmt, ...) {
  va_list vaList;
  va_start(vaList, fmt);
  serialVPrintF(fmt, vaList);
  va_end(vaList);
}

/**
 * Support for debugging of EEPROM and USERROW contents.
 */
auto readEEPROM  = [](int addr) -> uint8_t { return EEPROM.read(addr);  };

auto readUSERSIG = [](int addr) -> uint8_t { return USERSIG.read(addr); };

static char mapIndexToChar(int8_t Idx) { return ((Idx % 8) == 0) ? ' ' : ((Idx % 4) == 0) ? '|' : ((Idx % 2) == 0) ? '-' : ':'; }

static void serialPrintArray(const char * Heading, uint8_t (*Func)(int), int16_t Len) {
  serialPrintF(Heading);
  for (int16_t Idx = 0; Idx < Len; ++Idx) {
    if ((Idx & 0x0F) == 0)
      serialPrintF("\n0x%02x: ", Idx);
    serialPrintF("%02X%c", Func(Idx), mapIndexToChar(Idx));
  }
  serialPrintF("\n");
}

/**
 * OneWire (iButton & Thermometer) support
 */
extern OneWireTag owTag;
extern void printChannelF(uint8_t ch, void(*handler)(const char*, ...));
extern void printAllF(void(*handler)(const char*, ...));

void serialPrintRomID(uint8_t * pRomID, bool reverse) {
    Serial.printf("%02x-", pRomID[0]);
    for (uint8_t i = 1; i < 7; ++i)
        Serial.printf("%02x", reverse ? pRomID[7-i] : pRomID[i]);
    Serial.printf("-%02x", pRomID[7]);
}

static void serialPrintTagStatus(void) {
    const char* pMsg;
    switch (owTag.result) {
        case OWResult::OK:           pMsg = "OK";           break;
        case OWResult::NO_PRESENCE:  pMsg = "NO_PRESENCE";  break;
        case OWResult::CRC_ERROR:    pMsg = "CRC_ERROR";    break;
        case OWResult::WRONG_FAMILY: pMsg = "WRONG_FAMILY"; break;
        case OWResult::VERIFY_FAIL:  pMsg = "VERIFY_FAIL";  break;
        case OWResult::BUS_ERROR:    pMsg = "BUS_ERROR";    break;
        default:                     pMsg = "UNKNOWN";      break;
    }
    Serial.printf(" (%d) %s", (int)owTag.result, pMsg);
}

#if OW_DS18TEMP
extern DS18Temp owTemp;
#endif

/**
 * Multi-stage DEBUG output formatter.
 */
#if DEBUG_LEVEL
void serialPrintFOptions(uint16_t Options, const char * fmt, ...) {
  rs485Status(true);
  if (Options == 0)                 goto optionsDone;
  if (Options & PO_EEPROM)          serialPrintArray("EEPROM", readEEPROM, 256);
  if (Options & PO_USERROW)         serialPrintArray("USERROW", readUSERSIG, 32);
  if (Options & PO_CMDBUF)          serialPrintF("'%s' L=%d\n", hostCmdStr(), (int)hostCmdLen());
  if (Options & PO_UPTIME) {
    updateUpTime();
    serialPrintF("(%lu) %ud %02uh%02um%02u.%03u\n", RunTime, UPdays, UPhr, UPmin, UPsec, UPmsec);
  }
  if (Options & PO_RLY_LED)         printAllF(serialPrintF);
  if (Options & PO_SYSSTAT) {
    int Mem = (int)(&Mem) - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
    serialPrintF("Vcc=%umV  Temp=%u°C  Free=%uB\n", readSupplyVoltage(), readTemp() - 273, Mem);
  }
  if (Options & PO_1W_BUTTON) {
    serialPrintRomID(owTag.rom, true);
    serialPrintTagStatus();
    serialPrintF("\n");
  }
#if OW_DS18TEMP
  if (Options & PO_1W_DS1820) {
    //serialPrintF("%s ", DS18Temp::modelName(owTemp.model));
    serialPrintRomID(owTemp.rom, true);
    serialPrintF(" %.2fC %dbit\n", owTemp.tempC, owTemp.getResolution());
  }
#endif
  if (Options & PO_FIRMWARE)        serialPrintF("%s %dMHz\n", (DEV_HW_INFO " / " DEV_FW_INFO " / CLK="), F_CPU/1000000);
optionsDone:
  if (fmt == NULL) goto messageDone;
  va_list vaList;
  va_start(vaList, fmt);
  serialVPrintF(fmt, vaList);
  va_end(vaList);
messageDone:
  rs485Status(false);
}
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-05-18
 *   - New file: rs485Setup/Status, serialWrite, serialVPrintF, serialPrintF,
 *     serialPrintFOptions with PO_xxx flags; uptime tracking;
 *     EEPROM/USERROW dump helpers
 *
 * 2026-06-12
 *   - serialPrintTagStatus() added
 *   - PO_1W_BUTTON handling in serialPrintFOptions
 *
 * 2026-06-15
 *   - serialPrintRomID() added (reverse-order ROM formatter shared by iButton
 *     and DS18xx)
 *   - PO_1W_DS1820 block in serialPrintFOptions
 *   - OW_DS18TEMP guards on DS18Temp.h include and extern owTemp declaration
 */
