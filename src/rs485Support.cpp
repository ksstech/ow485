// rs485Support.cpp

#include <Arduino.h>
#include <megaTinyCore.h>
#include <EEPROM.h>
#include <USERSIG.h>

#include "platform-ow485.h"
#include "OneWireTag.h"
#include "rs485Support.h"

// Global variables
uint32_t RunTime;
uint8_t UPhr, UPmin, UPsec;
uint16_t UPdays, UPmsec;

extern uint8_t DevID;
extern String CmdBuf;

// External variables
extern int __heap_start, *__brkval;
extern OneWireTag owTag;

extern void printChannelF(uint8_t ch, void(*handler)(const char*, ...));
extern void printAllF(void(*handler)(const char*, ...));

char mapIndexToChar(int8_t Idx) { return ((Idx % 8) == 0) ? ' ' : ((Idx % 4) == 0) ? '|' : ((Idx % 2) == 0) ? '-' : ':'; }

auto readEEPROM  = [](int addr) -> uint8_t { return EEPROM.read(addr);  };

auto readUSERSIG = [](int addr) -> uint8_t { return USERSIG.read(addr); };

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

static int serialVPrintF(const char * fmt, va_list vaList) {
  char MsgBuf[PRINT_BUFSIZE];
  int iRV = vsnprintf(MsgBuf, PRINT_BUFSIZE, fmt, vaList);
  serialWrite(MsgBuf);
  return iRV;
}

static int serialPrintF(const char * fmt, ...) {
  va_list vaList;
  va_start(vaList, fmt);
  int iRV = serialVPrintF(fmt, vaList);
  va_end(vaList);
  return iRV;
}

static int serialPrintArray(const char * Heading, uint8_t (*Func)(int), int16_t Len) {
  int iRV = serialPrintF(Heading);
  for (int16_t Idx = 0; Idx < Len; ++Idx) {
    if ((Idx & 0x0F) == 0)
      iRV += serialPrintF("\n0x%02x: ", Idx);
    iRV += serialPrintF("%02X%c", Func(Idx), mapIndexToChar(Idx));
  }
  iRV += serialPrintF("\n");
  return iRV;
}

void serialPrintFOptions(uint16_t Options, const char * fmt, ...) {
  rs485Status(true);
  if (Options == 0)                 goto optionsDone;
  if (Options & PO_ONEWIRE)         owTag.printRomInfo(1);
  if (Options & PO_RLY_LED)         printAllF(serialPrintF);
  if (Options & PO_FIRMWARE)        serialPrintF("%s %dMHz\n", (DEV_HW_INFO " / " DEV_FW_INFO " / CLK="), F_CPU/1000000);
  if (Options & PO_CMDBUF)          serialPrintF("'%s' L=%d\n", CmdBuf.c_str(), CmdBuf.length());
  if (Options & PO_SYSSTAT) {
    int Mem = (int)(&Mem) - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
    serialPrintF("Vcc=%umV  Temp=%u°C  Free=%uB\n", readSupplyVoltage(), readTemp() - 273, Mem);
  }
  if (Options & PO_EEPROM)          serialPrintArray("EEPROM", readEEPROM, 256);
  if (Options & PO_USERROW)         serialPrintArray("USERROW", readUSERSIG, 32);
  if (Options & PO_RUNTIME)         serialPrintF("%lu ", millis());
  if (Options & PO_UPTIME) {
    updateUpTime();
    serialPrintF("(%lu) %ud %02uh%02um%02u.%03u ", RunTime, UPdays, UPhr, UPmin, UPsec, UPmsec);
  }
  if (Options & PO_ADDR)            serialPrintF("%02x\n", DevID);
optionsDone:
  if (fmt == NULL) goto messageDone;
  va_list vaList;
  va_start(vaList, fmt);
  serialVPrintF(fmt, vaList);
  va_end(vaList);
messageDone:
  rs485Status(false);
}
