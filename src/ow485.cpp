/*
 * RS485 to OneWire - Copyright (c) 2026 Andre M. Maree / KSS Technologies (Pty) Ltd.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  MANDATORY IDE SETTINGS  (Tools menu)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Board                : ATtiny3224 / 3214 / 3204 (megaTinyCore)
 *  Chip                 : ATtiny3224
 *  Clock Source         : 10 MHz internal             ← REQUIRED at 3.3 V; see below
 *  millis()/micros()    : TCB0 or TCB1 (any TCB)      ← must NOT be TCA0
 *  Programmer           : SerialUPDI (HV) via PA0
 *
 *  CLOCK SPEED IS COUPLED TO VCC — tinyAVR 2-series datasheet DS40002315:
 *    0– 5 MHz  at 1.8–5.5 V  (Speed Grade 1)
 *    0–10 MHz  at 2.7–5.5 V  (Speed Grade 2)  ← this board at 3.3 V
 *    0–20 MHz  at 4.5–5.5 V  (Speed Grade 3)
 *
 *  At VCC = 3.3 V, 20 MHz is outside the guaranteed operating envelope.
 *  The chip may appear to run but delayMicroseconds() — a busy-loop scaled
 *  by F_CPU — can mis-time under marginal conditions.  For OneWire this
 *  directly stretches the 70 µs presence-sample delay past the end of the
 *  132 µs presence pulse, causing missed detections and CRC failures.
 *  Select 10 MHz; all timing constants (delayMicroseconds, millis, the
 *  OneWire library) scale automatically via F_CPU — no code changes needed.
 *  PWM frequency becomes: 10 MHz ÷ (64 × 256) ≈ 610 Hz — still adequate.
 *
 *  millis() timer: megaTinyCore 2.6.x defaults to TCB1 on ATtiny3224.
 *  TCB0 also works.  Either is fine.  The only forbidden choice is TCA0:
 *  that prevents split-mode PWM and corrupts OneWire inter-slot timing.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  TIMER ALLOCATION  (ATtiny3224, tinyAVR 2-series)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  TCB1  millis()/micros() — 1 ms periodic interrupt.  Configured by core.
 *        megaTinyCore 2.6.x default on ATtiny3224.  TCB0 also works if
 *        changed in the IDE; either choice leaves TCA0 free for PWM.
 *        TCB1 WO default is PA3; it is OUTPUT-COMPARE-disabled in periodic
 *        mode, so PA3 remains usable as LED GPIO / PWM output.
 *
 *  TCB0  Free.  Reserve for future input-capture, precise pulse timing, or
 *        hardware PWM.  Default WO on PA2.
 *
 *  TCA0  Split mode, fully dedicated to PWM via analogWrite().
 *        10 MHz ÷ (prescaler 64 × 256 counts) ≈ 610 Hz PWM frequency.
 *        Split-mode WO routing for 14-pin package (PB4/PB5 absent):
 *          PORTMUX.TCAROUTEA selects PA-group for WO3–WO5
 *          WO5 → PA5  (HCMP2 / LCMP2, channel 0 in this firmware — LED)
 *          WO3 → PA3  (HCMP0 / LCMP0, channel 2 if a second LED is fitted)
 *          WO4 → PA4  — NOT enabled (PA4 is the 1-Wire bus; enabling WO4
 *                        here would fight the 1-Wire driver).
 *                        Do NOT call analogWrite(PIN_PA4, …).
 *          WO2 → PA2  — NOT enabled (PA2 is the relay; active-HIGH digital).
 *                        Do NOT call analogWrite(PIN_PA2, …).
 *
 *  RTC   Unused.  Reserved for deep-sleep wakeup via 32 kHz internal osc.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  PIN ASSIGNMENT
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  PA0  UPDI      AdaFruit HV Friend SerialUPDI.  Permanently UPDI on
 *                 tinyAVR 2-series; never use as GPIO.
 *
 *  PA2  RELAY     Active-HIGH digital output.  TCA0 WO2 is also on PA2;
 *                 never call analogWrite(PIN_PA2, …).
 *                 OutputChannel ch1, digital mode (levelLow=0, levelHigh=1).
 *
 *  PA3  LED_AUX   Optional second status LED (hardware PWM WO3 → PA3).
 *                 If not fitted, leave configure(2, …) absent or call stop(2).
 *
 *  PA4  1-WIRE    DS1990 / RW1990 / DS18xx bus.  2 kΩ pullup to VCC.
 *
 *  PA5  LED       Primary status LED, hardware PWM WO5 → PA5.
 *                 OutputChannel ch0, PWM mode (levelLow=0, levelHigh=255).
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  REQUIRED LIBRARY  (install via Library Manager)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  No external libraries are required.  The OneWire bus driver is
 *  implemented directly using VPORT registers (see OneWireTag.h for
 *  the explanation of why Paul Stoffregen's OneWire library is
 *  incompatible with the tinyAVR 2-series VPORT register layout).
 *
 * https://onlinedocs.microchip.com/oxy/GUID-ACE76F82-8072-410B-AFA7-16B2BF7B4CBA-en-US-8/index.html
 *
 * https://www.electronics-lab.com/wp-content/uploads/2022/05/ATtiny-3224-pinout.jpg
 *
 * https://www.electronics-lab.com/attiny-development-boards-are-compatible-with-arduino-ide/
 *
 * Some thoughts:
 *  Limit number of slaves on bus
 *  Limit valid address range of slave devices
 *  Add command to provide config parameters to the client
 *  Allow OW GPIO reads to be interval scheduled and automatically reported to host
 *  Open command scope to specify a GPOut/Actuator bitmap (multiple devices) to switch more than one on or off
 *   0 1 2 3 4 5 6 7
 *   1 x 1 x x 1 0 x
 *  Similarly a command to specify a GPIn / binary sensor mask to read more than 1 device, same format as the actuator command
 *  Broadcast address 0xFF
 *  Start of open window for slaves to respond if needed
 *
 */

#include <Arduino.h>
#include <USERSIG.h>

#include "platform-ow485.h"
#include "rs485Support.h"
#include "ow485-comms.h"
#include "OutputChannel.h"
#include "OneWireBus.h"
#include "OneWireTag.h"
#if OW_DS18TEMP
#include "DS18Temp.h"
#endif

/* ── OutputChannel instance ──────────────────────────────────────────────────
 * Pin array is the single source of pin assignment for all output channels.
 * Channel 0 = PA5 (primary status LED, PWM)
 * Channel 1 = PA2 (relay, digital active-HIGH — setOutputMode called in setup())
 *
 * The array length N_CH is deduced by the template from the initialiser;
 * adding a third element automatically generates a 3-channel OutputChannel.
 */
static const uint8_t kOutPins[] = { pinLED, pinRelay };
OutputChannel<sizeof(kOutPins)> out(kOutPins);
void printChannelF(uint8_t ch, void(*handler)(const char*, ...)) { out.printChannelF(ch, handler); }
void printAllF(void(*handler)(const char*, ...)) { out.printAllF(handler); }

/* ── Peripheral objects ─────────────────────────────────────────────────── */
OneWireBus owBus(pin1Wire);
OneWireTag owTag(owBus);
#if OW_DS18TEMP
DS18Temp owTemp(owBus);
bool ds18Pending = false;
#endif

uint8_t DevID, owAddr;
uint32_t lastMsOneWire;

const char helpText[] =
  "All commands should be sent as a comma separated (CSV) ASCII string with NO spaces and ONLY an NL terminator\n"
  "Format of a command is XX,??[,xx..zz] where\n"
  " XX  2 byte hexadecimal address, possibly limited in range\n"
  " ??  2 byte alphanumeric command being one of the following\n"
  "     AP  Actuator command/config\n"
  "         ch,rr[,fi[,on[,fo[,of]]]]\n"
  "     CA  Config Address, store hex value as new device ID\n"
  "     CG  Config GPIO as in/out based on additional parameters\n"
  "     SA  System Address\n"
  "         YY being the requested device address in hex\n"
  "     SI  System Info request\n"
  "         { FW ver, MCU clock, MCU ID, Silicon Revision }\n"
  "     SR  System Reboot\n"
  "     OR  Read IButton data\n"
  "     OW  Write new data on Ibutton device\n"
  "         AB9876543210 being the address in hex to be written to the tag\n"
  "     RT  Relay On\n"
  "     RF  Relay Off\n"
  "     LT  LED On\n"
  "     LF  LED Off\n"
  " 10,rT  /  11,Ca,27  /  1D,or  / 18,OW,AB9876543210  /  15,ap,0,ffff,1000,2000,3000,4444\n ";

/* ══════════════════════════════════════════════════════════════════════════
 Output channel preset helpers  —  call from anywhere to change behaviour
 ══════════════════════════════════════════════════════════════════════════ */

void outHeartbeat(uint8_t Ch) { out.configure(Ch, OUT_INFINITE, 2, 1, 2, 1); }

void outError(uint8_t Ch) { out.configure(Ch, 3, 0, 1, 0, 1); }

void outAck(uint8_t Ch) { out.configure(Ch, 1, 1, 2, 1, 0); }

void outOn(uint8_t Ch) { out.configure(Ch, OUT_INFINITE, 0, OUT_INFINITE, 0, 0); }

void outOff(uint8_t Ch) { out.configure(Ch, OUT_STOP, 0, 0, 0, 0); }

void outAllOff(void) { out.stopAll(); }

void actuatePWM(void) {
  uint16_t para[6] = { 0 };
  int8_t Idx = 0;
  long int iVal;
  do {
    if (DEBUG_LEVEL & DEBUG_CMDBUF) serialPrintFOptions(PO_CMDBUF, NULL);
    iVal = hostConsumeHexValue(4);  // max 4 hex digits
    if (iVal == -1)                 // error ?
      break;                        // yes, break out
    para[Idx++] = (uint16_t)iVal;   // no, store value
  } while (Idx < 6);
  #if (DEBUG_LEVEL & DEBUG_PWM)
    serialPrintFOptions(0,"iVal=%lx Idx=%d 0=%x 1=%x 2=%x 3=%x 4=%x 5=%x", iVal, Idx,
                      para[0], para[1], para[2], para[3], para[4], para[5]);
  #endif
  if (Idx == 0)
    return;
  out.configure(para[0], para[1], para[2], para[3], para[4], para[5]);
}

void systemSetAddress(void) {
  long int iVal = hostConsumeHexValue(2);
  if (iVal != -1) {
    USERSIG.update(UR_IDX_DEVID, DevID = owAddr = (uint8_t)iVal);
    USERSIG.flush();
  } else {
    if (DEBUG_LEVEL & DEBUG_ADDRESS) serialPrintFOptions(0, "Invalid HEX address");
  }
}

void systemReset(void) {
  if (DEBUG_LEVEL & DEBUG_TRACK)      serialPrintFOptions(0, "Rebooting");
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8CLK_gc);  //enable the WDT, minimum timeout
  while (1);  // spin until reset
}

/* ══════════════════════════════════════════════════════════════════════════
   1-Wire demonstration — non-blocking poll via millis()
   ══════════════════════════════════════════════════════════════════════════ */

static void oneWireWriteTag(void) {
  const uint8_t serial[6] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB };
  OneWireTag::buildRomCode(serial, owTag.rom);
  owTag.program();
  if (DEBUG_LEVEL & DEBUG_1WIRE) serialPrintFOptions(PO_1W_BUTTON, "Write");
}

static void oneWireReadTag(void) {
  owTag.readRaw();
  if (DEBUG_LEVEL & DEBUG_1WIRE) serialPrintFOptions(PO_1W_BUTTON, "Read");
}

static void oneWireReadCheck(void) {
  if (millis() - lastMsOneWire < OW_SCAN_MS) return;
  lastMsOneWire = millis();

#if OW_DS18TEMP
  // Harvest a completed DS18 conversion — does NOT skip the iButton scan below.
  if (ds18Pending && owTemp.isReady()) {
    ds18Pending = false;
    owTemp.readTemp();
    if (DEBUG_LEVEL & DEBUG_1WIRE) serialPrintFOptions(PO_1W_DS1820, NULL);
  }
#endif

  // Dedicated scan instance — owTag.rom is untouched by the search so the last
  // valid iButton read persists until systemPoll() consumes and clears it.
  static OneWireTag  scanTag(owBus);
  static uint32_t    lastMsDS18 = 0;

  // SEARCH ROM pass 1: collect all ROM codes before taking any bus action.
  // Two passes prevent a bus reset (inside startConversion) from corrupting
  // an in-progress SEARCH ROM sequence when both device types are present.
  uint8_t roms[2][OW_ROM_BYTES];
  uint8_t nFound = 0;
  scanTag.resetSearch();
  while (nFound < 2 && scanTag.scanNext() == OWResult::OK)
    memcpy(roms[nFound++], scanTag.rom, OW_ROM_BYTES);

  // Pass 2: identify and handle each device
  for (uint8_t i = 0; i < nFound; i++) {
    if (OneWireTag::isCompatible(roms[i])) {
      // iButton (DS1990 / RW1990 — family 0x01): update owTag only on a new find
      memcpy(owTag.rom, roms[i], OW_ROM_BYTES);
      owTag.result      = OWResult::OK;
      owTag.timestampMs = millis();
      if (DEBUG_LEVEL & DEBUG_1WIRE) serialPrintFOptions(PO_1W_BUTTON, NULL);
    }
#if OW_DS18TEMP
    else {
      // DS18xx temperature sensor — hardwired; trigger conversion every 5 s
      memcpy(owTemp.rom, roms[i], OW_ROM_BYTES);
      owTemp.identify();
      if (owTemp.model != DS18Model::UNKNOWN && !ds18Pending &&
          millis() - lastMsDS18 >= OW_DS18_INTERVAL_MS) {
        if (owTemp.startConversion() == OWResult::OK) {
          ds18Pending = true;
          lastMsDS18  = millis();
        }
      }
    }
#endif
  }
}

/* ══════════════════════════════════════════════════════════════════════════
   setup()
   ══════════════════════════════════════════════════════════════════════════ */

void systemPoll(void) {
  bool bTagSent = false;
  rs485Status(true);
  if (owTag.rom[0]) {
    serialPrintRomID(owTag.rom, true);
    memset(owTag.rom, 0, OW_ROM_BYTES);   // Clear ROM buffers to make it obvious when new data is read
    owTag.result = OWResult::OK;
    bTagSent = true;
  }
#if OW_DS18TEMP
  bool bTempSent = false;
  if (owTemp.rom[0]) {
    if (bTagSent)             serialWrite(",");
    serialPrintRomID(owTemp.rom, true);
    serialPrintF(",%.2fC", owTemp.tempC);
    memset(owTemp.rom, 0, OW_ROM_BYTES);  // Clear ROM buffers to make it obvious when new data is read
    owTemp.result = OWResult::OK;
    bTempSent = true;
  }
  if (bTagSent || bTempSent)  serialWrite("\n");
#else
  if (bTagSent)               serialWrite("\n");
#endif
  rs485Status(false);
}

void setup(void) {
  rs485Setup();                             // Initialise RS485 serial comms
  DevID = USERSIG.read(UR_IDX_DEVID);       // Setup Device ID to be used
  if (DevID == 0xFF)                        // If no valid address set (yet)
    DevID = 0x10;                           // use default
#if DEBUG_LEVEL
  out.configure(0, OUT_INFINITE, 1, 1, 1, 1);
#else
  out.configure(0, 0, 0, 0, 0, 0);          // ch0 off (LED)
#endif
  out.setOutputMode(1, true, 0, 1);         // ch1 = relay, digital, active-HIGH
  out.configureDigital(1, OUT_STOP, 0, 0);  // relay off
  if (DEBUG_LEVEL & DEBUG_TRACK) serialPrintFOptions(PO_FIRMWARE|PO_SYSSTAT, "Started\n");
}

void loop(void) {
  out.update();       /* MUST be first: drive all output channel state machines */
  oneWireReadCheck(); /* 1-Wire tag scan */
  bool xCmd = hostReadCmd();
#if (DEBUG_LEVEL & DEBUG_CMD_ECHO)
  hostFlushEcho();
#endif
  if (xCmd) {
    if (DEBUG_LEVEL & DEBUG_CMDBUF)     serialPrintFOptions(PO_CMDBUF, NULL);
    long int iVal = hostConsumeHexValue(2);  // read 1 or 2 hex address bytes
    if (iVal != -1L) {                       // Value hex value found
      owAddr = iVal;
      if (owAddr == DevID) {
        // save 2 bytes as actual command, remove from buffer [incl comma]
        char Command[3];
        iVal = hostConsumeString(Command, sizeof(Command));
        if (DEBUG_LEVEL & DEBUG_CMDBUF)           serialPrintFOptions(PO_CMDBUF, Command);
        if (iVal == 0)                            systemPoll();
        else if (strncmp(Command, "AP", 2) == 0)  actuatePWM();
        else if (strncmp(Command, "OR", 2) == 0)  oneWireReadTag();
        else if (strncmp(Command, "OW", 2) == 0)  oneWireWriteTag();
        else if (strncmp(Command, "SA", 2) == 0)  systemSetAddress();
        else if (strncmp(Command, "SR", 2) == 0)  systemReset();
      #if DEBUG_LEVEL
        else if (strncmp(Command, "RT", 2) == 0)  outOn(1);
        else if (strncmp(Command, "RF", 2) == 0)  outOff(1);
        else if (strncmp(Command, "LT", 2) == 0)  outOn(0);
        else if (strncmp(Command, "LF", 2) == 0)  outOff(0);
        else if (strncmp(Command, "SI", 2) == 0)  serialPrintFOptions(PO_FIRMWARE|PO_UPTIME|PO_SYSSTAT|PO_RLY_LED|PO_1W_BUTTON|PO_USERROW|PO_EEPROM, NULL);
        else if (DEBUG_LEVEL & DEBUG_TRACK)       serialPrintFOptions(0, helpText);
      #endif
        else {
          // do nothing
        }
      } else {
        if (DEBUG_LEVEL & DEBUG_TRACK)            serialPrintFOptions(0, "Unknown Address\n");
      }
    } else {
      if (DEBUG_LEVEL & DEBUG_TRACK)              serialPrintFOptions(0, "Invalid packet\n");
    }
    // Reset ALL command buffer values
    hostCmdReset();
  } else {
    // do whatever else requires attention.
  }
}

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-05-11  v0.1.0
 *   - Initial code
 *
 * 2026-05-13  v0.1.1
 *   - Centralise all formatted print output in single function
 *
 * 2026-05-16  v0.1.2
 *   - Add PWM support for LED
 *
 * 2026-05-18  v0.1.3
 *   - Initial (non-functional) support for Serial1/UART1 debug output on PAx
 *
 * 2026-05-18  v0.1.4
 *   - Separated into multiple source headers
 *
 * 2026-05-18  v0.1.5
 *   - Changes to PWM FSM
 *
 * 2026-05-21  v0.1.6
 *   - Replace iButtonTag library with native OneWire functions
 *
 * 2026-05-21  v0.2.0
 *   - Rewritten 1-Wire support
 *
 * 2026-06-12  v0.2.1
 *   - StatusLED2 → OutputChannel; relay added as ch1 (digital mode)
 *
 * 2026-06-12  v0.3.0
 *   - 1-Wire split into OneWireBus + OneWireTag + DS18Temp
 *   - oneWireReadCheck auto-detects iButton vs DS18xx
 *
 * 2026-06-12  v0.4.0
 *   - ow485-comms: command history (8 deep), VT100 line editor, immediate
 *     RS485 echo; CMD_MAX=33
 *   - Fix bare serialWrite() calls in loop() that bypassed RS485 direction
 *     control
 *
 * 2026-06-15  v0.5.0
 *   - OW_DS18TEMP flag makes DS18xx support conditional (saves 37 B SRAM
 *     + DS18Temp.h when disabled)
 *   - oneWireReadCheck() rewritten: two-pass SEARCH ROM with dedicated
 *     scanTag — owTag persists until systemPoll() clears it
 *   - DS18 conversion throttled to OW_DS18_INTERVAL_MS (5 s)
 *   - systemPoll() added — empty command reports last tag + temperature
 *   - oneWireRead/Write renamed to oneWireReadTag/WriteTag (static)
 *   - All build and timing constants moved to platform-ow485.h
 */
