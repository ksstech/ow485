/**
 * DS18Temp.cpp  —  DS18xx temperature sensor driver
 *
 * VARIANT HANDLING
 * ────────────────
 * The driver auto-detects the device model from rom[0] (family byte) and
 * adjusts scratchpad write length and temperature calculation accordingly.
 *
 *   DS18S20 (0x10):  9-bit hardware.  Temperature LSB/MSB gives 0.5 °C steps.
 *                    Extended resolution uses bytes [6] (COUNT_REMAIN) and
 *                    [7] (COUNT_PER_C) for ~0.1 °C effective resolution.
 *
 *   DS18B20 (0x28):  Config register byte [4] bits 5–6 set 9–12 bit resolution.
 *   DS1822  (0x22):  Same register layout; slightly lower accuracy spec.
 *   DS1825  (0x3B):  Same as DS18B20 with address pin.
 *
 *   Temperature formula:
 *     DS18B20/DS1822/DS1825:  raw = (int16_t)(MSB<<8 | LSB);  tempC = raw / 16.0
 *     DS18S20:                raw = (int16_t)(MSB<<8 | LSB);
 *                             tempC = raw/2.0 - 0.25 + (COUNT_PER_C - COUNT_REMAIN) / (float)COUNT_PER_C
 */

#include "DS18Temp.h"
#include <string.h>

/* ── Constructor ────────────────────────────────────────────────────────── */
DS18Temp::DS18Temp(OneWireBus& bus)
    : _bus(bus),
      _convStartMs(0),
      _convDurationMs(DS18_CONV_MS_12),
      _converting(false)
{
    memset(rom, 0, OW_ROM_BYTES);
    memset(scratchpad, 0, DS18_SCRATCH_BYTES);
    tempC           = -999.0f;
    result          = OWResult::NO_PRESENCE;
    timestampMs     = 0;
    model           = DS18Model::UNKNOWN;
    parasiticPower  = false;
}

OWResult DS18Temp::_done(OWResult r)
{
    result      = r;
    timestampMs = millis();
    return r;
}

bool DS18Temp::_resetAndSelect()
{
    if (!_bus.reset()) return false;
    _bus.select(rom);
    return true;
}

/* ══════════════════════════════════════════════════════════════════════════
   Device discovery
   ══════════════════════════════════════════════════════════════════════════ */

void DS18Temp::identify()
{
    switch (rom[0]) {
        case DS18_FAMILY_S20:  model = DS18Model::DS18S20; break;
        case DS18_FAMILY_B20:  model = DS18Model::DS18B20; break;
        case DS18_FAMILY_1822: model = DS18Model::DS1822;  break;
        case DS18_FAMILY_1825: model = DS18Model::DS1825;  break;
        default:               model = DS18Model::UNKNOWN;  break;
    }
}

OWResult DS18Temp::detect()
{
    /* Read ROM — works when exactly one device is on the bus */
    if (!_bus.reset()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(0x33u);
    _bus.readBytes(rom, OW_ROM_BYTES);

    if (OneWireBus::crc8(rom, 7) != rom[7]) return _done(OWResult::CRC_ERROR);

    identify();
    if (model == DS18Model::UNKNOWN) return _done(OWResult::WRONG_FAMILY);

    /* Check power supply mode */
    if (!_resetAndSelect()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(DS18_CMD_READ_POWER);
    parasiticPower = (_bus.readBit() == 0);

    /* Read initial scratchpad to populate config/resolution */
    readScratchpad();

    return _done(OWResult::OK);
}

/* ══════════════════════════════════════════════════════════════════════════
   Temperature conversion
   ══════════════════════════════════════════════════════════════════════════ */

OWResult DS18Temp::startConversion()
{
    if (!_resetAndSelect()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(DS18_CMD_CONVERT);
    _convStartMs    = millis();
    _convDurationMs = conversionTimeMs();
    _converting     = true;
    return _done(OWResult::OK);
}

bool DS18Temp::isReady()
{
    if (!_converting) return true;

    /* Time-based check (works for both parasitic and external power) */
    if (millis() - _convStartMs >= _convDurationMs) {
        _converting = false;
        return true;
    }

    /* Bus-read check (external power only — parasitic holds bus HIGH) */
    if (!parasiticPower && _bus.readBit()) {
        _converting = false;
        return true;
    }

    return false;
}

OWResult DS18Temp::readTemp()
{
    const OWResult r = readScratchpad();
    if (r != OWResult::OK) return r;

    const int16_t raw = (int16_t)((uint16_t)scratchpad[1] << 8 | scratchpad[0]);

    if (model == DS18Model::DS18S20) {
        /* DS18S20 extended resolution:
         * tempC = floor(raw/2) - 0.25 + (COUNT_PER_C - COUNT_REMAIN) / COUNT_PER_C
         * This gives ~0.1 °C effective resolution from 9-bit hardware. */
        const uint8_t countRemain = scratchpad[6];
        const uint8_t countPerC   = scratchpad[7];
        if (countPerC == 0) {
            tempC = raw * 0.5f;    /* fallback: straight 0.5 °C steps */
        } else {
            tempC = (float)(raw >> 1) - 0.25f
                  + (float)(countPerC - countRemain) / (float)countPerC;
        }
    } else {
        /* DS18B20, DS1822, DS1825 — all use raw / 16.0 */
        tempC = raw / 16.0f;
    }

    _converting = false;
    return _done(OWResult::OK);
}

OWResult DS18Temp::convert()
{
    OWResult r = startConversion();
    if (r != OWResult::OK) return r;
    delay(_convDurationMs + 10u);    /* +10 ms safety margin */
    return readTemp();
}

/* ══════════════════════════════════════════════════════════════════════════
   Scratchpad operations
   ══════════════════════════════════════════════════════════════════════════ */

OWResult DS18Temp::readScratchpad()
{
    if (!_resetAndSelect()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(DS18_CMD_READ_SCRATCH);
    _bus.readBytes(scratchpad, DS18_SCRATCH_BYTES);
    if (OneWireBus::crc8(scratchpad, 8) != scratchpad[8])
        return _done(OWResult::CRC_ERROR);
    return _done(OWResult::OK);
}

OWResult DS18Temp::writeScratchpad()
{
    if (!_resetAndSelect()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(DS18_CMD_WRITE_SCRATCH);
    _bus.writeByte(scratchpad[2]);   /* TH */
    _bus.writeByte(scratchpad[3]);   /* TL */

    /* DS18B20/DS1822/DS1825 have a config register; DS18S20 does not */
    if (model != DS18Model::DS18S20) {
        _bus.writeByte(scratchpad[4]);  /* Config */
    }

    return _done(OWResult::OK);
}

OWResult DS18Temp::copyScratchpad()
{
    if (!_resetAndSelect()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(DS18_CMD_COPY_SCRATCH);
    delay(10);  /* EEPROM write time (max 10 ms per datasheet) */
    return _done(OWResult::OK);
}

OWResult DS18Temp::recallEeprom()
{
    if (!_resetAndSelect()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(DS18_CMD_RECALL_EEPROM);
    /* Wait until recall is complete (device pulls bus LOW while busy) */
    uint32_t start = millis();
    while (_bus.readBit() == 0) {
        if (millis() - start > 20u) return _done(OWResult::BUS_ERROR);
    }
    return _done(OWResult::OK);
}

/* ══════════════════════════════════════════════════════════════════════════
   Configuration helpers
   ══════════════════════════════════════════════════════════════════════════ */

void DS18Temp::setResolution(uint8_t bits)
{
    if (model == DS18Model::DS18S20) return;  /* fixed 9-bit hardware */
    if (bits < 9)  bits = 9;
    if (bits > 12) bits = 12;
    /* Config register: bits 5–6 hold resolution, rest are reserved 1s.
     * 9-bit=0x1F, 10-bit=0x3F, 11-bit=0x5F, 12-bit=0x7F */
    scratchpad[4] = ((bits - 9) << 5) | 0x1Fu;
}

uint8_t DS18Temp::getResolution() const
{
    if (model == DS18Model::DS18S20) return 9;
    return 9 + ((scratchpad[4] >> 5) & 0x03u);
}

void DS18Temp::setAlarms(int8_t high, int8_t low)
{
    scratchpad[2] = (uint8_t)high;
    scratchpad[3] = (uint8_t)low;
}

uint16_t DS18Temp::conversionTimeMs() const
{
    switch (getResolution()) {
        case 9:  return DS18_CONV_MS_9;
        case 10: return DS18_CONV_MS_10;
        case 11: return DS18_CONV_MS_11;
        default: return DS18_CONV_MS_12;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
   Print helpers
   ══════════════════════════════════════════════════════════════════════════ */

const char* DS18Temp::modelName(DS18Model m)
{
    switch (m) {
        case DS18Model::DS18S20: return "DS18S20";
        case DS18Model::DS18B20: return "DS18B20";
        case DS18Model::DS1822:  return "DS1822";
        case DS18Model::DS1825:  return "DS1825";
        default:                 return "DS18?";
    }
}

void DS18Temp::printStatus() const
{
    Serial.print(F("  model:  ")); Serial.println(modelName(model));
    Serial.print(F("  result: ")); OneWireBus::printResult(result);
    Serial.print(F("  tempC:  ")); Serial.println(tempC, 2);
    Serial.print(F("  res:    ")); Serial.print(getResolution()); Serial.println(F("-bit"));
    Serial.print(F("  TH/TL:  ")); Serial.print(getAlarmHigh());
    Serial.print(F(" / ")); Serial.println(getAlarmLow());
    Serial.print(F("  power:  ")); Serial.println(parasiticPower ? F("parasitic") : F("external"));
}

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-06-12
 *   - New file.  Implementation of the DS18xx driver.
 *   - Variant-aware temperature math:
 *       DS18B20/DS1822/DS1825:  tempC = raw / 16.0
 *       DS18S20:                tempC = (raw>>1) - 0.25
 *                                       + (COUNT_PER_C - COUNT_REMAIN)/COUNT_PER_C
 *       (DS18S20 path falls back to 0.5 °C steps if COUNT_PER_C reads 0.)
 *   - writeScratchpad() sends 3 bytes (TH/TL/config) for DS18B20-class parts,
 *     2 bytes (TH/TL) for DS18S20 which has no config register.
 *   - Resolution encoded in config byte bits 5–6: 9b=0x1F, 10b=0x3F, 11b=0x5F,
 *     12b=0x7F.  Conversion times 94/188/375/750 ms respectively.
 *   - detect() reads ROM, identifies model, probes power mode (READ_POWER
 *     0xB4), and loads the initial scratchpad.
 *   - printModel renamed modelName(), returns const char* for use in
 *     formatted RS485 output without Serial.print dependency.
 */
