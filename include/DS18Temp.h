// Copyright (c) 2026 Andre M. Maree / KSS Technologies (Pty) Ltd.
/**
 * DS18Temp.h  —  DS18xx 1-Wire temperature sensor driver
 *
 * AUTO-DETECTS AND HANDLES ALL VARIANTS:
 *   Family 0x10  DS18S20, DS1820   — 9-bit fixed, extended resolution via COUNT_REMAIN
 *   Family 0x28  DS18B20           — 9–12 bit configurable (default 12-bit)
 *   Family 0x22  DS1822            — 9–12 bit configurable, lower accuracy
 *   Family 0x3B  DS1825            — same protocol as DS18B20
 *
 * SCRATCHPAD LAYOUT (9 bytes, all variants):
 *   [0] Temperature LSB
 *   [1] Temperature MSB
 *   [2] TH register (alarm high, signed int8)
 *   [3] TL register (alarm low, signed int8)
 *   [4] Configuration register (DS18B20/DS1822: bits 5–6 = resolution)
 *   [5] Reserved (0xFF)
 *   [6] Reserved (DS18B20: 0x0C) / COUNT_REMAIN (DS18S20)
 *   [7] Reserved (0x10) / COUNT_PER_C (DS18S20)
 *   [8] CRC
 *
 * USAGE:
 *   OneWireBus bus(PIN_PA4);
 *   DS18Temp   temp(bus);
 *
 *   temp.detect();                // find device, identify model
 *   temp.setResolution(12);       // 9–12 (DS18B20/DS1822 only)
 *   temp.writeScratchpad();       // send config to device
 *
 *   temp.startConversion();       // non-blocking
 *   // ... do other work ...
 *   if (temp.isReady()) {
 *       temp.readTemp();
 *       Serial.println(temp.tempC);
 *   }
 */

#pragma once
#include "OneWireBus.h"

/* ── Scratchpad size ────────────────────────────────────────────────────── */
static constexpr uint8_t DS18_SCRATCH_BYTES = 9u;

/* ── Family codes ───────────────────────────────────────────────────────── */
static constexpr uint8_t DS18_FAMILY_S20  = 0x10u;  ///< DS18S20, DS1820
static constexpr uint8_t DS18_FAMILY_B20  = 0x28u;  ///< DS18B20
static constexpr uint8_t DS18_FAMILY_1822 = 0x22u;  ///< DS1822
static constexpr uint8_t DS18_FAMILY_1825 = 0x3Bu;  ///< DS1825

/* ── 1-Wire commands ────────────────────────────────────────────────────── */
static constexpr uint8_t DS18_CMD_CONVERT      = 0x44u;
static constexpr uint8_t DS18_CMD_READ_SCRATCH  = 0xBEu;
static constexpr uint8_t DS18_CMD_WRITE_SCRATCH = 0x4Eu;
static constexpr uint8_t DS18_CMD_COPY_SCRATCH  = 0x48u;
static constexpr uint8_t DS18_CMD_RECALL_EEPROM = 0xB8u;
static constexpr uint8_t DS18_CMD_READ_POWER    = 0xB4u;

/* ── Device model (auto-detected from family byte) ──────────────────────── */
enum class DS18Model : uint8_t {
    UNKNOWN  = 0,
    DS18S20  = 1,   ///< Family 0x10 — 9-bit fixed, extended via COUNT_REMAIN
    DS18B20  = 2,   ///< Family 0x28 — 9–12 bit configurable
    DS1822   = 3,   ///< Family 0x22 — 9–12 bit configurable
    DS1825   = 4,   ///< Family 0x3B — same as DS18B20
};

/* ── Conversion times by resolution (milliseconds) ──────────────────────── */
static constexpr uint16_t DS18_CONV_MS_9  =  94u;
static constexpr uint16_t DS18_CONV_MS_10 = 188u;
static constexpr uint16_t DS18_CONV_MS_11 = 375u;
static constexpr uint16_t DS18_CONV_MS_12 = 750u;

/* ── DS18Temp class ─────────────────────────────────────────────────────── */
class DS18Temp {
public:
    explicit DS18Temp(OneWireBus& bus);

    /* ── Public state ──────────────────────────────────────────────────── */
    uint8_t   rom[OW_ROM_BYTES];                ///< Device ROM (set by detect())
    uint8_t   scratchpad[DS18_SCRATCH_BYTES];   ///< Last scratchpad read/pending write
    float     tempC;                            ///< Last computed temperature (°C)
    OWResult  result;                           ///< Last operation result
    uint32_t  timestampMs;                      ///< millis() of last operation
    DS18Model model;                            ///< Auto-detected device model
    bool      parasiticPower;                   ///< True if device uses parasitic power

    /* ── Device discovery ──────────────────────────────────────────────── */
    /**
     * Find the first DS18xx on the bus via READ ROM (single device) or
     * SEARCH ROM.  Populates rom[], identifies model, reads power mode.
     * For a known device, populate rom[] manually and call identify().
     */
    OWResult detect();

    /** Identify model from rom[0].  Call after manually setting rom[]. */
    void identify();

    /* ── Temperature conversion ───────────────────────────────────────── */
    /** Issue CONVERT T command.  Non-blocking; call isReady() to poll. */
    OWResult startConversion();

    /** True when conversion is complete (polls bus or checks elapsed time). */
    bool isReady();

    /**
     * Read scratchpad and compute tempC.  Call after isReady() returns true.
     * Handles all variant-specific temperature calculations automatically.
     */
    OWResult readTemp();

    /**
     * Blocking convenience: startConversion, wait, readTemp in one call.
     * Blocks for up to 750 ms depending on resolution.
     */
    OWResult convert();

    /* ── Scratchpad operations ────────────────────────────────────────── */
    /** Read all 9 scratchpad bytes into scratchpad[]. */
    OWResult readScratchpad();

    /**
     * Write scratchpad registers to the device.
     * DS18B20/DS1822: writes scratchpad[2] (TH), [3] (TL), [4] (config).
     * DS18S20: writes scratchpad[2] (TH), [3] (TL) only.
     * Modify scratchpad[] before calling (or use setResolution/setAlarms).
     */
    OWResult writeScratchpad();

    /** Copy scratchpad to device EEPROM (persists TH, TL, config across power cycles). */
    OWResult copyScratchpad();

    /** Recall EEPROM values back into scratchpad. */
    OWResult recallEeprom();

    /* ── Configuration helpers (modify scratchpad[] in-place) ─────────── */
    /**
     * Set resolution: 9, 10, 11, or 12 bits.
     * DS18B20/DS1822: modifies scratchpad[4] bits 5–6.
     * DS18S20: no effect (fixed 9-bit hardware).
     * Call writeScratchpad() after to send to device.
     */
    void setResolution(uint8_t bits);

    /** Get current resolution from scratchpad[4] (or 9 for DS18S20). */
    uint8_t getResolution() const;

    /** Set alarm high/low thresholds (signed °C, −55 to +125). */
    void setAlarms(int8_t high, int8_t low);
    int8_t getAlarmHigh() const { return (int8_t)scratchpad[2]; }
    int8_t getAlarmLow()  const { return (int8_t)scratchpad[3]; }

    /** Conversion time in ms for the current resolution. */
    uint16_t conversionTimeMs() const;

    /* ── Print helpers ────────────────────────────────────────────────── */
    static const char* modelName(DS18Model m);
    void printStatus() const;

private:
    OneWireBus& _bus;
    uint32_t    _convStartMs;
    uint16_t    _convDurationMs;
    bool        _converting;

    OWResult _done(OWResult r);
    bool     _resetAndSelect();
};

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-06-12
 *   - New file.  DS18xx temperature sensor driver over the shared OneWireBus.
 *   - Auto-detects model from rom[0]: DS18S20/DS1820 (0x10), DS18B20 (0x28),
 *     DS1822 (0x22), DS1825 (0x3B).
 *   - Public state: rom[8], scratchpad[9], tempC, result, timestampMs, model,
 *     parasiticPower (same parameterless / public-buffer style as OneWireTag).
 *   - Full scratchpad read/write, copy-to-EEPROM, recall-from-EEPROM.
 *   - Configuration helpers: setResolution (9–12 bit, no-op on DS18S20),
 *     setAlarms (TH/TL), getResolution, conversionTimeMs.
 *   - Non-blocking conversion: startConversion() + isReady() + readTemp(),
 *     plus blocking convert() convenience.  isReady() uses elapsed-time check
 *     (works for parasitic power) and bus-read check (external power).
 *   - printModel renamed to modelName() returning const char* so callers can
 *     embed the name in their own formatted output (avoids Serial.print coupling).
 */
