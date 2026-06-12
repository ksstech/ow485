/**
 * OneWireBus.h  —  Low-level 1-Wire bus driver using direct VPORT access
 *                   for ATtiny3224 / megaTinyCore 2.6.x
 *
 * Extracted from OneWireTag so that multiple higher-level drivers (iButton,
 * DS18xx, etc.) can share one bus instance without duplicating primitives.
 *
 * Usage:
 *   OneWireBus  bus(PIN_PA4);
 *   OneWireTag  tag(bus);
 *   DS18Temp    temp(bus);
 */

#pragma once
#include <Arduino.h>

static constexpr uint8_t OW_ROM_BYTES = 8u;

enum class OWResult : uint8_t {
    OK            = 0,
    NO_PRESENCE   = 1,
    CRC_ERROR     = 2,
    WRONG_FAMILY  = 3,
    VERIFY_FAIL   = 4,
    BUS_ERROR     = 5,
};

class OneWireBus {
public:
    explicit OneWireBus(uint8_t pin);

    /* ── Standard bus operations ───────────────────────────────────────── */
    bool    reset();
    void    writeBit(uint8_t bit);
    uint8_t readBit();
    void    writeByte(uint8_t byte);
    uint8_t readByte();

    /** Read 'len' bytes into 'buf'. */
    void readBytes(uint8_t* buf, uint8_t len);

    /* ── ROM addressing ───────────────────────────────────────────────── */
    /** MATCH ROM (0x55) — address a specific device by its 8-byte ROM. */
    void select(const uint8_t* rom);

    /** SKIP ROM (0xCC) — address the single device on the bus. */
    void skipRom();

    /* ── CRC ──────────────────────────────────────────────────────────── */
    static uint8_t crc8(const uint8_t* data, uint8_t len);

    /* ── Utility ──────────────────────────────────────────────────────── */
    static void    printRomCode(const uint8_t* rom);
    static void    printResult(OWResult r);

    /** Return the Arduino pin number (for non-timing-critical direct access). */
    uint8_t pin() const { return _pin; }

private:
    volatile uint8_t* _vport;
    uint8_t           _bitmask;
    uint8_t           _pin;
};

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-06-12
 *   - New file.  Low-level 1-Wire bus primitives extracted from OneWireTag
 *     so multiple device drivers (OneWireTag, DS18Temp) share one bus
 *     instance on the same pin without duplicating code.
 *   - Public API: reset, writeBit/readBit, writeByte/readByte, readBytes,
 *     select (MATCH ROM 0x55), skipRom (SKIP ROM 0xCC), crc8, print helpers.
 *   - Owns OWResult enum and OW_ROM_BYTES (single definition site).
 *   - pin() accessor added so higher-level drivers can do non-timing-critical
 *     direct GPIO (e.g. RW1990 5–10 ms programming pulses).
 *   - Uses direct VPORT register access (DIR/OUT/IN at +0/+1/+2).  Does NOT
 *     use PaulStoffregen/OneWire: that library's classic-AVR offsets (+1/+2
 *     from PINx) land on INTFLAGS / wrong-port DIR on tinyAVR 2-series.
 */
