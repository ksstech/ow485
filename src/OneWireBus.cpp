/**
 * OneWireBus.cpp  —  1-Wire bus primitives using direct VPORT register access
 *
 * TIMING (F_CPU = 10 MHz, tinyAVR 2-series Speed Grade 2 at 3.3 V)
 * ──────
 *  Reset:     520 µs LOW, release, poll presence over full 480 µs recovery.
 *  Write '1': 8 µs LOW, release 62 µs.
 *  Write '0': 65 µs LOW, release 5 µs.
 *  Read:      2 µs LOW start, release, sample at ~10 µs, 55 µs recovery.
 *
 * WHY NOT Paul Stoffregen's OneWire library:
 *  The library uses classic AVR register offset arithmetic (PINx+1=DDRx,
 *  PINx+2=PORTx).  On tinyAVR 2-series the VPORT layout is DIR/OUT/IN,
 *  and portInputRegister() returns &VPORTx.IN.  The +1/+2 offsets land on
 *  INTFLAGS and the next port's DIR — completely wrong registers.
 *  See PaulStoffregen/OneWire PR #94 (open since 2020, never merged).
 */

#include "OneWireBus.h"

static constexpr uint8_t VPORT_DIR = 0;
static constexpr uint8_t VPORT_OUT = 1;
static constexpr uint8_t VPORT_IN  = 2;

/* ── Constructor ────────────────────────────────────────────────────────── */
OneWireBus::OneWireBus(uint8_t pin)
{
    _pin = pin;
    uint8_t port = digitalPinToPort(pin);
    _vport   = (volatile uint8_t*)(&VPORTA) + (port * 4u);
    _bitmask = digitalPinToBitMask(pin);
    _vport[VPORT_DIR] &= ~_bitmask;
}

/* ── Bus operations ─────────────────────────────────────────────────────── */

bool OneWireBus::reset()
{
    _vport[VPORT_DIR] &= ~_bitmask;
    delayMicroseconds(10);

    noInterrupts();
    _vport[VPORT_OUT] &= ~_bitmask;
    _vport[VPORT_DIR] |=  _bitmask;
    interrupts();
    delayMicroseconds(520);

    noInterrupts();
    _vport[VPORT_DIR] &= ~_bitmask;
    interrupts();

    bool present = false;
    for (uint8_t i = 0; i < 48u; ++i) {
        delayMicroseconds(10);
        if (!(_vport[VPORT_IN] & _bitmask)) present = true;
    }
    return present;
}

void OneWireBus::writeBit(uint8_t bit)
{
    if (bit & 1) {
        noInterrupts();
        _vport[VPORT_OUT] &= ~_bitmask;
        _vport[VPORT_DIR] |=  _bitmask;
        delayMicroseconds(8);
        _vport[VPORT_DIR] &= ~_bitmask;
        interrupts();
        delayMicroseconds(62);
    } else {
        noInterrupts();
        _vport[VPORT_OUT] &= ~_bitmask;
        _vport[VPORT_DIR] |=  _bitmask;
        delayMicroseconds(65);
        _vport[VPORT_DIR] &= ~_bitmask;
        interrupts();
        delayMicroseconds(5);
    }
}

uint8_t OneWireBus::readBit()
{
    uint8_t r;
    noInterrupts();
    _vport[VPORT_OUT] &= ~_bitmask;
    _vport[VPORT_DIR] |=  _bitmask;
    delayMicroseconds(2);
    _vport[VPORT_DIR] &= ~_bitmask;
    delayMicroseconds(8);
    r = (_vport[VPORT_IN] & _bitmask) ? 1u : 0u;
    interrupts();
    delayMicroseconds(55);
    return r;
}

void OneWireBus::writeByte(uint8_t byte)
{
    for (uint8_t bit = 0; bit < 8u; ++bit)
        writeBit((byte >> bit) & 1u);
}

uint8_t OneWireBus::readByte()
{
    uint8_t byte = 0;
    for (uint8_t bit = 0; bit < 8u; ++bit)
        byte |= (readBit() << bit);
    return byte;
}

void OneWireBus::readBytes(uint8_t* buf, uint8_t len)
{
    for (uint8_t i = 0; i < len; ++i)
        buf[i] = readByte();
}

/* ── ROM addressing ─────────────────────────────────────────────────────── */

void OneWireBus::select(const uint8_t* rom)
{
    writeByte(0x55u);   /* MATCH ROM */
    for (uint8_t i = 0; i < OW_ROM_BYTES; ++i)
        writeByte(rom[i]);
}

void OneWireBus::skipRom()
{
    writeByte(0xCCu);   /* SKIP ROM */
}

/* ── CRC8 — Dallas/Maxim (polynomial 0x31, reflected) ──────────────────── */

uint8_t OneWireBus::crc8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0x00u;
    for (uint8_t i = 0; i < len; ++i) {
        uint8_t byte = data[i];
        for (uint8_t bit = 0; bit < 8u; ++bit) {
            const uint8_t mix = (crc ^ byte) & 0x01u;
            crc >>= 1;
            if (mix) crc ^= 0x8Cu;
            byte >>= 1;
        }
    }
    return crc;
}

/* ── Print utilities ────────────────────────────────────────────────────── */

void OneWireBus::printRomCode(const uint8_t* rom)
{
    for (uint8_t i = 0; i < OW_ROM_BYTES; ++i) {
        if (rom[i] < 0x10u) Serial.print('0');
        Serial.print(rom[i], HEX);
        if (i < OW_ROM_BYTES - 1u) Serial.print(':');
    }
    Serial.println();
}

void OneWireBus::printResult(OWResult r)
{
    switch (r) {
        case OWResult::OK:           Serial.println(F("OK"));           break;
        case OWResult::NO_PRESENCE:  Serial.println(F("NO_PRESENCE"));  break;
        case OWResult::CRC_ERROR:    Serial.println(F("CRC_ERROR"));    break;
        case OWResult::WRONG_FAMILY: Serial.println(F("WRONG_FAMILY")); break;
        case OWResult::VERIFY_FAIL:  Serial.println(F("VERIFY_FAIL"));  break;
        case OWResult::BUS_ERROR:    Serial.println(F("BUS_ERROR"));    break;
        default:                     Serial.println(F("UNKNOWN"));      break;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-06-12
 *   - New file.  Implementation of the shared bus primitives.
 *   - Bus timing tuned for F_CPU = 10 MHz (tinyAVR 2-series Speed Grade 2 at
 *     3.3 V): reset 520 µs LOW + full 480 µs presence poll (no early break);
 *     read slot samples at ~10 µs (within tRDV = 15 µs).
 *   - CRC8 implemented locally (Dallas/Maxim poly 0x31, reflected).
 *   - Constructor stores _pin and resolves _vport/_bitmask via megaTinyCore's
 *     digitalPinToPort()/digitalPinToBitMask() — robust against the core's
 *     non-obvious pin numbering (PIN_PA4 = Arduino pin 0).
 */
