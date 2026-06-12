/**
 * OneWireTag.cpp  —  DS1990 / RW1990 iButton driver
 *                    Direct VPORT implementation for ATtiny3224
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  BUS TIMING SUMMARY (all operations, F_CPU = 10 MHz)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Reset:      520 µs LOW, release, poll presence over full 480 µs recovery.
 *              Tag must respond within 300 µs (tPDHIGH + tPDLOW).
 *
 *  Write '1':  8 µs LOW, release for 62 µs.   (spec: 1–15 µs LOW)
 *  Write '0':  65 µs LOW, release for 5 µs.   (spec: 60–120 µs LOW)
 *
 *  Read:       2 µs LOW start, release, sample at ~10 µs, 55 µs recovery.
 *              Sample is well within tRDV = 15 µs max.
 *
 *  RW1990 write: 5 ms ('1') or 10 ms ('0') LOW, 2 ms release.
 *                Interrupts enabled during delay() — millis()/PWM unaffected.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  CRC8 — DALLAS/MAXIM 1-WIRE (polynomial 0x31, reflected)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Implemented locally; no OneWire library dependency.
 *  A valid ROM code satisfies: crc8(rom, 7) == rom[7].
 */

#include "OneWireTag.h"
#include "rs485Support.h"
#include <string.h>    /* memcmp, memcpy */

/* ── VPORT register offsets ─────────────────────────────────────────────── */
/*  tinyAVR 2-series VPORT layout (each VPORTx is 4 bytes):
 *    +0 = DIR      (direction: 1 = output, 0 = input)
 *    +1 = OUT      (output value when direction = output)
 *    +2 = IN       (read pin state — always, regardless of direction)
 *    +3 = INTFLAGS
 */
static constexpr uint8_t VPORT_DIR = 0;
static constexpr uint8_t VPORT_OUT = 1;
static constexpr uint8_t VPORT_IN  = 2;

/* ── Constructor ────────────────────────────────────────────────────────── */
OneWireTag::OneWireTag(uint8_t pin)
{
    uint8_t port = digitalPinToPort(pin);
    _vport   = (volatile uint8_t*)(&VPORTA) + (port * 4u);
    _bitmask = digitalPinToBitMask(pin);

    _vport[VPORT_DIR] &= ~_bitmask;

    memset(rom, 0, OW_ROM_BYTES);
    result      = OWResult::NO_PRESENCE;
    timestampMs = 0;
}

/* ── Result + timestamp helper ──────────────────────────────────────────── */
OWResult OneWireTag::_done(OWResult r)
{
    result      = r;
    timestampMs = millis();
    return r;
}

/* ══════════════════════════════════════════════════════════════════════════
   Low-level bus operations — all using VPORT directly
   ══════════════════════════════════════════════════════════════════════════ */

bool OneWireTag::_reset()
{
    /* Ensure bus is released before starting */
    _vport[VPORT_DIR] &= ~_bitmask;
    delayMicroseconds(10);

    /* Drive bus LOW for 520 µs (>= 480 µs minimum reset pulse) */
    noInterrupts();
    _vport[VPORT_OUT] &= ~_bitmask;       /* OUT = 0 (will drive LOW) */
    _vport[VPORT_DIR] |=  _bitmask;        /* DIR = output → bus goes LOW */
    interrupts();
    delayMicroseconds(520);

    /* Release bus and poll for presence over the FULL 480 µs recovery.
     * Do NOT break early — the full 480 µs is mandatory recovery time. */
    noInterrupts();
    _vport[VPORT_DIR] &= ~_bitmask;        /* DIR = input → pullup restores HIGH */
    interrupts();

    bool present = false;
    for (uint8_t i = 0; i < 48u; ++i) {
        delayMicroseconds(10);
        if (!(_vport[VPORT_IN] & _bitmask)) present = true;   /* no break */
    }
    return present;
}

void OneWireTag::_writeBit(uint8_t bit)
{
    if (bit & 1) {
        /* Write '1': short LOW (8 µs), then release for rest of slot */
        noInterrupts();
        _vport[VPORT_OUT] &= ~_bitmask;
        _vport[VPORT_DIR] |=  _bitmask;
        delayMicroseconds(8);
        _vport[VPORT_DIR] &= ~_bitmask;
        interrupts();
        delayMicroseconds(62);
    } else {
        /* Write '0': long LOW (65 µs), then brief release */
        noInterrupts();
        _vport[VPORT_OUT] &= ~_bitmask;
        _vport[VPORT_DIR] |=  _bitmask;
        delayMicroseconds(65);
        _vport[VPORT_DIR] &= ~_bitmask;
        interrupts();
        delayMicroseconds(5);
    }
}

uint8_t OneWireTag::_readBit()
{
    uint8_t r;
    noInterrupts();
    _vport[VPORT_OUT] &= ~_bitmask;
    _vport[VPORT_DIR] |=  _bitmask;       /* drive LOW — start of read slot */
    delayMicroseconds(2);
    _vport[VPORT_DIR] &= ~_bitmask;       /* release — tag may now drive */
    delayMicroseconds(8);
    r = (_vport[VPORT_IN] & _bitmask) ? 1u : 0u;   /* sample at ~10 µs */
    interrupts();
    delayMicroseconds(55);                 /* complete slot (~65 µs total) */
    return r;
}

void OneWireTag::_writeByte(uint8_t byte)
{
    for (uint8_t bit = 0; bit < 8u; ++bit) {
        _writeBit((byte >> bit) & 1u);     /* LSB first */
    }
}

uint8_t OneWireTag::_readByte()
{
    uint8_t byte = 0;
    for (uint8_t bit = 0; bit < 8u; ++bit) {
        byte |= (_readBit() << bit);       /* LSB first */
    }
    return byte;
}

/* ── RW1990 extended-pulse write bit ────────────────────────────────────── */

void OneWireTag::_writeBitRW(uint8_t bit)
{
    /*
     * 5 ms LOW = '1', 10 ms LOW = '0'.
     * Interrupts enabled during delay() so millis() and PWM continue.
     */
    _vport[VPORT_OUT] &= ~_bitmask;
    _vport[VPORT_DIR] |=  _bitmask;
    delay(bit ? RW_PULSE_ONE_MS : RW_PULSE_ZERO_MS);

    _vport[VPORT_DIR] &= ~_bitmask;       /* release — pullup restores HIGH */
    delay(RW_RELEASE_MS);
}

/* ══════════════════════════════════════════════════════════════════════════
   Public API — all operations use the public rom[] buffer directly
   ══════════════════════════════════════════════════════════════════════════ */

OWResult OneWireTag::readRaw()
{
    if (!_reset()) return _done(OWResult::NO_PRESENCE);
    _writeByte(0x33u);                                     /* READ ROM */
    for (uint8_t i = 0; i < OW_ROM_BYTES; ++i) rom[i] = _readByte();
    if (crc8(rom, 7) != rom[7]) return _done(OWResult::CRC_ERROR);
    return _done(OWResult::OK);
}

OWResult OneWireTag::read()
{
    const OWResult r = readRaw();
    if (r != OWResult::OK) return r;
    if (!isCompatible(rom)) return _done(OWResult::WRONG_FAMILY);
    return _done(OWResult::OK);
}

OWResult OneWireTag::scanNext()
{
    if (!_reset()) return _done(OWResult::NO_PRESENCE);
    _writeByte(0xF0u);                                     /* SEARCH ROM */
    for (uint8_t i = 0; i < OW_ROM_BYTES; ++i) {
        uint8_t byte = 0;
        for (uint8_t b = 0; b < 8; ++b) {
            uint8_t bit    = _readBit();
            uint8_t bitCmp = _readBit();
            if (bit && bitCmp) return _done(OWResult::NO_PRESENCE);
            _writeBit(bit);
            byte |= (bit << b);
        }
        rom[i] = byte;
    }
    if (crc8(rom, 7) != rom[7]) return _done(OWResult::CRC_ERROR);
    if (!isCompatible(rom)) return _done(OWResult::WRONG_FAMILY);
    return _done(OWResult::OK);
}

void OneWireTag::resetSearch()
{
    delay(1);
    _done(OWResult::OK);
}

/* ── Write / Verify / Program — data taken from rom[] ───────────────────── */

OWResult OneWireTag::write(bool useAltCmd)
{
    if (!_reset()) return _done(OWResult::NO_PRESENCE);
    _writeByte(useAltCmd ? RW_CMD_WRITE_ALT : RW_CMD_WRITE);
    delay(RW_ENTER_MS);
    for (uint8_t byte = 0; byte < OW_ROM_BYTES; ++byte) {
        for (uint8_t bit = 0; bit < 8; ++bit) {
            _writeBitRW((rom[byte] >> bit) & 0x01u);
        }
    }
    if (!_reset()) return _done(OWResult::NO_PRESENCE);
    delay(RW_SETTLE_MS);
    return _done(OWResult::OK);
}

OWResult OneWireTag::verify()
{
    /* Save expected ROM, then read the tag into rom[].
     * After verify, rom[] always holds the tag's actual current data. */
    uint8_t expected[OW_ROM_BYTES];
    memcpy(expected, rom, OW_ROM_BYTES);

    const OWResult r = readRaw();
    if (r != OWResult::OK) return r;
    if (memcmp(rom, expected, OW_ROM_BYTES) != 0) return _done(OWResult::VERIFY_FAIL);
    return _done(OWResult::OK);
}

OWResult OneWireTag::program(bool useAltCmd)
{
    if (rom[7] == 0x00u) rom[7] = crc8(rom, 7);
    if (crc8(rom, 7) != rom[7]) return _done(OWResult::CRC_ERROR);
    const OWResult wr = write(useAltCmd);
    if (wr != OWResult::OK) return wr;
    return verify();
}

/* ══════════════════════════════════════════════════════════════════════════
   CRC8 — Dallas/Maxim 1-Wire (polynomial 0x31, reflected, init 0x00)
   ══════════════════════════════════════════════════════════════════════════ */

uint8_t OneWireTag::crc8(const uint8_t* data, uint8_t len)
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

/* ══════════════════════════════════════════════════════════════════════════
   Static utilities
   ══════════════════════════════════════════════════════════════════════════ */

bool OneWireTag::isCompatible(const uint8_t* romCode)
{
    return (romCode[0] == OW_FAMILY_DS1990);
}

void OneWireTag::buildRomCode(const uint8_t* serial6, uint8_t* romCode)
{
    romCode[0] = OW_FAMILY_DS1990;
    memcpy(&romCode[1], serial6, 6);
    romCode[7] = crc8(romCode, 7);
}

void OneWireTag::printRomCode(const uint8_t* romCode)
{
    for (uint8_t i = 0; i < OW_ROM_BYTES; ++i) {
        if (romCode[i] < 0x10u) Serial.print('0');
        Serial.print(romCode[i], HEX);
        if (i < OW_ROM_BYTES - 1u) Serial.print(':');
    }
    Serial.println();
}

void OneWireTag::printResult(OWResult r)
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

void OneWireTag::printRomInfo(bool reverse)
{
    const char * pMsg;
    switch (result) {
        case OWResult::OK:           pMsg = "OK";           break;
        case OWResult::NO_PRESENCE:  pMsg = "NO_PRESENCE";  break;
        case OWResult::CRC_ERROR:    pMsg = "CRC_ERROR";    break;
        case OWResult::WRONG_FAMILY: pMsg = "WRONG_FAMILY"; break;
        case OWResult::VERIFY_FAIL:  pMsg = "VERIFY_FAIL";  break;
        case OWResult::BUS_ERROR:    pMsg = "BUS_ERROR";    break;
        default:                     pMsg = "UNKNOWN";      break;
    }
    char buf[48];
    int iRV = snprintf(buf, sizeof(buf), "F=%02x [", rom[0]);
    for (uint8_t i = 1; i < 7; ++i) {
        iRV += snprintf(buf+iRV, sizeof(buf)-iRV, "%02x", reverse ? rom[7-i]: rom[i]);
    }
    iRV += snprintf(buf+iRV, sizeof(buf)-iRV, "] C=%02x (%d) %s\n", rom[7], (int)result, pMsg);
    serialWrite(buf);
}
