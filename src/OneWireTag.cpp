/**
 * OneWireTag.cpp  —  DS1990 / RW1990 iButton driver
 *
 * All standard 1-Wire bus operations are delegated to OneWireBus.
 * Only the RW1990 extended-pulse write (6–12 ms per bit) is handled here,
 * using Arduino GPIO calls — the pulse widths have no µs-level sensitivity.
 *
 * NOTE — _writeBitRW uses pinMode/digitalWrite (Arduino GPIO) not VPORT:
 *   The 6–12 ms pulse widths tolerate microsecond-level jitter; the
 *   higher-overhead Arduino API is safe here.  The tradeoff is that
 *   pinMode(INPUT) disables the internal pullup (OUT=0 already set before
 *   direction change), so bus release relies on the external 2 kΩ pullup.
 *   If porting to a board without an external pullup, either enable it
 *   explicitly (digitalWrite(pin, HIGH) before pinMode(INPUT)) or switch
 *   back to direct VPORT access as done in OneWireBus.
 */

#include "OneWireTag.h"
#include <string.h>

/* ── Constructor ────────────────────────────────────────────────────────── */
OneWireTag::OneWireTag(OneWireBus& bus) : _bus(bus)
{
    memset(rom, 0, OW_ROM_BYTES);
    result      = OWResult::NO_PRESENCE;
    timestampMs = 0;
}

OWResult OneWireTag::_done(OWResult r)
{
    result      = r;
    timestampMs = millis();
    return r;
}

/* ── Read ───────────────────────────────────────────────────────────────── */

OWResult OneWireTag::readRaw()
{
    if (!_bus.reset()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(0x33u);                             /* READ ROM */
    _bus.readBytes(rom, OW_ROM_BYTES);
    if (OneWireBus::crc8(rom, 7) != rom[7]) return _done(OWResult::CRC_ERROR);
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
    if (!_bus.reset()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(0xF0u);
    for (uint8_t i = 0; i < OW_ROM_BYTES; ++i) {
        uint8_t byte = 0;
        for (uint8_t b = 0; b < 8; ++b) {
            uint8_t bit    = _bus.readBit();
            uint8_t bitCmp = _bus.readBit();
            if (bit && bitCmp) return _done(OWResult::NO_PRESENCE);
            _bus.writeBit(bit);
            byte |= (bit << b);
        }
        rom[i] = byte;
    }
    if (OneWireBus::crc8(rom, 7) != rom[7]) return _done(OWResult::CRC_ERROR);
    if (!isCompatible(rom)) return _done(OWResult::WRONG_FAMILY);
    return _done(OWResult::OK);
}

void OneWireTag::resetSearch() { delay(1); _done(OWResult::OK); }

/* ── Write / Verify / Program ───────────────────────────────────────────── */

OWResult OneWireTag::write(bool useAltCmd)
{
    if (!_bus.reset()) return _done(OWResult::NO_PRESENCE);
    _bus.writeByte(useAltCmd ? RW_CMD_WRITE_ALT : RW_CMD_WRITE);
    delay(RW_ENTER_MS);
    for (uint8_t i = 0; i < OW_ROM_BYTES; ++i)
        for (uint8_t b = 0; b < 8; ++b)
            _writeBitRW((rom[i] >> b) & 0x01u);
    if (!_bus.reset()) return _done(OWResult::NO_PRESENCE);
    delay(RW_SETTLE_MS);
    return _done(OWResult::OK);
}

OWResult OneWireTag::verify()
{
    uint8_t expected[OW_ROM_BYTES];
    memcpy(expected, rom, OW_ROM_BYTES);
    const OWResult r = readRaw();
    if (r != OWResult::OK) return r;
    if (memcmp(rom, expected, OW_ROM_BYTES) != 0) return _done(OWResult::VERIFY_FAIL);
    return _done(OWResult::OK);
}

OWResult OneWireTag::program()
{
    if (rom[7] == 0x00u) rom[7] = OneWireBus::crc8(rom, 7);
    if (OneWireBus::crc8(rom, 7) != rom[7]) return _done(OWResult::CRC_ERROR);
    const OWResult wr = write();
    if (wr != OWResult::OK) return wr;
    return verify();
}

/* ── Static utilities ───────────────────────────────────────────────────── */

bool OneWireTag::isCompatible(const uint8_t* romCode)
{
    return (romCode[0] == OW_FAMILY_DS1990);
}

void OneWireTag::buildRomCode(const uint8_t* serial6, uint8_t* romCode)
{
    romCode[0] = OW_FAMILY_DS1990;
    memcpy(&romCode[1], serial6, 6);
    romCode[7] = OneWireBus::crc8(romCode, 7);
}

/* ── RW1990 extended-pulse bit write ────────────────────────────────────── */

void OneWireTag::_writeBitRW(uint8_t bit)
{
    /* 6–12 ms pulses: no µs-level timing sensitivity; Arduino GPIO is fine.
     * Bus release relies on the external 2 kΩ pullup (see file header note). */
    const uint8_t p = _bus.pin();
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
    delay(bit ? RW_PULSE_ONE_MS : RW_PULSE_ZERO_MS);
    pinMode(p, INPUT);
    delay(RW_RELEASE_MS);
}

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-06-12  v2 refactor
 *   - Now delegates reset/read/write/select/crc8 to OneWireBus.  File reduced
 *     to iButton-specific logic only.
 *   - _writeBitRW() switched from direct VPORT access (v1) to Arduino GPIO
 *     via _bus.pin().  The 6–12 ms RW1990 programming pulses have no
 *     µs-level timing sensitivity so function-call overhead is irrelevant.
 *     See file header for the pullup dependency note.
 *   - write() now checks the post-write reset return value (was unchecked
 *     in v1 New1W draft).
 *
 * 2026-06-12  v1 (monolithic, pre-refactor — recoverable via git tag v1.0)
 *   - Extended-pulse RW1990 programming using direct VPORT access for all
 *     operations including _writeBitRW.
 */
