/**
 * OneWireTag.h  —  DS1990 / RW1990 iButton read, write and verify
 *                  for ATtiny3224 / megaTinyCore 2.6.x
 *
 * DIRECT VPORT IMPLEMENTATION — WHY NOT Paul Stoffregen's OneWire LIBRARY
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * The OneWire library uses classic AVR register offset arithmetic:
 *
 *   base = portInputRegister(port)     ← &PINx on classic ATmega
 *   *(base + 0) = PINx   (input)
 *   *(base + 1) = DDRx   (direction)
 *   *(base + 2) = PORTx  (output)
 *
 * On the tinyAVR 2-series (ATtiny3224), the VPORT layout is different:
 *
 *   VPORTA + 0 = DIR      (direction)
 *   VPORTA + 1 = OUT      (output)
 *   VPORTA + 2 = IN       (input)
 *
 * The library's (base+1) hits INTFLAGS instead of DIR, and (base+2) hits
 * the next VPORT's DIR instead of OUT.  Every write slot and direction
 * change goes to the wrong register.  The library never produces a valid
 * reset pulse on PA4 and _ow.reset() always returns false.
 *
 * This was confirmed by diagnostic testing:
 *   - OneWire library: 0/5 resets succeeded, all reads 0xFF
 *   - Direct VPORTA bit-bang: valid ROM 01:4E:BB:2A:04:00:00:91, CRC pass
 *
 * This driver uses VPORTA register access directly for all bus operations.
 * The OneWire library is not included as a dependency.
 *
 * PORT NOTE: All functions reference VPORTA because PA4 is on Port A of
 * the ATtiny3224.  If porting to a PB pin, change VPORTA → VPORTB.
 *
 * SUPPORTED TAGS
 * ──────────────
 *  Family 0x01   DS1990A, DS1990R  (read-only)
 *                RW1990, RW1990.1  (writable, command 0xD1)
 *                RW1990.2          (writable, may need alternate command 0xC1)
 *
 * HARDWARE
 * ────────
 *  PA4 → 1-Wire bus, 4.7 kΩ pullup to VCC (3.3 V) required.
 *
 * IDE SETTINGS
 * ────────────
 *  Clock: 10 MHz internal  (3.3 V → Speed Grade 2, max 10 MHz per DS40002315)
 *  millis()/micros(): any TCB — must NOT be TCA0
 */

#pragma once
#include <Arduino.h>

/* ── Bus constants ──────────────────────────────────────────────────────── */
static constexpr uint8_t OW_ROM_BYTES     = 8u;
static constexpr uint8_t OW_FAMILY_DS1990 = 0x01u;

/* ── RW1990 write constants ─────────────────────────────────────────────── */
static constexpr uint8_t RW_CMD_WRITE     = 0xD1u;
static constexpr uint8_t RW_CMD_WRITE_ALT = 0xC1u;
static constexpr uint8_t RW_PULSE_ONE_MS  = 6u;
static constexpr uint8_t RW_PULSE_ZERO_MS = 12u;
static constexpr uint8_t RW_RELEASE_MS    = 2u;
static constexpr uint8_t RW_ENTER_MS      = 2u;
static constexpr uint16_t RW_SETTLE_MS    = 300u;

/* ── Result codes ───────────────────────────────────────────────────────── */
enum class OWResult : uint8_t {
    OK            = 0,
    NO_PRESENCE   = 1,
    CRC_ERROR     = 2,
    WRONG_FAMILY  = 3,
    VERIFY_FAIL   = 4,
    BUS_ERROR     = 5,
};

/* ── OneWireTag class ───────────────────────────────────────────────────── */
class OneWireTag {
public:
    explicit OneWireTag(uint8_t pin);

    /* ── Public state — read from anywhere, no getters needed ──────────── */
    /**
     * rom[]        The shared ROM buffer.
     *              - After read()/readRaw()/scanNext(): contains the tag's ROM.
     *              - Before write()/program(): caller places the desired ROM here.
     *              - After verify(): contains the tag's actual ROM (for comparison).
     *
     * result       OWResult from the most recent operation.
     *
     * timestampMs  millis() value when the most recent operation completed.
     */
    uint8_t  rom[OW_ROM_BYTES];
    OWResult result;
    uint32_t timestampMs;

    /* ── Read — results go into rom[] ─────────────────────────────────── */
    OWResult readRaw();                         ///< READ ROM → rom[], no family check
    OWResult read();                            ///< READ ROM → rom[], validates family
    OWResult scanNext();                        ///< SEARCH ROM → rom[]
    void     resetSearch();

    /* ── Write / Program — data taken from rom[] ─────────────────────── */
    OWResult write(bool useAltCmd = false);     ///< Burn rom[] to RW1990
    OWResult verify();                          ///< Read tag, compare to rom[]
    OWResult program(bool useAltCmd);           ///< Fill CRC in rom[7], write, verify

    /* ── Utilities ────────────────────────────────────────────────────── */
    static uint8_t crc8(const uint8_t* data, uint8_t len);
    static bool    isCompatible(const uint8_t* romCode);
    static void    buildRomCode(const uint8_t* serial6, uint8_t* romCode);
    static void    printRomCode(const uint8_t* romCode);
    static void    printResult(OWResult r);
    void    printRomInfo(bool reverse);

private:
    volatile uint8_t* _vport;
    uint8_t           _bitmask;

    /** Set result and timestamp in one place. */
    OWResult _done(OWResult r);

    /* ── Low-level bus operations ─────────────────────────────────────── */
    bool    _reset();
    void    _writeBit(uint8_t bit);
    uint8_t _readBit();
    void    _writeByte(uint8_t byte);
    uint8_t _readByte();
    void    _writeBitRW(uint8_t bit);
};
