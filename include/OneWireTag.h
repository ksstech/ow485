/**
 * OneWireTag.h  —  DS1990 / RW1990 iButton read, write and verify
 *
 * Uses OneWireBus for all standard bus operations.
 * Only the RW1990 extended-pulse write is handled here.
 */

#pragma once
#include "OneWireBus.h"

static constexpr uint8_t OW_FAMILY_DS1990 = 0x01u;

/* ── RW1990 timing constants ────────────────────────────────────────────── */
static constexpr uint8_t  RW_CMD_WRITE     = 0xD1u;
static constexpr uint8_t  RW_CMD_WRITE_ALT = 0xC1u;
static constexpr uint8_t  RW_PULSE_ONE_MS  = 6u;    /* 6 ms — spec says 5 ms; extra margin confirmed on hw */
static constexpr uint8_t  RW_PULSE_ZERO_MS = 12u;   /* 12 ms — spec says 10 ms; extra margin confirmed on hw */
static constexpr uint8_t  RW_RELEASE_MS    = 2u;
static constexpr uint8_t  RW_ENTER_MS      = 10u;
static constexpr uint16_t RW_SETTLE_MS     = 300u;  /* 300 ms conservative settle (spec min 20 ms) */

class OneWireTag {
public:
    explicit OneWireTag(OneWireBus& bus);

    /* ── Public state ──────────────────────────────────────────────────── */
    uint8_t  rom[OW_ROM_BYTES];
    OWResult result;
    uint32_t timestampMs;

    /* ── Read — results go into rom[] ─────────────────────────────────── */
    OWResult readRaw();
    OWResult read();
    OWResult scanNext();
    void     resetSearch();

    /* ── Write / Program — data taken from rom[] ─────────────────────── */
    OWResult write(bool useAltCmd = false);
    OWResult verify();
    OWResult program();

    /* ── Utilities ────────────────────────────────────────────────────── */
    static bool isCompatible(const uint8_t* romCode);
    static void buildRomCode(const uint8_t* serial6, uint8_t* romCode);

private:
    OneWireBus& _bus;
    OWResult _done(OWResult r);
    void     _writeBitRW(uint8_t bit);
};

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-06-12  v2 refactor
 *   - Refactored to take an OneWireBus& reference; all standard bus ops now
 *     delegate to the shared bus.  Only the RW1990 extended-pulse write
 *     (_writeBitRW) remains local.  OWResult / OW_ROM_BYTES now come from
 *     OneWireBus.h.
 *   - Constants reconciled with v1 hardware-tested values:
 *       RW_PULSE_ONE_MS  = 6u    (ref spec 5u, hw-validated margin kept)
 *       RW_PULSE_ZERO_MS = 12u   (ref spec 10u, hw-validated margin kept)
 *       RW_SETTLE_MS     = 300u  (ref spec 20u, conservative value kept; uint16_t)
 *       RW_ENTER_MS      = 10u   (same as ref spec; v1 had a 2u typo, now fixed)
 *   - printRomCode, printResult moved to OneWireBus; printRomInfo moved to
 *     rs485Support.cpp (tagPrintRomInfo).
 *   - program() made parameterless; call write(true) directly to test 0xC1.
 *
 * 2026-06-12  v1 (monolithic, pre-refactor — recoverable via git tag v1.0)
 *   - Owned all bus primitives inline (VPORT direct).
 *   - Parameterless API: read/readRaw/scanNext/write/verify/program all
 *     operate on the public rom[] buffer.  Public state: rom[8] + result
 *     + timestampMs.
 */
