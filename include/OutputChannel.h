/**
 * OutputChannel.h  —  Multi-channel non-blocking output sequencer
 *                     for ATtiny3224 / megaTinyCore 2.6.x
 *
 * Supports PWM outputs (LED, motor speed) via analogWrite() and digital
 * outputs (relay, solenoid) via digitalWrite(), with configurable per-channel
 * min/max levels and the same 4-stage sequence for both modes:
 *   FADE_IN → FULL_ON → FADE_OUT → FULL_OFF → repeat/stop
 *
 * USAGE
 *   static const uint8_t pins[] = { PIN_PA5, PIN_PA3, PIN_PA2 };
 *   OutputChannel<3> out(pins);
 *
 *   // ch0: default PWM 0–255, breathing LED
 *   out.configure(0, OUT_INFINITE, 1, 2, 1, 0);
 *
 *   // ch1: motor, PWM range 50–200, 3 cycles
 *   out.setOutputMode(1, false, 50, 200);
 *   out.configure(1, 3, 2, 5, 2, 1);
 *
 *   // ch2: relay, digital, 4 pulses of 2 s on / 2 s off
 *   out.setOutputMode(2, true, 0, 1);
 *   out.configureDigital(2, 4, 2, 2);
 *
 *   // in loop():
 *   out.update();
 *
 * PWM NOTE
 * ────────
 * Hardware PWM is driven via analogWrite() which uses TCA0 split mode.
 * On the 14-pin ATtiny3224, PA5 = WO5 and PA3 = WO3 (alternate routing).
 * millis() MUST be assigned to TCB0 or TCB1 — NOT TCA0.
 */

#pragma once
#include <Arduino.h>

/* ── Sentinel values for the 'repeats' parameter ────────────────────────── */
static constexpr uint16_t OUT_STOP    = 0x0000u;  ///< Stop and drive to levelLow
static constexpr uint16_t OUT_INFINITE = 0xFFFFu; ///< Run indefinitely

/* ── Stage identifiers ───────────────────────────────────────────────────── */
/**
 * IDLE is last (= 4) so the four active stages form a zero-based ring:
 *   FADE_IN(0) → FULL_ON(1) → FADE_OUT(2) → FULL_OFF(3) → FADE_IN(0) …
 *
 * Benefits:
 *   - Stage value == durationMs[] index; no offset arithmetic.
 *   - _nextStage(s) = (s+1) & 3 — two instructions.
 *   - start() loop: for s = 0; s < 4 — natural zero-based.
 *
 * NOTE: zero-initialising an OutChannel gives stage = FADE_IN (0), not IDLE.
 *       constructor and stop() both explicitly set stage = OutStage::IDLE.
 */
enum class OutStage : uint8_t {
    FADE_IN  = 0,
    FULL_ON  = 1,
    FADE_OUT = 2,
    FULL_OFF = 3,
    IDLE     = 4,
};

/* ── Per-channel state ───────────────────────────────────────────────────── */
/**
 * All per-channel configuration and runtime state in one struct.
 *
 * durationMs[4] is indexed directly by (uint8_t)OutStage 0–3:
 *   [0] FADE_IN  [1] FULL_ON  [2] FADE_OUT  [3] FULL_OFF
 *
 * digital  false = analogWrite (PWM), true = digitalWrite.
 *          Zero-init gives false (PWM) — safe default.
 *          In digital mode FADE_IN/FADE_OUT write levelHigh/levelLow at stage
 *          entry and hold; no progressive updates during the stage duration.
 *
 * levelLow / levelHigh  Output values for the OFF and ON boundaries.
 *   Default: 0 / 255 (standard LED PWM).
 *   Relay:   0 / 1   (digital, set via setOutputMode).
 *   Motor:  50 / 200 (PWM speed range, set via setOutputMode).
 *   Zero-init gives levelHigh = 0; constructor must set it to 255.
 */
struct OutChannel {
    /* [config] */
    uint8_t  pin;
    uint8_t  levelLow;       ///< output level for OFF boundary (default 0)
    uint8_t  levelHigh;      ///< output level for ON  boundary (default 255)
    bool     digital;        ///< false = analogWrite (PWM), true = digitalWrite
    uint16_t repeats;
    uint32_t durationMs[4];  ///< indexed by OutStage 0–3

    /* [runtime] */
    OutStage stage;
    uint32_t stageStartMs;
    uint16_t cyclesDone;
    bool     running;
};

/* ── OutputChannel template ──────────────────────────────────────────────── */
template<uint8_t N_CH>
class OutputChannel {
    static_assert(N_CH > 0, "OutputChannel: N_CH must be at least 1");

public:
    /**
     * Construct and initialise all channels.
     * @param pins  C-array of Arduino pin numbers, length exactly N_CH.
     *              PWM channels must support analogWrite (hardware PWM pin).
     *              Digital channels may be any OUTPUT-capable pin.
     */
    explicit OutputChannel(const uint8_t (&pins)[N_CH])
    {
        for (uint8_t i = 0; i < N_CH; ++i) {
            _ch[i]           = {};
            _ch[i].pin       = pins[i];
            _ch[i].levelHigh = 255;            // not zero-init safe
            _ch[i].stage     = OutStage::IDLE; // 0 = FADE_IN; must be explicit
            // levelLow=0, digital=false, running=false correct from zero-init
            pinMode(pins[i], OUTPUT);
            digitalWrite(pins[i], LOW);        // safe for all pin types; avoids TCA0 WO on digital pins
        }
    }

    /* ── Configuration & control ─────────────────────────────────────────── */

    /**
     * Set output mode and level bounds for one channel.
     * Call once after construction, before configure(), when non-default
     * values are needed.  Safe to call while running; takes effect at the
     * next _enterStage boundary.
     *
     * @param ch        Channel index (0 … N_CH-1).
     * @param digital   false = analogWrite (PWM), true = digitalWrite.
     * @param levelLow  Output level for the OFF boundary (default 0).
     * @param levelHigh Output level for the ON  boundary (default 255).
     */
    void setOutputMode(uint8_t ch, bool digital,
                       uint8_t levelLow = 0, uint8_t levelHigh = 255)
    {
        if (ch >= N_CH) return;
        _ch[ch].digital   = digital;
        _ch[ch].levelLow  = levelLow;
        _ch[ch].levelHigh = levelHigh;
    }

    /**
     * Configure and immediately start (or stop) one channel.
     *
     * @param ch         Channel index (0 … N_CH-1).
     * @param repeats    OUT_STOP = drive to levelLow.
     *                   OUT_INFINITE = run forever.
     *                   1–65534 = exact cycle count.
     * @param fadeInSec  0 = skip stage; 1–65535 = whole seconds.
     * @param onSec      0 = skip stage; 1–65535 = whole seconds.
     * @param fadeOutSec 0 = skip stage; 1–65535 = whole seconds.
     * @param offSec     0 = skip stage; 1–65535 = whole seconds.
     */
    void configure(uint8_t  ch,
                   uint16_t repeats,
                   uint16_t fadeInSec,
                   uint16_t onSec,
                   uint16_t fadeOutSec,
                   uint16_t offSec)
    {
        if (ch >= N_CH) return;
        _ch[ch].repeats       = repeats;
        _ch[ch].durationMs[0] = (uint32_t)fadeInSec  * 1000UL;
        _ch[ch].durationMs[1] = (uint32_t)onSec      * 1000UL;
        _ch[ch].durationMs[2] = (uint32_t)fadeOutSec * 1000UL;
        _ch[ch].durationMs[3] = (uint32_t)offSec     * 1000UL;
        (repeats == OUT_STOP) ? stop(ch) : start(ch);
    }

    /**
     * Configure a digital-mode channel; always zeroes the fade stages.
     *
     * Permanent ON:      configureDigital(ch, OUT_INFINITE, 65535, 0)
     *   offSec=0 means FULL_OFF is skipped in _advance — pin never written LOW
     *   between cycles.  levelHigh re-asserted every 65535 s (harmless).
     * Timed N pulses:    configureDigital(ch, N, onSec, offSec)
     * Off immediately:   configureDigital(ch, OUT_STOP, 0, 0)
     */
    void configureDigital(uint8_t ch, uint16_t repeats, uint16_t onSec, uint16_t offSec)
    {
        configure(ch, repeats, 0, onSec, 0, offSec);
    }

    /** Restart a channel from its first non-zero stage. */
    void start(uint8_t ch)
    {
        if (ch >= N_CH || _ch[ch].repeats == OUT_STOP) return;
        _ch[ch].cyclesDone = 0;
        _ch[ch].running    = true;

        /* Scan FADE_IN(0)..FULL_OFF(3); enter the first stage with duration > 0. */
        for (uint8_t s = 0; s < 4u; ++s) {
            if (_ch[ch].durationMs[s] > 0) {
                _enterStage(ch, (OutStage)s);
                return;
            }
        }
        stop(ch);   /* all durations zero */
    }

    /** Stop one channel immediately and drive it to levelLow. */
    void stop(uint8_t ch)
    {
        if (ch >= N_CH) return;
        _ch[ch].running = false;
        _ch[ch].stage   = OutStage::IDLE;
        _writePin(ch, _ch[ch].levelLow);
    }

    void stopAll() { for (uint8_t i = 0; i < N_CH; ++i) stop(i); }

    /** Call from loop() as often as possible; drives all channel state machines. */
    void update() { for (uint8_t i = 0; i < N_CH; ++i) _updateChannel(i); }

    /* ── Accessors ───────────────────────────────────────────────────────── */

    bool     isRunning(uint8_t ch)     const { return ch < N_CH && _ch[ch].running; }
    bool     isIdle(uint8_t ch)        const { return ch < N_CH && _ch[ch].stage == OutStage::IDLE; }
    OutStage getStage(uint8_t ch)      const { return ch < N_CH ? _ch[ch].stage : OutStage::IDLE; }
    uint16_t getCycleCount(uint8_t ch) const { return ch < N_CH ? _ch[ch].cyclesDone : 0u; }

    static constexpr uint8_t channelCount() { return N_CH; }
    const OutChannel& channel(uint8_t ch) const { return _ch[ch < N_CH ? ch : 0]; }

    /* ── Debug output ────────────────────────────────────────────────────── */

    /** Buffer size (bytes incl. null) sufficient for formatChannel(). */
    static constexpr uint8_t OUT_STATUS_BUF = 112u;

    /**
     * Format one channel's state as a single line into buf using snprintf.
     * No Serial I/O; caller owns transmission and REDE control.
     *
     * Format:
     *   ch<N> p<pin> md=<P|D> lo=<n> hi=<n> <stg> <el>/<dur>s lv=<n> cy=<n>/<reps>
     *   in=<s>s on=<s>s fo=<s>s off=<s>s
     *
     * Stage abbreviations: FI  ON  FO  OFF  IDLE
     * md: P = PWM (analogWrite), D = Digital (digitalWrite)
     *
     * @return Characters written excl. null, or <0 on error.
     */
    int formatChannel(uint8_t ch, char* buf, uint8_t len) const
    {
        if (ch >= N_CH || !buf || len == 0) return -1;
        const OutChannel& c = _ch[ch];

        char rstr[7];
        if      (c.repeats == OUT_INFINITE) snprintf(rstr, sizeof(rstr), "INF");
        else if (c.repeats == OUT_STOP)     snprintf(rstr, sizeof(rstr), "STP");
        else                                snprintf(rstr, sizeof(rstr), "%u", c.repeats);

        static const char* const kAbbrev[] = {"FI", "ON", "FO", "OFF", "IDLE"};
        const char*    stg      = kAbbrev[(uint8_t)c.stage];
        const uint8_t  si       = (uint8_t)c.stage < 4u ? (uint8_t)c.stage : 0u;
        const uint32_t elapsed  = c.running ? millis() - c.stageStartMs : 0UL;
        const uint32_t duration = c.running ? c.durationMs[si]          : 0UL;
        const uint8_t  lv       = _currentLevel(c, elapsed, duration);

        return snprintf(buf, len,
            "ch%u p%u md=%c lo=%u hi=%u %s %lu/%lus lv=%u cy=%u/%s in=%lus on=%lus fo=%lus off=%lus\n",
            ch, c.pin, c.digital ? 'D' : 'P', c.levelLow, c.levelHigh,
            stg, elapsed/1000UL, duration/1000UL, lv,
            c.cyclesDone, rstr,
            c.durationMs[0]/1000UL, c.durationMs[1]/1000UL,
            c.durationMs[2]/1000UL, c.durationMs[3]/1000UL);
    }

    /**
     * Printf-style output handler type.
     * Matches the signature of serialPrintF(const char* fmt, ...).
     */
    using PrintFHandler = void(*)(const char* fmt, ...);

    /**
     * Format and output one channel's status by calling handler(fmt, args...).
     * No buffer allocated in OutputChannel; only rstr[7] is local (feeds %s).
     */
    void printChannelF(uint8_t ch, PrintFHandler handler) const
    {
        if (ch >= N_CH || !handler) return;
        const OutChannel& c = _ch[ch];

        char rstr[7];
        if      (c.repeats == OUT_INFINITE) snprintf(rstr, sizeof(rstr), "INF");
        else if (c.repeats == OUT_STOP)     snprintf(rstr, sizeof(rstr), "STP");
        else                                snprintf(rstr, sizeof(rstr), "%u", c.repeats);

        static const char* const kAbbrev[] = {"FI", "ON", "FO", "OFF", "IDLE"};
        const char*    stg      = kAbbrev[(uint8_t)c.stage];
        const uint8_t  si       = (uint8_t)c.stage < 4u ? (uint8_t)c.stage : 0u;
        const uint32_t elapsed  = c.running ? millis() - c.stageStartMs : 0UL;
        const uint32_t duration = c.running ? c.durationMs[si]          : 0UL;
        const uint8_t  lv       = _currentLevel(c, elapsed, duration);

        handler(
            "ch%u p%u md=%c lo=%u hi=%u %s %lu/%lus lv=%u cy=%u/%s in=%lus on=%lus fo=%lus off=%lus\n",
            (unsigned)ch, (unsigned)c.pin, c.digital ? 'D' : 'P',
            (unsigned)c.levelLow, (unsigned)c.levelHigh,
            stg, elapsed/1000UL, duration/1000UL, (unsigned)lv,
            (unsigned)c.cyclesDone, rstr,
            c.durationMs[0]/1000UL, c.durationMs[1]/1000UL,
            c.durationMs[2]/1000UL, c.durationMs[3]/1000UL);
    }

    /** Output all channels, one handler call per channel. */
    void printAllF(PrintFHandler handler) const
    {
        for (uint8_t i = 0; i < N_CH; ++i) printChannelF(i, handler);
    }

private:
    OutChannel _ch[N_CH];

    /* ── Pin I/O ─────────────────────────────────────────────────────────── */

    void _writePin(uint8_t ch, uint8_t level)
    {
        if (_ch[ch].digital) digitalWrite(_ch[ch].pin, level ? HIGH : LOW);
        else                 analogWrite(_ch[ch].pin, level);
    }

    /* ── State machine ───────────────────────────────────────────────────── */

    void _updateChannel(uint8_t ch)
    {
        if (!_ch[ch].running) return;

        const OutStage s = _ch[ch].stage;
        if (s == OutStage::IDLE) return;

        const uint32_t elapsed  = millis() - _ch[ch].stageStartMs;
        const uint32_t duration = _ch[ch].durationMs[(uint8_t)s];

        if (elapsed >= duration) { _advance(ch); return; }

        /* Digital: boundary level held by _enterStage; no progressive writes. */
        if (_ch[ch].digital) return;

        if      (s == OutStage::FADE_IN)
            _writePin(ch, _lerp(elapsed, duration, true,  _ch[ch].levelLow, _ch[ch].levelHigh));
        else if (s == OutStage::FADE_OUT)
            _writePin(ch, _lerp(elapsed, duration, false, _ch[ch].levelLow, _ch[ch].levelHigh));
    }

    void _enterStage(uint8_t ch, OutStage s)
    {
        _ch[ch].stage        = s;
        _ch[ch].stageStartMs = millis();
        /* FULL_ON and FADE_OUT start at levelHigh; all others at levelLow. */
        const uint8_t level = (s == OutStage::FULL_ON || s == OutStage::FADE_OUT)
                              ? _ch[ch].levelHigh : _ch[ch].levelLow;
        _writePin(ch, level);
    }

    /** Next stage in the ring: (s+1) & 3 wraps FULL_OFF(3) → FADE_IN(0). */
    static OutStage _nextStage(OutStage s)
    {
        return (OutStage)(((uint8_t)s + 1u) & 3u);
    }

    /**
     * Walk forward through stages, skipping any with zero duration, until a
     * runnable stage is found or all repeats are exhausted.
     * Bounded by a guard counter; stops if all four durations are zero.
     */
    void _advance(uint8_t ch)
    {
        OutStage s = _ch[ch].stage;

        for (uint8_t guard = 0; guard < 8u; ++guard) {

            if (s == OutStage::FULL_OFF) {
                ++_ch[ch].cyclesDone;
                const bool infinite = (_ch[ch].repeats == OUT_INFINITE);
                if (!infinite && _ch[ch].cyclesDone >= _ch[ch].repeats) {
                    stop(ch);
                    return;
                }
            }

            s = _nextStage(s);

            if (_ch[ch].durationMs[(uint8_t)s] > 0) {
                _enterStage(ch, s);
                return;
            }
        }

        stop(ch);   /* all four durations zero */
    }

    /* ── Helpers ─────────────────────────────────────────────────────────── */

    /** Instantaneous output level for display — does not write to pin. */
    static uint8_t _currentLevel(const OutChannel& c, uint32_t elapsed, uint32_t duration)
    {
        if (c.stage == OutStage::FULL_ON)  return c.levelHigh;
        if (c.stage == OutStage::FADE_IN)  return duration ? _lerp(elapsed, duration, true,  c.levelLow, c.levelHigh) : c.levelLow;
        if (c.stage == OutStage::FADE_OUT) return duration ? _lerp(elapsed, duration, false, c.levelLow, c.levelHigh) : c.levelLow;
        return c.levelLow;   /* FULL_OFF or IDLE */
    }

    /**
     * Linear interpolation between lo and hi over [0, duration].
     * rising true  → lo…hi (fade in).
     * rising false → hi…lo (fade out).
     * Preconditions: duration > 0, lo <= hi.
     */
    static uint8_t _lerp(uint32_t elapsed, uint32_t duration, bool rising, uint8_t lo, uint8_t hi)
    {
        const uint32_t span = (uint32_t)(hi - lo);
        uint32_t frac = (elapsed * span) / duration;
        if (frac > span) frac = span;
        return rising ? (uint8_t)(lo + frac) : (uint8_t)(hi - frac);
    }
};

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG  (complete history — StatusLED.h → StatusLED2.h → OutputChannel.h)
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-06-12  (2)
 *   - configureDigital(ch, repeats, onSec, offSec): convenience wrapper that
 *     always zeroes fade stages, making digital call-sites self-documenting.
 *     Permanent ON = (OUT_INFINITE, 65535, 0): FULL_OFF has duration=0 so
 *     _advance skips it without calling _enterStage; pin never written LOW
 *     between cycles.  levelHigh re-asserted harmlessly every 65535 s.
 *
 * 2026-06-12  (1)  —  Renamed and extended from StatusLED2.h (revision 9).
 *   Renames:
 *     StatusLED<N_CH>   → OutputChannel<N_CH>
 *     LEDChannel        → OutChannel
 *     LEDStage          → OutStage
 *     LED_STOP/INFINITE → OUT_STOP/INFINITE
 *     LED_STATUS_BUF    → OUT_STATUS_BUF  (96 → 112 for new fields)
 *     pw= format token  → lv= (generic output level)
 *   New — bool digital field in OutChannel:
 *     false = analogWrite (PWM) — prior behaviour; zero-init safe default.
 *     true  = digitalWrite — _enterStage writes levelHigh/levelLow at stage
 *             entry and holds; no progressive writes during the stage.
 *     setOutputMode(ch, digital, levelLow=0, levelHigh=255) configures both.
 *   New — uint8_t levelLow / levelHigh fields in OutChannel:
 *     levelLow  = output value for the OFF boundary (default 0).
 *     levelHigh = output value for the ON  boundary (default 255).
 *     _lerp interpolates between lo…hi instead of 0…255.
 *     _enterStage uses levelHigh/levelLow instead of 255/0 literals.
 *     stop() drives levelLow instead of hardcoded 0.
 *   New — _writePin(ch, level) private helper; routes to digitalWrite or
 *     analogWrite depending on digital flag; replaces all direct analogWrite
 *     calls in the state machine.
 *   New — _currentLevel(c, elapsed, duration) replaces _currentPWM; takes
 *     full OutChannel& to access levelLow/levelHigh.
 *   Constructor: analogWrite(pin, 0) → digitalWrite(pin, LOW).
 *     analogWrite would activate TCA0 WO on digital-mode pins (e.g. relay on
 *     PA2) before setOutputMode is called.  digitalWrite(LOW) is safe for all
 *     pin types; the first analogWrite comes from _enterStage when a PWM
 *     channel starts.
 *
 * ── History inherited from StatusLED2.h ─────────────────────────────────
 *
 * 2026-05-29  (9)
 *   - Decoupled from any specific output function.
 *   - Removed: extern serialWrite declaration, printChannel(), printAll().
 *   - Added PrintFHandler type alias: void(*)(const char* fmt, ...).
 *   - Added printChannelF(ch, handler): passes format string and all args
 *     directly to the handler — no buffer in the class.  Only rstr[7] is
 *     local (unavoidable: feeds %s in the format).  Handler owns buffer
 *     allocation, vsnprintf, and RS485 transmission.
 *   - Added printAllF(handler): calls printChannelF() for each channel.
 *
 * 2026-05-29  (8)
 *   - formatChannel(): unified to a single format string for all states.
 *     kAbbrev extended to {"FI","ON","FO","OFF","IDLE"} — IDLE is just another
 *     abbreviation; no special-case branch needed.
 *     IDLE shows el=0 dur=0 pw=0 (consistent column layout).
 *     durationMs[] out-of-bounds risk (IDLE=4, array 0–3) resolved by
 *     clamping si = stage < 4 ? stage : 0 before the array access.
 *
 * 2026-05-29  (7)
 *   - Removed "RUN" token from the running format string in formatChannel().
 *     Any non-IDLE stage implies the channel is running; the token was
 *     redundant.  Saves 4 bytes per line.
 *
 * 2026-05-29  (6)
 *   - printChannel/printAll now call serialWrite() instead of managing REDE
 *     and Serial directly; redePin parameter removed from both.
 *   - _transmit() private helper removed (was the only REDE/Serial site).
 *   - extern void serialWrite(const char*) forward-declared in the header.
 *
 * 2026-05-29  (5)  ← major restructure
 *   - LEDStage reordered: FADE_IN=0, FULL_ON=1, FADE_OUT=2, FULL_OFF=3,
 *     IDLE=4.  Active stages form a zero-based ring; IDLE sits outside it.
 *   - LEDChannel: four named duration fields (fadeInMs/onMs/fadeOutMs/offMs)
 *     replaced by durationMs[4], indexed directly by (uint8_t)stage.
 *     Eliminates the s-1 offset that was required throughout.
 *   - _nextStage(s): (s+1) & 3 — 2 instructions, no jump table.
 *   - _updateChannel(): switch eliminated; one elapsed>=durationMs[s] check
 *     covers all active stages; only FADE_IN/FADE_OUT write progressive PWM.
 *   - _enterStage(): 5-case switch replaced by single ternary
 *     (FULL_ON || FADE_OUT ? 255 : 0).
 *   - start() loop: for s=0; s<4 — natural zero-based, no enum offset.
 *   - _advance(): _stageDuration() helper inlined as direct durationMs[s].
 *   - formatChannel(): stage abbreviations via kAbbrev[s] local static array.
 *   - _currentPWM(): signature changed to (LEDStage, elapsed, duration).
 *   - Zero-init note: stage=0 now means FADE_IN, not IDLE; constructor and
 *     stop() both explicitly set stage = LEDStage::IDLE.
 *
 * 2026-05-29  (4)
 *   - Rewrote printChannel/printAll to use snprintf (RS485 compatible).
 *   - formatChannel(ch, buf, len) builds a complete status line; no Serial I/O.
 *   - printChannel/printAll assert REDE, transmit in one burst, release REDE.
 *   - Removed _printStage() and _printPadded() (replaced by snprintf).
 *   - Added LED_STATUS_BUF = 96 constant for caller buffer sizing.
 *
 * 2026-05-29  (3)
 *   - Added formatChannel() / printChannel() / printAll() debug functions.
 *   - Private helpers: _printStage(), _printPadded(), _stageDurationOf(),
 *     _currentPWM().
 *
 * 2026-05-29  (2)
 *   - start(): replaced {FADE_IN..FULL_OFF} lookup array with direct
 *     enum-value iteration.  Removes a redundant array; saves 4 bytes stack.
 *
 * 2026-05-29  (1)
 *   - State machine reworked to skip zero-duration stages atomically:
 *     _advance() walks forward in one bounded loop without intermediate pin
 *     writes — eliminates the glitch-to-zero between repeats for non-LED
 *     loads (e.g. DC motors).
 *   - start() enters the first non-zero stage directly (no startup glitch
 *     for configs such as in=0, on=10s).
 *   - All-zero configurations call stop() rather than spinning; guard counter
 *     in _advance() prevents unbounded loops.
 *
 * ── Undated pre-history (StatusLED.h development) ───────────────────────
 *
 *   - Converted to template class StatusLED<N_CH>; all per-channel state
 *     consolidated in LEDChannel struct.  Channel count fixed at compile
 *     time; no dynamic allocation.  Implementation moved to header (required
 *     for C++ templates).
 *   - configure() API: (channel, repeats, fadeInSec, onSec, fadeOutSec,
 *     offSec).  repeats: 0=stop, 65535=infinite, else exact count.  Stage
 *     durations taken as uint16_t seconds, stored as uint32_t ms for
 *     millis() arithmetic (65535 s x 1000 > uint16_t).
 *   - Original single-channel version: hardware PWM via analogWrite (TCA0
 *     split mode, WO5->PA5), loop()-driven (no ISR), 5-stage sequence
 *     FADE_IN -> FULL_ON -> FADE_OUT -> FULL_OFF -> repeat/idle.
 */
