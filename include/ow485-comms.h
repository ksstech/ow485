// ow485-comms.h — RS485 command input, echo, VT100 editing, history
// Copyright (c) 2026 Andre M. Maree / KSS Technologies (Pty) Ltd.
//
// Command input, optional echo, optional VT100 line editing, optional history.
// All state is pre-allocated; no heap usage.
//
// Feature bits live in DEBUG_LEVEL (defined in ow485.cpp before this include):
//   DEBUG_CMD_ECHO    0x0020  character echo + RS485 flush via hostFlushEcho()
//   DEBUG_CMD_EDITING 0x0040  VT100 in-place editing: cursor, insert/overwrite,
//                             BS/DEL, Home, End, Delete  (requires DEBUG_CMD_ECHO)
//   DEBUG_CMD_HISTORY 0x0080  8-deep history ring, dedup, Up/Down recall
//                             (requires DEBUG_CMD_ECHO + DEBUG_CMD_EDITING)
//
// Static SRAM by active bits:
//   0x0000  _edit[33] + _editLen + _dispPos + _hadCR                    ~36 B
//   +ECHO   + _echoBuf[80] + _echoBufLen                               ~117 B
//   +EDIT   + _editCursor + _escState + _csiParam                      ~120 B
//   +HIST   + _hist[8][33] + _histHead + _histCount + _histIdx         ~387 B
//
// Terminal sequences understood when DEBUG_CMD_EDITING is active:
//   ESC [ A   Up arrow    — recall older history entry   (DEBUG_CMD_HISTORY)
//   ESC [ B   Down arrow  — recall newer / clear         (DEBUG_CMD_HISTORY)
//   ESC [ C   Right arrow — cursor right
//   ESC [ D   Left arrow  — cursor left
//   ESC [ H   Home        — cursor to start of line
//   ESC [ F   End         — cursor to end of line
//   ESC [ 3 ~ Delete      — delete character at cursor
//   DEL / BS  (0x7f/0x08) — delete character before cursor
//
// Note: without DEBUG_CMD_EDITING, ESC sequences are not parsed.

// Provide self-contained defaults so the header can be included standalone.
#ifndef DEBUG_CMD_ECHO
#  define DEBUG_CMD_ECHO      0x0020
#  define DEBUG_CMD_EDITING   0x0040
#  define DEBUG_CMD_HISTORY   0x0080
#endif
#ifndef DEBUG_LEVEL
#  define DEBUG_LEVEL  (DEBUG_CMD_ECHO | DEBUG_CMD_EDITING | DEBUG_CMD_HISTORY)
#endif

static constexpr uint8_t CMD_MAX  = 33;    // 32 data bytes + NUL terminator
#if (DEBUG_LEVEL & DEBUG_CMD_HISTORY)
static constexpr uint8_t CMD_HIST = 8;     // history ring depth
#endif

/* ── Static state ───────────────────────────────────────────────────────── */

#if (DEBUG_LEVEL & DEBUG_CMD_HISTORY)
static char    _hist[CMD_HIST][CMD_MAX];   // history ring buffer
#endif
static char    _edit[CMD_MAX];             // current edit line (no NUL during edit)
#if (DEBUG_LEVEL & DEBUG_CMD_ECHO)
static char    _echoBuf[80];               // echo queue — flushed once per loop()
static uint8_t _echoBufLen;
#endif
static uint8_t _editLen;
#if (DEBUG_LEVEL & DEBUG_CMD_EDITING)
static uint8_t _editCursor;               // insert position 0.._editLen
static uint8_t _escState;                 // 0=NORMAL 1=ESC 2=CSI
static uint8_t _csiParam;                 // accumulated CSI numeric parameter
#endif
static uint8_t _dispPos;                  // consume position for hostConsumeXxx()
#if (DEBUG_LEVEL & DEBUG_CMD_HISTORY)
static uint8_t _histHead;                 // ring index of next write slot
static uint8_t _histCount;               // valid entries 0..CMD_HIST
static int8_t  _histIdx;                  // -1=new input; 0=newest; 1=older…
#endif
static bool    _hadCR;                    // CRLF suppression

/* ── Echo-buffer helpers ────────────────────────────────────────────────── */

#if (DEBUG_LEVEL & DEBUG_CMD_ECHO)
static void _txChar(char c) {
    if (_echoBufLen < sizeof(_echoBuf))
        _echoBuf[_echoBufLen++] = c;
}

static void _txStr(const char* s) {
    while (*s && _echoBufLen < sizeof(_echoBuf))
        _echoBuf[_echoBufLen++] = *s++;
}
#endif  // DEBUG_CMD_ECHO

/* Redraw present only when echo + at least one feature that moves the cursor. */
#if (DEBUG_LEVEL & (DEBUG_CMD_EDITING | DEBUG_CMD_HISTORY))
/* ESC[G (CHA col-1) instead of bare CR: immune to terminal CR→CRLF translation. */
static void _redraw(void) {
    _txStr("\x1b[G");
    if (_editLen) {
        uint8_t avail = sizeof(_echoBuf) - _echoBufLen;
        uint8_t n = (_editLen < avail) ? _editLen : avail;
        memcpy(_echoBuf + _echoBufLen, _edit, n);
        _echoBufLen += n;
    }
    _txStr("\x1b[K");
    if (_editCursor < _editLen) {
        char seq[7];
        snprintf(seq, sizeof(seq), "\x1b[%uD", (unsigned)(_editLen - _editCursor));
        _txStr(seq);
    }
}
#endif  // DEBUG_CMD_EDITING | DEBUG_CMD_HISTORY

/* ── Public interface ───────────────────────────────────────────────────── */

const char* hostCmdStr(void) {
    return (_dispPos <= _editLen) ? _edit + _dispPos : _edit + _editLen;
}

uint8_t hostCmdLen(void) {
    return (_editLen > _dispPos) ? _editLen - _dispPos : 0;
}

#if (DEBUG_LEVEL & DEBUG_CMD_ECHO)
void hostFlushEcho(void) {
    if (!_echoBufLen) return;
    rs485Status(true);
    Serial.write((const uint8_t*)_echoBuf, _echoBufLen);
    Serial.flush();
    _echoBufLen = 0;
    rs485Status(false);
}
#endif

void hostCmdReset(void) {
    _editLen = 0;
    _dispPos = 0;
#if (DEBUG_LEVEL & DEBUG_CMD_EDITING)
    _editCursor = 0;
#endif
}

/* ── Consume functions ──────────────────────────────────────────────────── */

uint8_t convertUpperToHex(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    return 0xFF;
}

char hostConsumeChar(void) {
    return (_dispPos < _editLen) ? _edit[_dispPos++] : (char)-1;
}

char hostConsumeHexChar(void) { return convertUpperToHex(hostConsumeChar()); }

long int hostConsumeHexValue(int8_t Num) {
    long int iVal = 0;
    int8_t Idx = 0;
    do {
        char cChr = hostConsumeChar();
        if (cChr == ',' || cChr == (char)-1) break;
        uint8_t u8Val = convertUpperToHex(cChr);
        if (u8Val == 0xFF) return -1;
        iVal <<= 4;
        iVal += u8Val;
    } while (++Idx < Num);
    if (Idx == Num) {
        if (_dispPos < _editLen && _edit[_dispPos] == ',')
            ++_dispPos;
    }
    return (Idx == 0 || Idx > Num) ? -1L : iVal;
}

uint8_t hostConsumeString(char* Buf, uint8_t u8S) {
    uint8_t n     = u8S - 1;
    uint8_t avail = (_editLen > _dispPos) ? _editLen - _dispPos : 0;
    if (n > avail) n = avail;
    memcpy(Buf, _edit + _dispPos, n);
    Buf[n]  = '\0';
    _dispPos += n;
    if (_dispPos < _editLen && _edit[_dispPos] == ',')
        ++_dispPos;
    return n;
}

/* ── hostReadCmd ────────────────────────────────────────────────────────── */

bool hostReadCmd(void) {
    while (Serial.available()) {
        char c = (char)Serial.read();

#if (DEBUG_LEVEL & DEBUG_CMD_EDITING)
        switch (_escState) {

        /* ── Normal input ──────────────────────────────────────────────── */
        case 0:
#endif

            if (c == '\x1b') {
#if (DEBUG_LEVEL & DEBUG_CMD_EDITING)
                _escState = 1;
#endif

            } else if (c == '\r' || c == '\n') {
                bool suppress = (c == '\n' && _hadCR);
                _hadCR = (c == '\r');
#if (DEBUG_LEVEL & DEBUG_CMD_EDITING)
                _escState = 0;
#endif
                if (!suppress && _editLen > 0) {
                    _edit[_editLen] = '\0';
                    for (uint8_t i = 0; i < _editLen; ++i)
                        _edit[i] = (char)toupper((unsigned char)_edit[i]);
#if (DEBUG_LEVEL & DEBUG_CMD_HISTORY)
                    bool isDup = false;
                    for (uint8_t i = 0; i < _histCount; ++i) {
                        if (strcmp(_hist[(_histHead + CMD_HIST - 1 - i) % CMD_HIST], _edit) == 0)
                            { isDup = true; break; }
                    }
                    if (!isDup) {
                        memcpy(_hist[_histHead], _edit, _editLen + 1);
                        _histHead  = (_histHead + 1) % CMD_HIST;
                        if (_histCount < CMD_HIST) ++_histCount;
                    }
                    _histIdx = -1;
#endif
                    _dispPos = 0;
#if (DEBUG_LEVEL & DEBUG_CMD_ECHO)
                    _txStr("\r\n");
#endif
                    return true;
                }

            } else if (c == '\x7f' || c == '\x08') {
#if (DEBUG_LEVEL & DEBUG_CMD_EDITING)
                if (_editCursor > 0) {
                    memmove(_edit + _editCursor - 1,
                            _edit + _editCursor,
                            _editLen - _editCursor);
                    --_editCursor;
                    --_editLen;
                    _redraw();
                }
#else
                if (_editLen > 0) {
                    --_editLen;
#  if (DEBUG_LEVEL & DEBUG_CMD_ECHO)
                    _txStr("\x08 \x08");   /* BS space BS — erase last char on terminal */
#  endif
                }
#endif

            } else if (c >= 0x20 && c < 0x7f) {
#if (DEBUG_LEVEL & DEBUG_CMD_EDITING)
                if (_editLen < CMD_MAX - 1) {
                    if (_editCursor < _editLen) {
                        memmove(_edit + _editCursor + 1,
                                _edit + _editCursor,
                                _editLen - _editCursor);
                        _edit[_editCursor] = c;
                        ++_editCursor;
                        ++_editLen;
                        _redraw();
                    } else {
                        _edit[_editCursor] = c;
                        ++_editCursor;
                        ++_editLen;
#  if (DEBUG_LEVEL & DEBUG_CMD_ECHO)
                        _txChar(c);
#  endif
                    }
                }
#else
                if (_editLen < CMD_MAX - 1) {
                    _edit[_editLen++] = c;
#  if (DEBUG_LEVEL & DEBUG_CMD_ECHO)
                    _txChar(c);
#  endif
                }
#endif
            }

#if (DEBUG_LEVEL & DEBUG_CMD_EDITING)
            break;  /* case 0 */

        /* ── ESC received ─────────────────────────────────────────────── */
        case 1:
            if (c == '[') { _escState = 2; _csiParam = 0; }
            else           { _escState = 0; }
            break;

        /* ── ESC [ received (CSI) ──────────────────────────────────────── */
        case 2:
            if (c >= '0' && c <= '9') {
                _csiParam = _csiParam * 10 + (uint8_t)(c - '0');
                break;
            }
            _escState = 0;

#  if (DEBUG_LEVEL & DEBUG_CMD_HISTORY)
            if (c == 'A') {                              /* up — older */
                if (_histIdx < (int8_t)_histCount - 1) {
                    ++_histIdx;
                    uint8_t e = (_histHead + CMD_HIST - 1 - (uint8_t)_histIdx) % CMD_HIST;
                    uint8_t n = (uint8_t)strlen(_hist[e]);
                    memcpy(_edit, _hist[e], n + 1);
                    _editLen    = n;
                    _editCursor = n;
                    _redraw();
                }
            } else if (c == 'B') {                      /* down — newer / clear */
                if (_histIdx > 0) {
                    --_histIdx;
                    uint8_t e = (_histHead + CMD_HIST - 1 - (uint8_t)_histIdx) % CMD_HIST;
                    uint8_t n = (uint8_t)strlen(_hist[e]);
                    memcpy(_edit, _hist[e], n + 1);
                    _editLen    = n;
                    _editCursor = n;
                    _redraw();
                } else if (_histIdx == 0) {
                    _histIdx    = -1;
                    _editLen    = 0;
                    _editCursor = 0;
                    _redraw();
                }
            } else
#  endif  /* DEBUG_CMD_HISTORY */
            if (c == 'C') {                              /* right */
                if (_editCursor < _editLen) {
                    ++_editCursor;
                    _txStr("\x1b[C");
                }
            } else if (c == 'D') {                       /* left */
                if (_editCursor > 0) {
                    --_editCursor;
                    _txStr("\x1b[D");
                }
            } else if (c == 'H' || (_csiParam == 1 && c == '~')) {   /* Home */
                if (_editCursor > 0) {
                    _editCursor = 0;
                    _redraw();
                }
            } else if (c == 'F' || (_csiParam == 4 && c == '~')) {   /* End */
                if (_editCursor < _editLen) {
                    _editCursor = _editLen;
                    _redraw();
                }
            } else if (_csiParam == 3 && c == '~') {    /* Delete */
                if (_editCursor < _editLen) {
                    memmove(_edit + _editCursor,
                            _edit + _editCursor + 1,
                            _editLen - _editCursor - 1);
                    --_editLen;
                    _redraw();
                }
            }
            break;

        }  /* end switch (_escState) */
#endif  /* DEBUG_CMD_EDITING */

    }
    return false;
}

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-06-12
 *   - New file: command ring buffer, VT100 line editor, RS485 echo via
 *     _echoBuf[80]; CMD_MAX=33
 *   - hostReadCmd / hostFlushEcho / hostCmdReset /
 *     hostConsumeChar/Hex/HexValue/String
 *
 * 2026-06-15
 *   - CMD_LEVEL bits merged into DEBUG_LEVEL (bits 5-7: 0x0020 ECHO,
 *     0x0040 EDITING, 0x0080 HISTORY)
 *   - Backspace redraw uses ESC[G (CHA col-1) instead of bare CR —
 *     immune to terminal CR→CRLF expansion
 *   - History dedup extended from consecutive-only to full-ring search
 *   - hostFlushEcho() call site guarded in loop() — empty stub removed
 */
