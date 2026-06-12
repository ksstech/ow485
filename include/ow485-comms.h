// ow485-comms.h
//
// Command input with line editing, history, and immediate echo.
// All state is pre-allocated; no heap usage.
//
// Static allocation breakdown:
//   _hist[CMD_HIST][CMD_MAX]  =  8 × 48 = 384 bytes  (history ring)
//   _edit[CMD_MAX]            =      48 bytes          (edit line)
//   state variables           =     ~12 bytes
//   Total                     =    ~444 bytes
//
// Terminal sequences understood (standard VT100/xterm):
//   ESC [ A   Up arrow    — recall older history entry
//   ESC [ B   Down arrow  — recall newer history entry / clear
//   ESC [ C   Right arrow — move cursor right
//   ESC [ D   Left arrow  — move cursor left
//   ESC [ H   Home        — move cursor to start of line
//   ESC [ F   End         — move cursor to end of line
//   ESC [ 3 ~ Delete      — delete character at cursor
//   DEL / BS  (0x7f/0x08) — delete character before cursor

static constexpr uint8_t CMD_MAX  = 48;   // max command chars including NUL
static constexpr uint8_t CMD_HIST = 8;    // history ring depth

static char    _hist[CMD_HIST][CMD_MAX];  // history ring buffer
static char    _edit[CMD_MAX];            // current edit line (not NUL-terminated during edit)
static uint8_t _editLen;                  // chars in _edit
static uint8_t _editCursor;              // insert position 0.._editLen
static uint8_t _dispPos;                 // consume position for hostConsumeXxx()
static uint8_t _histHead;                // ring index of next write slot
static uint8_t _histCount;              // valid entries 0..CMD_HIST
static int8_t  _histIdx;                 // -1 = new input; 0 = newest; 1 = older …
static uint8_t _escState;               // 0=NORMAL 1=ESC 2=CSI
static uint8_t _csiParam;              // accumulated numeric CSI parameter
static bool    _hadCR;                  // CRLF suppression

/* ── RS485-wrapped output helpers ───────────────────────────────────────── */

static void _txChar(char c) {
    rs485Status(true);
    Serial.write((uint8_t)c);
    rs485Status(false);
}

static void _txStr(const char* s) {
    rs485Status(true);
    Serial.write(s);
    rs485Status(false);
}

/* Redraw the edit line after any in-place change.
 * Sequence: CR, full line content, ESC[K (erase tail), ESC[nD (reposition). */
static void _redraw(void) {
    rs485Status(true);
    Serial.write('\r');
    if (_editLen) Serial.write((const uint8_t*)_edit, _editLen);
    Serial.write("\x1b[K");
    if (_editCursor < _editLen) {
        uint8_t d = _editLen - _editCursor;
        char seq[7];
        snprintf(seq, sizeof(seq), "\x1b[%uD", (unsigned)d);
        Serial.write(seq);
    }
    rs485Status(false);
}

/* ── Public buffer accessors ────────────────────────────────────────────── */

/** Pointer to unconsumed tail of the dispatched command. */
const char* hostCmdStr(void) {
    return (_dispPos <= _editLen) ? _edit + _dispPos : _edit + _editLen;
}

/** Remaining unconsumed bytes in the dispatched command. */
uint8_t hostCmdLen(void) {
    return (_editLen > _dispPos) ? _editLen - _dispPos : 0;
}

/** Reset after command is fully consumed; prepares for next input. */
void hostCmdReset(void) {
    _editLen    = 0;
    _editCursor = 0;
    _dispPos    = 0;
}

/* ── Consume functions (work on dispatched command via _dispPos) ─────────── */

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

/* ── hostReadCmd: terminal line editor with history ─────────────────────── */

bool hostReadCmd(void) {
    while (Serial.available()) {
        char c = (char)Serial.read();

        switch (_escState) {

        /* ── Normal input ─────────────────────────────────────────────── */
        case 0:
            if (c == '\x1b') {
                _escState = 1;

            } else if (c == '\r' || c == '\n') {
                bool suppress = (c == '\n' && _hadCR);
                _hadCR = (c == '\r');
                _escState = 0;
                if (!suppress && _editLen > 0) {
                    _edit[_editLen] = '\0';
                    for (uint8_t i = 0; i < _editLen; ++i)
                        _edit[i] = (char)toupper((unsigned char)_edit[i]);
                    memcpy(_hist[_histHead], _edit, _editLen + 1);
                    _histHead  = (_histHead + 1) % CMD_HIST;
                    if (_histCount < CMD_HIST) ++_histCount;
                    _histIdx   = -1;
                    _dispPos   = 0;
                    _txStr("\r\n");
                    return true;
                }

            } else if (c == '\x7f' || c == '\x08') {   /* backspace / DEL */
                if (_editCursor > 0) {
                    memmove(_edit + _editCursor - 1,
                            _edit + _editCursor,
                            _editLen - _editCursor);
                    --_editCursor;
                    --_editLen;
                    _redraw();
                }

            } else if (c >= 0x20 && c < 0x7f) {        /* printable */
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
                        _txChar(c);             /* cursor at end — single-char echo */
                    }
                }
            }
            break;

        /* ── ESC received ────────────────────────────────────────────── */
        case 1:
            if (c == '[') { _escState = 2; _csiParam = 0; }
            else           { _escState = 0; }
            break;

        /* ── ESC [ received (CSI) ─────────────────────────────────────── */
        case 2:
            if (c >= '0' && c <= '9') {
                _csiParam = _csiParam * 10 + (uint8_t)(c - '0');
                break;  /* stay in CSI, accumulating parameter */
            }
            _escState = 0;

            if (c == 'A') {                             /* up — older history */
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

            } else if (c == 'C') {                      /* right */
                if (_editCursor < _editLen) {
                    ++_editCursor;
                    _txStr("\x1b[C");
                }

            } else if (c == 'D') {                      /* left */
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

            } else if (_csiParam == 3 && c == '~') {   /* Delete */
                if (_editCursor < _editLen) {
                    memmove(_edit + _editCursor,
                            _edit + _editCursor + 1,
                            _editLen - _editCursor - 1);
                    --_editLen;
                    _redraw();
                }
            }
            break;
        }
    }
    return false;
}
