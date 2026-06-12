// ow485-comms.h

String CmdBuf = "";                   // buffer containing data from master/mote

/*
 * Host buffer consume support
 */
uint8_t convertUpperToHex(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return 0xFF;
}

char hostConsumeChar(void) {
  if (CmdBuf.length() > 0) {
    char cChr = CmdBuf.charAt(0);
    CmdBuf.remove(0, sizeof(char));
    return cChr;
  }
  return -1;
}

char hostConsumeHexChar(void) { return convertUpperToHex(hostConsumeChar()); }

long int hostConsumeHexValue(int8_t Num) {
  long int iVal = 0;
  int8_t Idx = 0;
  do {
    char cChr = hostConsumeChar();                  // get next char
    if (cChr == ',' || cChr == -1)                  // seperator or terminator reached?
      break;                                        // stop parsing
    uint8_t u8Val = convertUpperToHex(cChr);        // convert char to hex value equivalent
    if (u8Val == 0xFF)                              // Invalid / non-hex char
      return -1;                                    // immediate error
    iVal <<= 4;
    iVal += u8Val;
  } while (++Idx < Num);
  if (Idx == Num) {                                 // requested # of chars consumed...
    if (CmdBuf.charAt(0) == ',')                    // seperator next char?
      hostConsumeChar();                            // waste it.....
  }
  return (Idx == 0 || Idx > Num) ? -1L : iVal;
}

uint8_t hostConsumeString(char * Buf, uint8_t u8S) {
  CmdBuf.toCharArray(Buf, u8S);
  if (CmdBuf.charAt(u8S - 1) != ',')
    --u8S;
  CmdBuf.remove(0, u8S);
  return u8S;
}

bool hostReadCmd(void) {
  static bool hadCR = false;
  if (!Serial.available()) return false;
  char c = Serial.read();
  if (c == '\r' || c == '\n') {
    bool suppress = (c == '\n' && hadCR);  // discard the \n of a CRLF pair
    hadCR = (c == '\r');
    if (!suppress && CmdBuf.length() > 0) {
      CmdBuf.toUpperCase();
      return true;
    }
    return false;
  }
  hadCR = false;
  CmdBuf += c;
  return false;
}
