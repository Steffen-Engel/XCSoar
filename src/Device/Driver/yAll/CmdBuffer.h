#pragma once

#include "stdint.h"


class cCmdBuffer
{
private:
  #define INBUF_SIZE 512
  uint8_t inBuf[INBUF_SIZE];
  uint16_t p;
  int offset;

public:
  cCmdBuffer();
  ~cCmdBuffer();

  int fill();

  void Reset();
  int32_t read32();
  int16_t read16();
  int8_t read8();
  void write8(int8_t c);
};

