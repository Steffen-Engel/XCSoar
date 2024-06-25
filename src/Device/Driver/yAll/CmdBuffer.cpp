#include "stdint.h"

#include "CmdBuffer.h"


cCmdBuffer::cCmdBuffer()
{
  Reset();
}

cCmdBuffer::~cCmdBuffer()
{
}

int cCmdBuffer::fill()
{
  return offset;
}

void cCmdBuffer::Reset()
{
  p = 0;
  offset = 0;
}

int32_t cCmdBuffer::read32()
{
  int value = (inBuf[p]&0xff) + ((inBuf[p+1]&0xff)<<8) + ((inBuf[p+2]&0xff)<<16) + ((inBuf[p+3]&0xff)<<24);
  p+=4;
  return value;
}

int16_t cCmdBuffer::read16()
{
  int value = (inBuf[p]&0xff) + ((inBuf[p+1])<<8);
  p+=2;
  return value;
}

int8_t cCmdBuffer::read8()
{
  return inBuf[p++]&0xff;
}


void cCmdBuffer::write8(int8_t c)
{
  inBuf[offset++] = (uint8_t)(c&0xFF);
};

