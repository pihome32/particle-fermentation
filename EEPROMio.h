#ifndef EEPROMIO_H
#define EEPROMIO_H

#include "application.h"
// #include <EEPROM.h>

enum {  // byte sizes of data types to make EEPROMRead/Write function calls more readable
  EEBYTE = 1,
  EECHAR = 1,
  EEINT = 4,
  EELONG = 4,
  EEDOUBLE = 8,
};

void EEPROMRead(unsigned int addr, byte *val, int bytes);
boolean EEPROMWrite(unsigned int addr, byte *val, int bytes);

template <typename T> void EEPROMRead(unsigned int addr, T* val, int bytes) {
  byte* temp = (byte*) val;
  EEPROMRead(addr, temp, bytes);
}

template <typename T> boolean EEPROMWrite(unsigned int addr, T val, int bytes) {
  byte* temp = (byte*) &val;
  return EEPROMWrite(addr, temp, bytes);
}

#endif