 /*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef TelCO2sensor_h
#define TelCO2sensor_h

#include "Arduino.h"

class TelClaireCO2
{
  public:
    TelClaireCO2(int deviceAddress);
    void calibration();
    unsigned int status();
    int readCO2();
  private:
    int _deviceAddress;
};

#endif

