/*
  
  Arduino version of my PCA9685 library
  https://github.com/Reinbert/pca9685
 
  MIT License

  Copyright (c) 2019 Reinhard Sprung

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

#include "fastPCA9685.h"

// Setup registers
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

// Define first LED and all LED. We calculate the rest
#define LED0_ON_L 0x6
#define LEDALL_ON_L 0xFA
#define PIN_ALL 16


#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "Wire.h"

/**
 * @brief Setup a PCA9685 device
 * @param addr The default address is 0x40
 * @param freq Frequency will be capped to range [40..1000] Hertz. Try 50 for servos
 */
FastPCA9685::FastPCA9685(uint8_t addr, float freq)
{
#ifdef ARDUINO_SAM_DUE
  _i2c = &Wire1;
#else
  _i2c = &Wire;
#endif

  _addr = addr;
  _freq = freq;
  setup();
}

FastPCA9685::FastPCA9685(TwoWire *i2c, uint8_t addr, float freq)
{
  _i2c = i2c;
  _addr = addr;
  _freq = freq;
  setup();
}

FastPCA9685::~FastPCA9685()
{
  if (_i2c)
  {
    _i2c->endTransmission();
    _i2c = 0;
  }
}

void FastPCA9685::setup()
{
  _i2c->begin();

  // Setup the chip. Enable auto-increment of registers.
  uint8_t settings = read8(PCA9685_MODE1) & 0x7F;
  uint8_t autoInc = settings | 0x20;
  write8(PCA9685_MODE1, autoInc);

  // Set frequency of PWM signals. Also ends sleep mode and starts PWM output.
  if (_freq > 0)
    setFreq(_freq);
}

/**
 * Sets the frequency of PWM signals.
 * Frequency will be capped to range [40..1000] Hertz. Try 50 for servos.
 */
void FastPCA9685::setFreq(float freq)
{
  // Cap at min and max
  _freq = (freq > 1000 ? 1000 : (freq < 40 ? 40 : freq));

  // To set pwm frequency we have to set the prescale register. The formula is:
  // prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
  // Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
  int prescale = (int)(25000000.0f / (4096 * _freq) - 0.5f);

  // Get settings and calc bytes for the different states.
  uint8_t settings = read8(PCA9685_MODE1) & 0x7F;	// Set restart bit to 0
  uint8_t sleep    = settings | 0x10;				      // Set sleep bit to 1
  uint8_t wake     = settings & 0xEF;				      // Set sleep bit to 0
  uint8_t restart  = wake | 0x80;					        // Set restart bit to 1

  // Go to sleep, set prescale and wake up again.
  write8(PCA9685_MODE1, sleep);
  write8(PCA9685_PRESCALE, prescale);
  write8(PCA9685_MODE1, wake);

  // Now wait a millisecond until oscillator finished stabilizing and restart PWM.
  delay(1);
  write8(PCA9685_MODE1, restart);
}

/**
 * Set all pins to full-off
 */
void FastPCA9685::reset()
{
  write16(LEDALL_ON_L    , 0x0);
  write16(LEDALL_ON_L + 2, 0x1000);
}

/**
 * Simple PWM control which sets on-tick to 0 and off-tick to value.
 * If value is <= 0, full-off will be enabled
 * If value is >= 4096, full-on will be enabled
 * Every value in between enables PWM output
 */
void FastPCA9685::writePWM(uint8_t pin, uint16_t value)
{
  if (value >= 4096)
    writeOn(pin, 1);
  else if (value > 0)
    writePWM(pin, 0, value); // (Deactivates full-on and off automatically)
  else
    writeOff(pin, 1);
}

/**
 * Simple full-on and full-off control
 * If value is 0, full-off will be enabled
 * If value is not 0, full-on will be enabled
 */
void FastPCA9685::writeOnOff(uint8_t pin, bool onOff)
{
  if (onOff)
    writeOn(pin, 1);
  else
    writeOff(pin, 1);
}

/**
 * Reads off registers as 16 bit of data
 * To get PWM: mask with 0xFFF
 * To get full-off bit: mask with 0x1000
 * Note: ALL_LED pin will always return 0
 */
uint16_t FastPCA9685::readOff(uint8_t pin)
{
  uint16_t off;
  readPWM(pin, 0, &off);
  return off;
}

/**
 * Reads on registers as 16 bit of data
 * To get PWM: mask with 0xFFF
 * To get full-on bit: mask with 0x1000
 * Note: ALL_LED pin will always return 0
 */
uint16_t FastPCA9685::readOn(uint8_t pin)
{
  uint16_t on;
  readPWM(pin, &on, 0);
  return on;
}

/**
 * * * * * * * * * * * * * * * * * *
 * Advanced PWM control
 * * * * * * * * * * * * * * * * * *
 */

/**
 * Write on and off ticks manually to a pin
 * (Deactivates any full-on and full-off)
 */
void FastPCA9685::writePWM(uint8_t pin, uint16_t on, uint16_t off)
{
  uint8_t reg = baseReg(pin);
  // Write to on and off registers and mask the 12 lowest bits of data to overwrite full-on and off
  write16(reg    , on  & 0x0FFF);
  write16(reg + 2, off & 0x0FFF);
}

/**
 * Reads both on and off registers as 16 bit of data
 * To get PWM: mask each value with 0xFFF
 * To get full-on or off bit: mask with 0x1000
 * Note: ALL_LED pin will always return 0
 */
void FastPCA9685::readPWM(uint8_t pin, uint16_t *on, uint16_t *off)
{
  uint8_t reg = baseReg(pin);
  if (on)
    *on  = read16(reg);
  if (off)
    *off = read16(reg + 2);
}

/**
 * Enables or deactivates full-on
 * tf = true: full-on
 * tf = false: according to PWM
 */
void FastPCA9685::writeOn(uint8_t pin, bool tf)
{
  uint8_t reg = baseReg(pin) + 1; // LEDX_ON_H
  uint8_t state = read8(reg);

  // Set bit 4 to 1 or 0 accordingly
  state = tf ? (state | 0x10) : (state & 0xEF);
  write8(reg, state);

  // For simplicity, we set full-off to 0 because it has priority over full-on
  if (tf)
    writeOff(pin, 0);
}

/**
 * Enables or deactivates full-off
 * tf = true: full-off
 * tf = false: according to PWM or full-on
 */
void FastPCA9685::writeOff(uint8_t pin, bool tf)
{
  uint8_t reg = baseReg(pin) + 3;	// LEDX_OFF_H
  uint8_t state = read8(reg);

  // Set bit 4 to 1 or 0 accordingly
  state = tf ? (state | 0x10) : (state & 0xEF);
  write8(reg, state);
}

/**
 * * * * * * * * * * * * * * * * * *
 * Helper functions
 * * * * * * * * * * * * * * * * * *
 */

void FastPCA9685::write8(uint8_t reg, uint8_t data)
{
  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  _i2c->write(data);
  _i2c->endTransmission();
}

void FastPCA9685::write16(uint8_t reg, uint16_t data)
{
  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  _i2c->write((uint8_t*)&data, 2);
  _i2c->endTransmission();
}

uint8_t FastPCA9685::read8(uint8_t reg)
{
  // Send register to read from.
  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  _i2c->endTransmission();

  // Request 1 byte of data and read it.
  _i2c->requestFrom(_addr, (uint8_t)1);
  return _i2c->read();
}

uint16_t FastPCA9685::read16(uint8_t reg)
{
  // Send register to read from.
  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  _i2c->endTransmission();

  // Request 2 bytes of data and read them.
  _i2c->requestFrom(_addr, (uint8_t)2);
  uint16_t data = _i2c->read();
  data += _i2c->read() << 8;
  return data;
}

uint8_t FastPCA9685::baseReg(uint8_t pin)
{
  return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
}
