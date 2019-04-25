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

#ifndef FASTPCA9685_H
#define FASTPCA9685_H

#include <stdint.h>

class TwoWire;

class FastPCA9685
{
public:
  FastPCA9685(uint8_t addr = 0x40, float freq = 50);
  FastPCA9685(TwoWire *i2c, uint8_t addr = 0x40, float freq = 50);
  virtual ~FastPCA9685();
  void setFreq(float freq);
  void reset();

/**
 * @brief Set PWM value
 * value <= 0   : set full-off
 * value >= 4096: set full-on
 * else         : set PWM
 */
void writePWM (uint8_t pin, uint16_t value);

/**
 * @brief Set full-on or full-off
 * value == 0: set full-off
 * value != 0: set full-on
 */
void writeOnOff (uint8_t pin, bool onOff);

/**
 * @brief Read off-register
 * To get PWM: mask with 0xFFF
 * To get full-off bit: mask with 0x1000
 * Note: ALL_LED pin will always return 0
 */
uint16_t readOff (uint8_t pin);

/**
 * @brief Read on-register
 * To get PWM: mask with 0xFFF
 * To get full-on bit: mask with 0x1000
 * Note: ALL_LED pin will always return 0
 */
uint16_t readOn (uint8_t pin);


/**
 * * * * * * * * * * * * * * * * * *
 * Advanced PWM control
 * * * * * * * * * * * * * * * * * *
 */

/**
 * @brief Manually define PWM duty-cycle
 */
void writePWM(uint8_t pin, uint16_t on, uint16_t off);

/**
 * @brief Read on and/or off regsiters
 */
void readPWM(uint8_t pin, uint16_t *on, uint16_t *off);

/**
 * @brief Set full-on bit to 1 or 0
 */
void writeOn(uint8_t pin, bool tf);

/**
 * @brief Set full-off bit to 1 or 0
 */
void writeOff(uint8_t pin, bool tf);

private:

  void setup();
  void write8(uint8_t reg, uint8_t data);
  void write16(uint8_t reg, uint16_t data);
  uint8_t read8(uint8_t reg);
  uint16_t read16(uint8_t reg);
  uint8_t baseReg(uint8_t pin);

  uint8_t _addr;
  float _freq;
  TwoWire *_i2c;

};

#endif // FASTPCA9685_H
