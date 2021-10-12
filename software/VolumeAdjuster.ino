// Volume Adjuster for ATtiny13A
// 
// Recently Great Scott built his version of an automatic volume adjuster.
// In this project he solved one of the biggest problems with modern movies.
// The problem is that the conversation volume is way too low in comparison
// to the music volume. That is why he combined an Arduino microcontroller
// with a microphone and an IR LED in order to create his automatic volume
// adjuster. It basically detects when loud movie music starts playing,
// lowers the volume and then brings the volume back up when the music is over.
//
// Since using an Arduino for this task is a bit of an overkill, here's a version
// for the ATtiny13A.
//
//                        +-\/-+
//      --- A0 (D5) PB5  1|    |8  Vcc
// MIC  --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- 
// POTI --- A2 (D4) PB4  3|    |6  PB1 (D1) ------ IR LED
//                  GND  4|    |5  PB0 (D0) ------ 
//                        +----+    
//
// Controller: ATtiny13
// Core:       MicroCore (https://github.com/MCUdude/MicroCore)
// Clockspeed: 1.2 MHz internal
// BOD:        BOD disabled (energy saving)
// Timing:     Micros disabled
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// No Arduino core functions or libraries are used. Use the makefile if 
// you want to compile without Arduino IDE.
//
// Note: The internal oscillator may need to be calibrated for the device
//       to function properly.
//
// Based on a project by Great Scott:
// https://youtu.be/j1V2I-otdzk
//
// 2021 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// Oscillator calibration value (uncomment and set if necessary)
//#define OSCCAL_VAL  0x6A

// Libraries
#include <avr/io.h>           // for GPIO
#include <avr/wdt.h>          // for the watchdog timer
#include <avr/sleep.h>        // for sleep functions
#include <avr/interrupt.h>    // for interrupts
#include <util/delay.h>       // for delays

// Pin assignments
#define IR_PIN      PB1       // IR LED pin
#define MIC_ADC     3         // ADC port for microphone
#define POT_ADC     2         // ADC port for volume control pot

// Assign IR commands for volume up/down. Different protocols and device
// addresses can be used.
#define VOL_UP()    NEC_sendCode(0x04,0x02) // LG TV volume up   (addr 0x04, cmd 0x02)
#define VOL_DOWN()  NEC_sendCode(0x04,0x03) // LG TV volume down (addr 0x04, cmd 0x03)

// Define hysteresis for volume adjustment
#define HYST      16          // hysteresis

// Macros to switch on/off IR LED
#define IR_on()   DDRB |=  (1<<IR_PIN)    // PB1 as output = IR at OC0B
#define IR_off()  DDRB &= ~(1<<IR_PIN)    // PB1 as input  = LED off

// -----------------------------------------------------------------------------------
// NEC Protocol Implementation
// -----------------------------------------------------------------------------------
//
// The NEC protocol uses pulse distance modulation.
//
//       +--------------+          +-------+       +-------+          +-- ON
//       |              |          |       |       |       |          |
//       |    9000us    |  4500us  |562.5us|562.5us|562.5us| 1687.5us |   ...
//       |              |          |       |       |       |          |
// ------+              +----------+       +-------+       +----------+   OFF
//
//       |<----- Start Frame ----->|<--- Bit=0 --->|<----- Bit=1 ---->| 
//
// IR telegram starts with a 9ms leading burst followed by a 4.5ms pause.
// Afterwards 4 data bytes are transmitted, least significant bit first.
// A "0" bit is a 562.5us burst followed by a 562.5us pause, a "1" bit is
// a 562.5us burst followed by a 1687.5us pause. A final 562.5us burst
// signifies the end of the transmission. The four data bytes are in order:
// - the 8-bit address for the receiving device,
// - the 8-bit logical inverse of the address,
// - the 8-bit command and
// - the 8-bit logical inverse of the command.
// The Extended NEC protocol uses 16-bit addresses. Instead of sending an
// 8-bit address and its logically inverse, first the low byte and then the
// high byte of the address is transmitted.

// Define values for 38kHz PWM frequency and 25% duty cycle
#define NEC_TOP   31          // 1200kHz / 38kHz - 1 = 31
#define NEC_DUTY  7           // 1200kHz / 38kHz / 4 - 1 = 7

// Macros to modulate the signals according to NEC protocol with compensated timings
#define NEC_startPulse()    {IR_on(); _delay_us(9000); IR_off(); _delay_us(4500);}
#define NEC_repeatPulse()   {IR_on(); _delay_us(9000); IR_off(); _delay_us(2250);}
#define NEC_normalPulse()   {IR_on(); _delay_us( 562); IR_off(); _delay_us( 557);}
#define NEC_bit1Pause()     _delay_us(1120) // 1687.5us - 562.5us = 1125us

// Send a single byte via IR
void NEC_sendByte(uint8_t value) {
  for (uint8_t i=8; i; i--, value>>=1) {  // send 8 bits, LSB first
    NEC_normalPulse();                    // 562us burst, 562us pause
    if (value & 1) NEC_bit1Pause();       // extend pause if bit is 1
  }
}

// Send complete telegram (start frame + address + command) via IR
void NEC_sendCode(uint16_t addr, uint8_t cmd) {
  // Prepare carrier wave
  OCR0A = NEC_TOP;            // set PWM frequency
  OCR0B = NEC_DUTY;           // set duty cycle

  // Send telegram
  NEC_startPulse();           // 9ms burst + 4.5ms pause to signify start of transmission
  if (addr > 0xFF) {          // if extended NEC protocol (16-bit address):
    NEC_sendByte(addr);       // send address low byte
    NEC_sendByte(addr >> 8);  // send address high byte
  } else {                    // if standard NEC protocol (8-bit address):
    NEC_sendByte(addr);       // send address byte
    NEC_sendByte(~addr);      // send inverse of address byte
  }
  NEC_sendByte(cmd);          // send command byte
  NEC_sendByte(~cmd);         // send inverse of command byte
  NEC_normalPulse();          // 562us burst to signify end of transmission
}

// -----------------------------------------------------------------------------------
// SAMSUNG Protocol Implementation
// -----------------------------------------------------------------------------------
//
// The SAMSUNG protocol corresponds to the NEC protocol, except that the start pulse is
// 4.5ms long and the address byte is sent twice.

#define SAM_startPulse()    {IR_on(); _delay_us(4500); IR_off(); _delay_us(4500);}

// Send complete telegram (start frame + address + command) via IR
void SAM_sendCode(uint8_t addr, uint8_t cmd) {
  // Prepare carrier wave
  OCR0A  = NEC_TOP;           // set PWM frequency
  OCR0B  = NEC_DUTY;          // set duty cycle

  // Send telegram
  SAM_startPulse();           // 9ms burst + 4.5ms pause to signify start of transmission
  NEC_sendByte(addr);         // send address byte
  NEC_sendByte(addr);         // send address byte again
  NEC_sendByte(cmd);          // send command byte
  NEC_sendByte(~cmd);         // send inverse of command byte
  NEC_normalPulse();          // 562us burst to signify end of transmission
}

// -----------------------------------------------------------------------------------
// RC-5 Protocol Implementation
// -----------------------------------------------------------------------------------
//
// The RC-5 protocol uses bi-phase modulation (Manchester coding).
//
//   +-------+                     +-------+    ON
//           |                     |
//     889us | 889us         889us | 889us
//           |                     |
//           +-------+     +-------+            OFF
//
//   |<-- Bit "0" -->|     |<-- Bit "1" -->|
//
// IR telegram starts with two start bits. The first bit is always "1",
// the second bit is "1" in the original protocol and inverted 7th bit
// of the command in the extended RC-5 protocol. The third bit toggles
// after each button release. The next five bits represent the device
// address, MSB first and the last six bits represent the command, MSB
// first.

// Define values for 36kHz PWM frequency and 25% duty cycle
#define RC5_TOP   32          // 1200kHz / 36kHz - 1 = 32
#define RC5_DUTY  7           // 1200kHz / 36kHz / 4 - 1 = 7

// Macros to modulate the signals according to RC-5 protocol with compensated timings
#define RC5_bit0Pulse()     {IR_on();  _delay_us(889); IR_off(); _delay_us(884);}
#define RC5_bit1Pulse()     {IR_off(); _delay_us(889); IR_on();  _delay_us(884);}

// Bitmasks
#define RC5_startBit  0b0010000000000000
#define RC5_cmdBit7   0b0001000000000000
#define RC5_toggleBit 0b0000100000000000

// Toggle variable
uint8_t RC5_toggle = 0;

// Send complete telegram (startbits + togglebit + address + command) via IR
void RC5_sendCode(uint8_t addr, uint8_t cmd) {
  // Prepare carrier wave
  OCR0A  = RC5_TOP;                           // set PWM frequency
  OCR0B  = RC5_DUTY;                          // set duty cycle

  // Prepare the message
  uint16_t message = addr << 6;               // shift address to the right position
  message |= (cmd & 0x3f);                    // add the low 6 bits of the command
  if (~cmd & 0x40) message |= RC5_cmdBit7;    // add inverse of 7th command bit
  message |= RC5_startBit;                    // add start bit
  if (RC5_toggle) message |= RC5_toggleBit;   // add toggle bit

  // Send the message
  uint16_t bitmask = RC5_startBit;            // set the bitmask to first bit to send
  for(uint8_t i=14; i; i--, bitmask>>=1) {    // 14 bits, MSB first
    (message & bitmask) ? (RC5_bit1Pulse()) : (RC5_bit0Pulse());  // send the bit
  }
  IR_off();                                   // switch off IR LED
  RC5_toggle ^= 1;                            // toggle the toggle bit
}

// -----------------------------------------------------------------------------------
// SONY SIRC Protocol Implementation
// -----------------------------------------------------------------------------------
//
// The SONY SIRC protocol uses pulse length modulation.
//
//       +--------------------+     +-----+     +----------+     +-- ON
//       |                    |     |     |     |          |     |
//       |       2400us       |600us|600us|600us|  1200us  |600us|   ...
//       |                    |     |     |     |          |     |
// ------+                    +-----+     +-----+          +-----+   OFF
//
//       |<------ Start Frame ----->|<- Bit=0 ->|<--- Bit=1 ---->| 
//
// A "0" bit is a 600us burst followed by a 600us space, a "1" bit is a
// 1200us burst followed by a 600us space. An IR telegram starts with a
// 2400us leading burst followed by a 600us space. The command and
// address bits are then transmitted, LSB first. Depending on the
// protocol version, these are in detail:
// - 12-bit version: 7 command bits, 5 address bits
// - 15-bit version: 7 command bits, 8 address bits
// - 20-bit version: 7 command bits, 5 address bits, 8 extended bits

// Define values for 40kHz PWM frequency and 25% duty cycle
#define SON_TOP   29                      // 1200kHz / 40kHz - 1 = 29
#define SON_DUTY  7                       // 1200kHz / 40kHz / 4 - 1 = 7

// Macros to modulate the signals according to SONY protocol with compensated timings
#define SON_startPulse()    {IR_on(); _delay_us(2400); IR_off(); _delay_us( 595);}
#define SON_bit0Pulse()     {IR_on(); _delay_us( 600); IR_off(); _delay_us( 595);}
#define SON_bit1Pulse()     {IR_on(); _delay_us(1200); IR_off(); _delay_us( 595);}

// Send "number" of bits of "value" via IR
void SON_sendByte(uint8_t value, uint8_t number) {
  do {                                      // send number of bits, LSB first
    (value & 1) ? (SON_bit1Pulse()) : (SON_bit0Pulse());  // send bit
    value>>=1;                              // next bit
  } while(--number);
}

// Send complete telegram (start frame + command + address) via IR
void SON_sendCode(uint16_t addr, uint8_t cmd, uint8_t bits) {
  // Prepare carrier wave
  OCR0A  = SON_TOP;                         // set PWM frequency
  OCR0B  = SON_DUTY;                        // set duty cycle

  // Send telegram
  SON_startPulse();                         // signify start of transmission
  SON_sendByte(cmd, 7);                     // send 7 command bits
  switch (bits) {
    case 12: SON_sendByte(addr, 5); break;  // 12-bit version: send 5 address bits
    case 15: SON_sendByte(addr, 8); break;  // 15-bit version: send 8 address bits
    case 20: SON_sendByte(addr, 8); SON_sendByte(addr>>8, 5); break; // 20-bit: 13 bits
    default: break;
  }
}

// -----------------------------------------------------------------------------
// Watchdog Timer Implementation
// -----------------------------------------------------------------------------

// Reset watchdog timer
void resetWatchdog(void) {
  cli();                                  // timed sequence coming up
  wdt_reset();                            // reset watchdog
  MCUSR = 0;                              // clear various "reset" flags
  WDTCR = (1<<WDCE)|(1<<WDE)|(1<<WDTIF);  // allow changes, clear interrupt
  WDTCR = (1<<WDTIE)|(1<<WDP0);           // set WDT interval 32 ms
  sei();                                  // interrupts are required now
}

// Watchdog interrupt service routine
ISR (WDT_vect) {
  wdt_disable();                          // disable watchdog
}

// -----------------------------------------------------------------------------------
// ADC Implementation
// -----------------------------------------------------------------------------------

// ADC read port
uint16_t readADC(uint8_t port) {
  PRR     = 0;                            // power on ADC
  ADMUX   = port;                         // set port, Vcc as reference
  ADCSRA |= (1<<ADEN) | (1<<ADSC);        // enable ADC, start sampling
  while (ADCSRA & (1<<ADSC));             // wait until sampling complete
  uint16_t result = ADC;                  // read sampling result
  ADCSRA &= ~(1<<ADEN);                   // disable ADC
  PRR     = 1<<PRADC;                     // shut down ADC
  return (result);                        // return sampling result
}

// -----------------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------------

// Main function
int main(void) {
  // Reset watchdog timer
  resetWatchdog();                      // do this first in case WDT fires

  // Set oscillator calibration value
  #ifdef OSCCAL_VAL
    OSCCAL = OSCCAL_VAL;                // set the value if defined above
  #endif
  
  // Set timer0 to toggle IR pin at carrier wave frequency
  TCCR0A = (1<<COM0B1)|(1<<WGM01)|(1<<WGM00); // PWM on OC0B (PB1)
  TCCR0B = (1<<WGM02)|(1<<CS00);              // start timer, no prescaler
  
  // Setup ADC
  ADCSRA = (1<<ADPS1)|(1<<ADPS0);       // set ADC prescaler 8

  // Disable unused peripherals and prepare sleep mode to save power
  ACSR  = 1<<ACD;                       // disable analog comperator
  DIDR0 = 0x1F;                         // disable digital intput buffer
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down

  // Setup all control variables
  uint16_t peak = readADC(MIC_ADC);     // read initial mic peak value
  uint16_t poti = readADC(POT_ADC);     // read initial volume control pot
  uint8_t  peak_timer  = 8;             // timer for next peak reading
  uint8_t  poti_timer  = 16;            // timer for next poti reading
  uint8_t  vol_counter = 0;             // counter for IR volume down control signals
  if(poti < HYST) poti = HYST;          // minimum poti value 

  // Main loop
  while(1) {
    resetWatchdog();                    // reset watchdog
    sleep_mode();                       // sleep until wake up by watchdog

    peak = ((peak << 5) - peak + readADC(MIC_ADC)) >> 5;  // low pass filter

    if(!(--poti_timer)) {               // time to update poti?
      poti = readADC(POT_ADC);          // update volume control pot
      if(poti < HYST) poti = HYST;      // minimum poti value
      poti_timer = 16;                  // reset poti timer
    }

    if(!(--peak_timer)) {               // time to check volume?
      if(peak > poti) {                 // volume too high?
        VOL_DOWN();                     // send "volume down" via IR
        vol_counter++;                  // increase counter for IR signals
        peak = (peak + readADC(MIC_ADC)) >> 1;  // reset peak value
      }
      else if((vol_counter) && (peak < (poti - HYST))) {  // volume too low?
        VOL_UP();                       // send "volume up" via IR
        vol_counter--;                  // decrease counter for IR signals
        peak = (peak + readADC(MIC_ADC)) >> 1;  // reset peak value
      }
      peak_timer = 8;                   // reset peak timer
    }
  }
}
