/*--------------------------------------------------------------------
  Copyright (c) 2021 Alorium Technology.  All right reserved.
  This file is part of the Alorium Technology Evo Servo library.
  Written by Bryan Craker (support@aloriumtech.com) of 
    Alorium Technology (info@aloriumtech.com) using the same interface as
    the Arduino Servo library by Michael Margolis, but the Evo
    implementation is modified to take advantage of the FPGA hardware
    acceleration available on the Evo board.
 
  Notable improvements compared to the Arduino library and hardware
  include the following:
    Reduced jitter
    PWM on pins 9 and 10 is not lost
    16 bit timer function is not lost

  Usage
  The evo_servo library is a replacement for the standard
    Arduino Servo library which is well documented at
    (https://www.arduino.cc/en/Reference/Servo).
    To get the Evo advantages, simply change
      #include <Servo.h>
    to
      #include <evo_servo.h>

  MIT License
 --------------------------------------------------------------------*/

/* 
  A servo is activated by creating an instance of the Servo class passing 
  the desired pin to the attach() method.
  The servos are pulsed in the background using the value most recently 
  written using the write() method.

  In contrast to the standard Servo library, this library does not use the
   AVR timers and does not disable the analogWrite function on any pins.

  The methods are:

    Servo - Class for manipulating servo motors connected to Arduino pins.

    attach(pin )  - Attaches a servo motor to an i/o pin.
    attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
    default min is 544, max is 2400  
 
    write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
    writeMicroseconds() - Sets the servo pulse width in microseconds 
    read()      - Gets the last written servo pulse width as an angle between 0 and 180. 
    readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
    attached()  - Returns true if there is a servo attached. 
    detach()    - Stops an attached servos from pulsing its i/o pin. 
 */

#ifndef EVOSERVO_H
#define EVOSERVO_H

// Gil temporary workaround with the following define
#define ARDUINO_EVO

// #ARDUINO_EVO is passed from IDE to the compiler if Evo is selected properly
#ifdef ARDUINO_EVO


#include <inttypes.h>

#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached

#define MAX_SERVOS             32     // all servos are independent, MAX depends on how many are built into the FPGA

#define INVALID_SERVO         255     // flag indicating an invalid servo index
#define EVO_SERVO_CTL_ADDR    0x8AA

typedef struct  {
  uint8_t nbr        :6 ;             // a pin number from 0 to 63 (although above MAX_SERVOS doesn't get used)
  uint8_t isActive   :1 ;             // true if this channel is enabled, pin not pulsed if false 
} ServoPin_t   ;  

typedef struct {
  ServoPin_t Pin;
  volatile unsigned int microseconds;
} servo_t;

class evo_servo
{
public:
  evo_servo();
  uint8_t attach(int pin);           // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(int pin, int min, int max); // as above but also sets min and max values for writes. 
  void detach();
  void write(int value, int speed = 0);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
  void writeMicroseconds(int value, int speed = 0); // Write pulse width in microseconds 
  int read();                        // returns current pulse width as an angle between 0 and 180 degrees
  int readMicroseconds();            // returns current pulse width in microseconds for this servo
  bool attached();                   // return true if this servo is attached, otherwise false 
private:
   uint8_t servoIndex;               // index into the channel data for this servo
   int8_t min;                       // minimum is this value times 4 added to MIN_PULSE_WIDTH    
   int8_t max;                       // maximum is this value times 4 added to MAX_PULSE_WIDTH   
};

#else
#error "Evo Servo library requires Tools->Board->EVOxxx selection. Install boards from https://github.com/AloriumTechnology/Arduino_Boards"
#endif

#endif
