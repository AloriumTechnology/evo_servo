/*--------------------------------------------------------------------
 Copyright (c) 2021 Alorium Technology.  All right reserved.
 This file is part of the Alorium Technology Evo evo_servo library.
 Written by Bryan Craker (support@aloriumtech.com) of
   Alorium Technology (info@aloriumtech.com) using the same interface as
   the Arduino evo_servo library by Michael Margolis, but the Evo
   implementation is modified to take advantage of the FPGA hardware
   acceleration available on the Evo board.

 MIT License

 --------------------------------------------------------------------*/

#include <avr/interrupt.h>
#include <Arduino.h>

#include "evo_servo.h"

#define SVEN  7
#define SVDIS 6
#define SVUP  5

static servo_t servos[MAX_SERVOS];                          // static array of servo structures
static volatile int8_t Channel[1 ];             // counter for the servo being pulsed for each timer (or -1 if refresh interval)

uint8_t evo_servoCount = 0;                                     // the total number of attached servos

//int is 32 bits in samd
uint32_t servo_ctl_val;

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo

evo_servo::evo_servo()
{
  if( evo_servoCount < MAX_SERVOS) {
    this->servoIndex = evo_servoCount++;                    // assign a servo index to this instance
	  servos[this->servoIndex].microseconds = DEFAULT_PULSE_WIDTH;   // store default values
  }
  else
    this->servoIndex = INVALID_SERVO ;  // too many servos
}

uint8_t evo_servo::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t evo_servo::attach(int pin, int min, int max)
{
  if((this->servoIndex < MAX_SERVOS ) && (pin < MAX_SERVOS)) {
    evo_send_write_trans(D2F_ENCLR_ADDR, (1 << pin)); // disable D2F on the pin to allow the servo signal through from the FPGA
    servos[this->servoIndex].Pin.nbr = pin;
    this->min  = (MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    this->max  = (MAX_PULSE_WIDTH - max)/4;
    // Start the hardware
    servo_ctl_val = B110 << 24; //You may disable pwh, pwl, or ctl
    servo_ctl_val = servo_ctl_val + (servos[this->servoIndex].microseconds) << 8;
    servo_ctl_val = servo_ctl_val + ((1 << SVEN) | (1 << SVUP) | (servos[this->servoIndex].Pin.nbr & B11111));
    evo_send_write_trans(EVO_SERVO_CTL_ADDR, servo_ctl_val);
    servos[this->servoIndex].Pin.isActive = true;
  }
  return this->servoIndex ;
}

void evo_servo::detach()
{
  servos[this->servoIndex].Pin.isActive = false;
  servo_ctl_val = B110 << 24; //You may disable pwh, pwl, or ctl
  servo_ctl_val = servo_ctl_val + ((1 << SVDIS) | (servos[this->servoIndex].Pin.nbr & B11111));
  evo_send_write_trans(EVO_SERVO_CTL_ADDR, servo_ctl_val);
}

void evo_servo::write(int value, int speed)
{
  if(value < MIN_PULSE_WIDTH)
  {  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    if(value < 0) value = 0;
    if(value > 180) value = 180;
    value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
    if (speed > 15) speed = 15; 
    else if (speed < 0) speed = 0;
  }
  this->writeMicroseconds(value, speed);
}

void evo_servo::writeMicroseconds(int value, int speed)
{
  // calculate and store the values for the given channel
  byte channel = this->servoIndex;
  if( (channel < MAX_SERVOS) )   // ensure channel is valid
  {
    if( value < SERVO_MIN() )          // ensure pulse width is valid
      value = SERVO_MIN();
    else if( value > SERVO_MAX() )
      value = SERVO_MAX();
    if (speed > 15) speed = 15;
    else if (speed < 0) speed = 0;

    servos[channel].microseconds = value;
    // Copy new value to the hardware
    servo_ctl_val = B000 << 24; //You may disable pwh, pwl, or ctl
    servo_ctl_val = servo_ctl_val + ((0xF000 & (speed << 12)) | (0x0FFF & servos[channel].microseconds)) << 8;
    servo_ctl_val = servo_ctl_val + ((1 << SVUP) | (servos[channel].Pin.nbr & B11111));
    evo_send_write_trans(EVO_SERVO_CTL_ADDR, servo_ctl_val);
  }
}

int evo_servo::read() // return the value as degrees
{
  return  map( this->readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int evo_servo::readMicroseconds()
{
  unsigned int pulsewidth;
  if( this->servoIndex != INVALID_SERVO )
    pulsewidth = servos[this->servoIndex].microseconds;
  else
    pulsewidth  = 0;

  return pulsewidth;
}

bool evo_servo::attached()
{
  // Hardware value should match software. Here's how to check the hardware if you need to. 
  //book enabled_check   
  return servos[this->servoIndex].Pin.isActive ;
}

