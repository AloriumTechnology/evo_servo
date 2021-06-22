/*
 evo_servo_sweep_hold

 Copyright (c) 2021 Alorium Technology.  All rights reserved.
 by Bryan Craker (support@aloriumtech.com) of
 Alorium Technology (info@aloriumtech.com)
 
 Demonstrates the awesomeness of the Evo Servo library and
  hardware. The example separately controls up to 16 servos.
  1 of them is done in a sweep pattern to give you something
  interesting to look at while the others are in a hold pattern
  where you can see if they are stable or jittery.
 If you'd like to put servo motors on all of pins being used
  you certainly can, but more typically leave most of them
  unconnected and just have two actual servo motors, one on
  the pin that is sweeping and one on any of the pins that are
  holding. If you have a long enough "arm" (a laser light works
  well) you should be able to see the difference in jitter between
  the ordinary and Evo versions.

*/

#include <evo_servo.h>

#define NUM_SERVOS 13
 
#define SWEEP_PIN 5

evo_servo myservos[NUM_SERVOS];  // create servo objects to control the servos
 
int pos = 0;    // variable to store the servo position on the ones that are sweeping

void setup() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    myservos[i].attach(i);  // 
    myservos[i].write(i*8); // start all servos holding in different positions
  }
}

void loop() {
  // Have one servo doing sweep pattern just to make something interesting
  //   to look at
  for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservos[pin].write(pos);  // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservos[pin].write(pos);  // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
}
