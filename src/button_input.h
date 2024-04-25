/*
Ensure that none of the pins set here happen to be RX or TX pins.
This has caused issues uploading sketches in the past.
*/

#ifndef button_input_h
#define button_input_h

#include <Arduino.h>
// #include <PinChangeInterrupt.h>
#include <TimerOne.h>

// Function declarations
void setupButtons();
void actionForButton();
void enableMotor(); // Defined in the drv8711_Tuning.ino sketch.
void disableMotor(); // Defined in the drv8711_Tuning.ino sketch.

#endif
