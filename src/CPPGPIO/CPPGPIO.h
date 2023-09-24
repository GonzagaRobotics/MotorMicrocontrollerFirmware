/*
Title: CPPGPIO.h
Author: Christine Cabero, Ben Pauka, Matthew VonWahlde, Cameron Zheng
Date Created: 12/3/22
Date Modified: 2/2/23
Description: This file sets up the GPIO pins for the ESP 32 controlling the motors (header file)
*/

#ifndef CPPGPIO_H
#define CPPGPIO_H

#include <Arduino.h>
#include "../MotorSet/MotorSet.h"


void setOutputPin(int);
void setUpPWMChannel(int, int, int, int);
void writePWMChannel(int, int);
MotorSet* addPinsToLeftMotorSet();
MotorSet* addPinsToRightMotorSet();


#endif