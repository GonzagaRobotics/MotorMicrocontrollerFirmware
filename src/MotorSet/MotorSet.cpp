/*
Title: MotorSet.cpp
Author: Christine Cabero, Ben Pauka, Matthew VonWahlde, Cameron Zheng
Date Created: 11/12/22
Date Modified: 2/2/23
Description: Function file for MotorSet, controls the motors on one side of the
            rover (i.e. the right set of motors or the left set)
            Motor set refers to a collection of motor controllers.
*/


#include "MotorSet.h"


/*
Function: ~MotorSet()
Input: N/A
Output: N/A
Description: Destructor for MotorSet, deletes all variables and objects used
*/
MotorSet::~MotorSet(){
    // Deleting all MotorController instances that each element of set is pointing to
    for(int i = 0; i < numMotors; i++){
        delete[] motorSet[i];
    }

    // Deleting the set
    delete[] motorSet;
}


/*
Function: addMotor()
Input: pin 1, pin 2, enable pin, and enable pin
Output: N/A
Description: Adds an instance of MotorController to the motorSet array
*/
void MotorSet::addMotor(int pin1, int pwmChannel1, char motorSide){
    // If the max number of motors has not been reached
    if(numMotors < maxMotors){
        // increment the number of motors
        numMotors++;

        // add a new motor to the set
        motorSet[numMotors - 1] = new MotorController(pin1, pwmChannel1, motorSide);
    }
}


/*
Function: driveForwards()
Input: A speed to go forwards
Output: Changes all motors to the value
Description: Spins the wheels forwards
*/
void MotorSet::driveForwards(float speed){
    for (int i = 0; i < numMotors; i++) {
        // In set[] from 0 to numMotors, set each motor to backwards at the same speed
        motorSet[i]->motorForwards(speed); 
    }
}
