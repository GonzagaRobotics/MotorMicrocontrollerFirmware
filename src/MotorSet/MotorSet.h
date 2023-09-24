/*
Title: MotorSet.h
Author: Christine Cabero, Ben Pauka, Matthew VonWahlde, Cameron Zheng
Date Created: 11/12/22
Date Modified: 2/2/23
Description: Header file for MotorSet
*/


#ifndef MOTOR_SET_H
#define MOTOR_SET_H


#include "../MotorController/MotorController.h"


#define maxMotors 3 // Max of 3 motors per side


/*
Function: MotorSet
Input: None
Output: None
Description: Declares an instance of a set of motors (all wheels in a rover)
*/
class MotorSet{
    public:
        ~MotorSet();

        void driveForwards(float);
        void driveBackwards(float);
        void stop();

        void addMotor(int, int, char);
    private:
        MotorController* motorSet[maxMotors];
        int numMotors = 0;
        char motorSide;  
};


#endif
