/* 
Title: DataProcess.cpp
Author: Christine Cabero, Ben Pauka, Matthew VonWahlde, Cameron Zheng, Nick Linthacum
Date Created: 2/2/23
Date Modified: 3/3/24
Description: Function definitions for data handling
*/

#include "DataProcess.h"

/*
Function: dataHandling
Input: Array of 2 floats, pointers to left and right motor set
        Left value, right value
Output: None
Description: Processes the input array and calls necessary functions
*/
void dataHandling(volatile float dataArr[], MotorSet* leftSet, MotorSet* rightSet){
    // Setting values to variables for easier use in functions (and reading clarity)
    float leftTrigger = dataArr[0];
    float rightTrigger = dataArr[1];

    float leftSpeedCurrent = leftTrigger;
    float rightSpeedCurrent = rightTrigger;
 
   if (leftSpeedCurrent == 0) { //If 0, then stop
        leftSet->stop();
    } else if (leftSpeedCurrent > 0) {
        leftSet->driveForwards(leftSpeedCurrent); //If positive, then go forward
    } else {
        leftSet->driveBackwards(-leftSpeedCurrent); //If negative, then go backwards
    }

    if (rightSpeedCurrent == 0) { //If 0, then stop
        rightSet->stop();
    } else if (rightSpeedCurrent > 0) {
        rightSet->driveForwards(rightSpeedCurrent); //If positive, then go forward
    } else {
        rightSet->driveBackwards(-rightSpeedCurrent); //If negative, then go backwards
    }

}


/*
Function: setTriggerWheelSpeed
Input: float - value of the trigger
Output: float - a value between 0 and 1
Description: Processes the input array and calls necessary functions
*/
float setTriggerWheelSpeed(float triggerVal){
    // if trigger is compressed fully (or meets threshold value), return full speed
    if(triggerVal < TRIGGER_COMPRESSED_THRESHOLD)
        return 1;

    // Return trigger value converted into a float between 0 (stopped) and 1 (full speed)
    return triggerVal * -0.5 + 0.5;
}

/*
Function: absValue
Input: float - value of the number
Output: float - the absolute value of the number
Description: Does the absolute value function on the number
*/
float absValue(float number) {
    return number >= 0 ? number : -number;
}