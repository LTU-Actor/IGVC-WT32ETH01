#pragma once

#ifndef WHEEL_H__
#define WHEEL_H__

#include "SimpleFOC.h"
// #include "PID_v1.h"
#include "QuickPID.h"

// Wheel motor controller pin definitions
#define POWER_PIN  12   // wheel throttle power
#define POWER_DIR_PIN 5   // wheel throttle direction
#define HALL_A_PIN 4 // hall sensor A, yellow
#define HALL_B_PIN 14 // hall sensor B, green
#define HALL_C_PIN 15 // hall sensor C, blue

#define POWER_PWM_CHANNEL 0


float currentVelocity = 0.0; // current wheel velocity
float outputVelocity = 0.0; // PID output wheel velocity
float targetVelocity = 0.0; // wheel target velocity
float wheel_p = 5; // kP wheel value
float wheel_i = 0; // kI wheel value
float wheel_d = 0; // kD wheel value
int hallA = 0;
int hallB = 0;
int hallC = 0;

bool wheelStop = false;


QuickPID wheelPID(&currentVelocity, &outputVelocity, &targetVelocity);
HallSensor hall(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN, 14);


void doA(){hall.handleA();}
void doB(){hall.handleB();}
void doC(){hall.handleC();}


// reads from the hall sensor, computes PID, and sends PWM to wheel
void wheelLoop() {
    hall.update();
    currentVelocity = hall.getVelocity();
    hallA = digitalRead(HALL_A_PIN);
    hallB = digitalRead(HALL_B_PIN);
    hallC = digitalRead(HALL_C_PIN);

    if(!wheelStop) {
        wheelPID.Compute();
        outputVelocity = max(0.0f, outputVelocity);
        ledcWrite(POWER_PWM_CHANNEL, targetVelocity);
    }
    else {
        ledcWrite(POWER_PWM_CHANNEL, 0);
    }
}




#endif