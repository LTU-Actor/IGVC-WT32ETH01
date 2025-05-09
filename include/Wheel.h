#pragma once

#ifndef WHEEL_H__
#define WHEEL_H__

#include "SimpleFOC.h"

// Wheel motor controller pin definitions
#define POWER_PIN  12   // wheel throttle power
#define POWER_DIR_PIN 5   // wheel throttle direction
#define HALL_A_PIN 4 // hall sensor A, yellow
#define HALL_B_PIN 14 // hall sensor B, green
#define HALL_C_PIN 15 // hall sensor C, blue

#define POWER_PWM_CHANNEL 0

#define MAX_PWM 150

const float max_ang_vel = 8.5;
float currentVelocity = 0.0; // current wheel velocity
float outputVelocity = 0.0; // PID output wheel velocity
float targetVelocity = 0.0; // wheel target velocity
float wheel_p = 1; // kP wheel value
float wheel_i = 0; // kI wheel value
float wheel_d = 0; // kD wheel value
int hallA = 0;
int hallB = 0;
int hallC = 0;

int wheelDelay = 0;

bool wheelStop = false;
String wheelCode = "";

HallSensor hall(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN, 14);


void doA(){hall.handleA();}
void doB(){hall.handleB();}
void doC(){hall.handleC();}


// reads from the hall sensor, computes PID, and sends PWM to wheel
void wheelLoop() {
    hall.update();
    currentVelocity = hall.getVelocity();

    if(wheelDelay > 0) {
        wheelStop = true;
        wheelDelay -= 10;
    }

    if(!wheelStop) {
        // wheelPID.Compute();
        if(targetVelocity == 0) {
            outputVelocity = 0;
        }
        else {
            // outputVelocity = map(targetVelocity, 0, 2.25, 100, MAX_PWM);
            outputVelocity = targetVelocity;
        }
        ledcWrite(POWER_PWM_CHANNEL, outputVelocity);
        wheelCode = String("OK");
    }
    else {
        ledcWrite(POWER_PWM_CHANNEL, 0);
        wheelCode = String("STOPPED");
    }
}




#endif