#pragma once

#ifndef WHEEL_H__
#define WHEEL_H__

#include "SimpleFOC.h"
#include "PID_v1.h"

// Wheel motor controller pin definitions
#define POWER_PIN  12   // wheel throttle power
#define POWER_DIR_PIN 5   // wheel throttle direction
#define HALL_A_PIN 4 // hall sensor A
#define HALL_B_PIN 14 // hall sensor B
#define HALL_C_PIN 15 // hall sensor C

#define POWER_PWM_CHANNEL 0

double currentVelocity = 0.0; // current wheel velocity
double outputVelocity = 0.0; // PID output wheel velocity
double targetVelocity = 0.0; // wheel target velocity
double wheel_p = 0; // kP wheel value
double wheel_i = 0; // kI wheel value
double wheel_d = 0; // kD wheel value
int hallA = 0;
int hallB = 0;
int hallC = 0;


PID wheelPID(&currentVelocity, &outputVelocity, &targetVelocity, wheel_p, wheel_i, wheel_d, DIRECT);
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

    wheelPID.Compute();
    int pwm_output = (outputVelocity == 0) ? 0 : map(outputVelocity, 0, 1, 100, 150);
    ledcWrite(POWER_PWM_CHANNEL, pwm_output);
}




#endif