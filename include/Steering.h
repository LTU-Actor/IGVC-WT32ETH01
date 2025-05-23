#pragma once

#ifndef STEERING_H_
#define STEERING_H_

#include "CytronMotorDriver.h"
#include "AS5600.h"
// #include "PID_v1.h"
// #include "SimpleFOC.h"
#include "QuickPID.h"

// Steering motor controller pin definitions
#define STEER_PIN 2    // steering power
#define STEER_DIR_PIN 17 // steering direction, TXD
#define ENC_SCL_PIN 32 // steering encoder scl, CFG
#define ENC_SDA_PIN 33 // steering encoder sda, 485_EN

#define ENC_TICKS 4096 // total number of encoder ticks
#define ENC_RANGE 1200 // steering range on either side of the steering center
#define TICK_TOLERANCE 25 // tolerance between current and target angle to stop the wheel

float steeringCenter = 1097.0; // encoder value when the wheel is centered
float minTick = steeringCenter - ENC_RANGE;
float maxTick = steeringCenter + ENC_RANGE;

float encRaw = 0.0;
float currentAngle = 0.0; // current steer angle
float outputAngle = 0.0; // PID steer output
float targetAngle = steeringCenter; // steer angle target
float steer_p = 1; // kP steer value
float steer_i = 0; // kI steer value
float steer_d = 0.004; // kD steer value

bool steerStop = false;
int lastSteerError = 0;
String steerCode = "";




QuickPID steerPID(&currentAngle, &outputAngle, &targetAngle); // PID controller for steering
CytronMD steerController(PWM_DIR, STEER_PIN, STEER_DIR_PIN); // steering controller for cytron MC
AS5600 as5600(&Wire); // steering encoder

// converts degrees to radians.
float degreeToRad(float degree) {
    return degree * (PI / 180.0);
}

// converts radians to degrees.
float radToDegree(float radian) {
    return radian * (180.0 / PI);
}

// converts encoder values (0 - 4096) to radians (-pi - pi).
float encToRad(float encValue) {
    return (encValue - steeringCenter) * (PI / 2048.0);
}

// converts radians (-pi - pi) to encoder values (0 - 4096).
float radToEnc(float radValue) {
    return radValue * (2048.0 / PI) + steeringCenter;
}

float filterEnc(float encValue) {
    return min(maxTick, max(minTick, encValue));
}

// reads from the encoder, computes PID, and sets the motor speed.
void steerLoop() {
    encRaw = as5600.readAngle(); 
    lastSteerError = as5600.lastError();
    if(lastSteerError != 0) { // if encoder got a bad read, stop the motor
        steerStop = true;
    }
    else {
        currentAngle = encRaw;
    }
    
    if(!steerStop) { // normal state
        steerPID.Compute();
        steerController.setSpeed(outputAngle);
        steerCode = String("OK");
    }
    else { // error state
        steerController.setSpeed(0);
        targetAngle = currentAngle;
        steerCode = String("STOPPED");
    }
}

// sets the center of steering to the current angle.
void setSteerCenter() {
    steeringCenter = as5600.readAngle();
    targetAngle = steeringCenter;
    minTick = steeringCenter - ENC_RANGE;
    maxTick = steeringCenter + ENC_RANGE;
}

// sets the center of steering to the current angle.
void setSteerCenterValue(int tick) {
    steeringCenter = tick;
    targetAngle = steeringCenter;
    minTick = steeringCenter - ENC_RANGE;
    maxTick = steeringCenter + ENC_RANGE;
}


#endif