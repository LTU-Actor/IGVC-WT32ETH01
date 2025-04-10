#pragma once

#ifndef STEERING_H_
#define STEERING_H_

#include "CytronMotorDriver.h"
#include "AS5600.h"
// #include "PID_v1.h"
#include "SimpleFOC.h"
#include "QuickPID.h"

// Steering motor controller pin definitions
#define STEER_PIN 2    // steering power
#define STEER_DIR_PIN 17 // steering direction, TXD
#define ENC_SCL_PIN 32 // steering encoder scl, CFG
#define ENC_SDA_PIN 33 // steering encoder sda, 485_EN

#define ANGLE_TOLERANCE 1

float currEnc = 0.0;
float currentAngle = 0.0; // current steer angle
float outputAngle = 0.0; // PID steer output
float targetAngle = 0.0; // steer angle target
float steer_p = 8; // kP steer value
float steer_i = 0; // kI steer value
float steer_d = 0.2; // kD steer value

float steeringCenter = 3145.0; // encoder value when the wheel is centered


QuickPID steerPID(&currentAngle, &outputAngle, &targetAngle); // PID controller for steering
CytronMD steerController(PWM_DIR, STEER_PIN, STEER_DIR_PIN); // steering controller for cytron MC
AS5600 as5600(&Wire); // steering encoder

// converts encoder values (0-4096) to degrees.
float encToDegree(float encValue) {
    return (encValue - steeringCenter) * (180.0 / 2048.0);
}

// reads from the encoder, computes PID, and sets the motor speed.
void steerLoop() {
    currEnc = as5600.readAngle();
    currentAngle = encToDegree(currEnc);
    // currentAngle = as5600.getAngularSpeed();
    if(abs(targetAngle - currentAngle) < ANGLE_TOLERANCE) {
        targetAngle = currentAngle;
    }
    steerPID.Compute();
    // if(abs(outputAngle) < 10) {
    //     outputAngle = 0;
    // }
    steerController.setSpeed(outputAngle);
}

// sets the center of steering to the current angle.
void setSteerCenter() {
    steeringCenter = as5600.readAngle();
}



#endif