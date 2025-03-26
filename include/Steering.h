#pragma once

#ifndef STEERING_H_
#define STEERING_H_

#include "CytronMotorDriver.h"
#include "AS5600.h"
#include "PID_v1.h"

// Steering motor controller pin definitions
#define STEER_PIN 2    // steering power
#define STEER_DIR_PIN 17 // steering direction, TXD
#define ENC_SCL_PIN 32 // steering encoder scl, CFG
#define ENC_SDA_PIN 33 // steering encoder sda, 485_EN

double currentAngle = 0.0; // current steer angle
double outputAngle = 0.0; // PID steer output
double targetAngle = 0.0; // steer angle target
double steer_p = 0; // kP steer value
double steer_i = 0; // kI steer value
double steer_d = 0; // kD steer value

double steeringCenter = 2048.0; // encoder value when the wheel is centered


PID steerPID(&currentAngle, &outputAngle, &targetAngle, steer_p, steer_i, steer_d, DIRECT); // PID controller for steering
CytronMD steerController(PWM_DIR, STEER_PIN, STEER_DIR_PIN); // steering controller for cytron MC
AS5600 as5600(&Wire); // steering encoder

// converts encoder values (0-4096) to degrees.
double encToDegree(double encValue) {
    return (encValue - steeringCenter) * (180.0 / 2048.0);
}

// reads from the encoder, computes PID, and sets the motor speed.
void steerLoop() {
    currentAngle = encToDegree(as5600.readAngle());
    steerPID.Compute();
    steerController.setSpeed(outputAngle);
}

// sets the center of steering to the current angle.
void setSteerCenter() {
    steeringCenter = as5600.readAngle();
}



#endif