#pragma once

#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

#include "PID_v1.h"
#include <Arduino.h>
#include "ArduinoJson.h"

double currentVelocity = 0.0; // current wheel velocity
double outputVelocity = 0.0; // PID output wheel velocity
double targetVelocity = 0.0; // wheel target velocity
double wheel_p = 0; // kP wheel value
double wheel_i = 0; // kI wheel value
double wheel_d = 0; // kD wheel value
double currentAngle = 0.0; // current steer angle
double outputAngle = 0.0; // PID steer output
double targetAngle = 0.0; // steer angle target
double steer_p = 0; // kP steer value
double steer_i = 0; // kI steer value
double steer_d = 0; // kD steer value

PID wheelPID(&currentVelocity, &outputVelocity, &targetVelocity, wheel_p, wheel_i, wheel_d, DIRECT);
PID steerPID(&currentAngle, &outputAngle, &targetAngle, steer_p, steer_i, steer_d, DIRECT);

void setPIDTuning(String pid_json) {
    JsonDocument js;
    deserializeJson(js, pid_json);

    wheel_p = js["linear"]["x"];
    wheel_i = js["linear"]["y"];
    wheel_d = js["linear"]["z"];

    steer_p = js["angular"]["x"];
    steer_i = js["angular"]["y"];
    steer_d = js["angular"]["z"];

    wheelPID.SetTunings(wheel_p, wheel_i, wheel_d);
    steerPID.SetTunings(steer_p, steer_i, steer_d);
}

void computePID() {
    wheelPID.Compute();
    steerPID.Compute();
}






#endif