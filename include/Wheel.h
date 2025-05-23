#pragma once

#ifndef WHEEL_H__
#define WHEEL_H__

// #include <SoftwareSerial.h>

// Wheel motor controller pin definitions
#define POWER_PIN  12   // wheel throttle power
#define POWER_DIR_PIN 5   // wheel throttle direction
// #define HALL_A_PIN 4 // hall sensor A, yellow
// #define HALL_B_PIN 14 // hall sensor B, green
// #define HALL_C_PIN 15 // hall sensor C, blue

#define POWER_PWM_CHANNEL 0

#define MAX_PWM 150

float currentVelocity = 0.0; // current wheel velocity
float outputVelocity = 0.0; // PID output wheel velocity
float targetVelocity = 0.0; // wheel target velocity
float wheel_p = 1; // kP wheel value
float wheel_i = 0; // kI wheel value
float wheel_d = 0; // kD wheel value

int wheelDelay = 0;

bool wheelStop = false;
String wheelCode = "";


bool use_odrive = false;
bool odrive_available = false;

float getODriveVelocity() {
    Serial.print("r axis0.vel_estimate\n");
    return Serial.parseFloat();
}

void setODriveState() {
    Serial.print("w axis0.requested_state 8\n");
    Serial.print("u 0\n");
}

void setVelocity(float velocity) {
    Serial.print(String("v 0 ") + String(velocity) + String(" 0") + '\n');
}


// reads from the hall sensor, computes PID, and sends PWM to wheel
void wheelLoop() {
    
    if(odrive_available) {
        setODriveState();
        // currentVelocity = getODriveVelocity();
    }
    else {
        currentVelocity = 0;
    }

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
            outputVelocity = targetVelocity;
        }
        
        if(odrive_available) {
            // odrv.setVelocity(0, outputVelocity);
            setVelocity(outputVelocity);
        }
        else {
            ledcWrite(POWER_PWM_CHANNEL, abs(outputVelocity));
        }
        wheelCode = String("OK");
    }
    else {
        if(odrive_available) {
            setVelocity(0);
        }
        else {
            ledcWrite(POWER_PWM_CHANNEL, 0);
        }
        wheelCode = String("STOPPED");
    }
}




#endif