#pragma once

#ifndef WHEEL_CONTROLLER_H_
#define WHEEL_CONTROLLER_H_

#include <Arduino.h>
#include <ETH.h>
#include "MQTT.h"
#include "CytronMotorDriver.h"
#include "AS5600.h"
#include "SimpleFOC.h"
#include "PID_v1.h"

// MQTT client name
#define CLIENT_NAME "testboard"

// Linear motor controller pin definitions
#define POWER_PIN  12   // wheel throttle power
#define POWER_DIR_PIN 5   // wheel throttle direction
#define HALL_A_PIN 4 // hall sensor A
#define HALL_B_PIN 14 // hall sensor B
#define HALL_C_PIN 15 // hall sensor C


// Steering motor controller pin definitions
#define STEER_PIN 2    // steering power
#define STEER_DIR_PIN 17 // steering direction
#define ENC_SCL_PIN 32 // steering encoder scl
#define ENC_SDA_PIN 33 // steering encoder sda

// PID values
#define K_P 2
#define K_I 5
#define K_D 1

// MQTT definitions
#define MESSAGE_BUFFERSIZE 1024   // Maximum MQTT message size, in bytes
#define POWER_TOPIC "power" // mqtt topic name for power
#define STEER_TOPIC "steer" // mqtt topic name for steering
#define DEBUG_TOPIC "debug" // mqtt topic name for debug info
#define ENC_DEBUG_TOPIC "encoder" // mqtt topic name for encoder debug info
#define HALL_VEL_TOPIC "velocity" // wheel velocity
#define HALL_ANGLE_TOPIC "angle" // wheel angle

#define ESTOP_TIMEOUT_MILLIS 50     // Number of milliseconds to wait between messages to trigger an estop

extern MQTTClient client(MESSAGE_BUFFERSIZE); // mqtt client
extern int timeout = 0; // time in millis until controls time out

double currentVelocity = 0.0;
double outputVelocity = 0.0;
double targetVelocity = 0.0;
int targetAngle = 0;

CytronMD steerController(PWM_DIR, STEER_PIN, STEER_DIR_PIN); // steering controller for cytron MC
AS5600 as5600(&Wire); // encoder wire 0
HallSensor hall(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN, 14);
PID wheelController(&currentVelocity, &outputVelocity, &targetVelocity, K_P, K_I, K_D, DIRECT);



void pub(const String& msg, const char* topic) {
    if(!client.connected()) {
        return;
    }
    client.publish(("/" + String(CLIENT_NAME) + "/" + topic), msg);
}

// publishes a message on to the debug topic
void debug(const String& msg) { 
    pub(msg, DEBUG_TOPIC);
}



// power pin callback, sends PWM to wheel power
void powerCb(String& msg) {
    // TODO: Use hall feedback to do smooth acceleration and speed control
    targetVelocity = msg.toDouble();
    if(targetVelocity >= 0) {
        digitalWrite(POWER_DIR_PIN, LOW);
    }
    else {
        digitalWrite(POWER_DIR_PIN, HIGH);
        targetVelocity *= -1;
    }
}

// steer pin callback, sends PWM to steer motor
void steerCb(String& msg) {
    // TODO: Use the Cytron package to arrive at steering angle with PID control
    targetAngle = msg.toInt();
    steerController.setSpeed(targetAngle);
    
}

// general callback that sorts topics to their specific callbacks
void topicCb(String& topic, String& msg) {
    if(topic.endsWith(POWER_TOPIC)) {
        powerCb(msg);
    }
    else if(topic.endsWith(STEER_TOPIC)) {
        steerCb(msg);
    }
    timeout = ESTOP_TIMEOUT_MILLIS;
}


// establishes client connection to MQTT and sets callback
bool mqttConnect(MQTTClient& client, void callback(String&, String&)) {
    if (!ETH.linkUp()) {
        return false;
    }

    if (!client.connect(CLIENT_NAME)) {
        return false;
    }
    client.subscribe("/" + String(CLIENT_NAME) + "/" + POWER_TOPIC);
    client.subscribe("/" + String(CLIENT_NAME) + "/" + STEER_TOPIC);
    client.onMessage(callback);
    return true;
}


void steerLoop() {
    int wire0 = as5600.readAngle();
    pub(String(wire0), ENC_DEBUG_TOPIC);
}

void doA(){hall.handleA();}
void doB(){hall.handleB();}
void doC(){hall.handleC();}

void hallLoop() {
    hall.update();
    currentVelocity = hall.getVelocity();
    double angle = hall.getAngle();
    pub(String(currentVelocity), HALL_VEL_TOPIC);
    pub(String(angle), HALL_ANGLE_TOPIC);

    int a = digitalRead(HALL_A_PIN);
    int b = digitalRead(HALL_B_PIN);
    int c = digitalRead(HALL_C_PIN);

    pub(String("A: ") + String(a) + String("\nB: ") + String(b) + String("\nC: ") + String(c), "hall");
}

void wheelLoop() {
    debug(String("Computed Velocity: ") + String(outputVelocity));
    wheelController.Compute();
}



#endif