#pragma once

#ifndef WHEEL_CONTROLLER_H_
#define WHEEL_CONTROLLER_H_

#include <Arduino.h>
#include <ETH.h>
#include "MQTT.h"
#include "CytronMotorDriver.h"
#include "AS5600.h"
#include "SimpleFOC.h"

// MQTT client name
#define CLIENT_NAME "testboard"

// Linear motor controller pin definitions
#define POWER_PIN  4   // wheel throttle power
#define POWER_DIR_PIN 5   // wheel throttle direction
#define BRAKE_PIN  3   // wheel braking toggle
#define HALL_A_PIN 12 // hall sensor A
#define HALL_B_PIN 14 // hall sensor B
#define HALL_C_PIN 15 // hall sensor C


// Steering motor controller pin definitions
#define STEER_PIN 2    // steering power
#define STEER_DIR_PIN 17 // steering direction
#define ENC_SCL_PIN 32 // steering encoder scl
#define ENC_SDA_PIN 33 // steering encoder sda



#define POWER_PWM_CHANNEL 0 // LEDC Channel for power

// MQTT definitions
#define MESSAGE_BUFFERSIZE 1024   // Maximum MQTT message size, in bytes
#define POWER_TOPIC "power" // mqtt topic name for power
#define BRAKE_TOPIC "brake" // mqtt topic name for braking
#define STEER_TOPIC "steer" // mqtt topic name for steering
#define DEBUG_TOPIC "debug" // mqtt topic name for debug info
#define ENC_DEBUG_TOPIC "encoder" // mqtt topic name for encoder debug info
#define HALL_VEL_TOPIC "velocity" // wheel velocity
#define HALL_ANGLE_TOPIC "angle" // wheel angle

#define ESTOP_TIMEOUT_MILLIS 50     // Number of milliseconds to wait between messages to trigger an estop

extern MQTTClient client(MESSAGE_BUFFERSIZE); // mqtt client
extern int timeout = 0; // time in millis until controls time out

CytronMD steerController(PWM_DIR, STEER_PIN, STEER_DIR_PIN); // steering controller for cytron MC
extern AS5600 as5600(&Wire); // encoder wire 0
extern HallSensor hall(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN, 14);

int target_speed = 0; // wheel target speed
int target_angle = 0; // wheel target angle

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
    target_speed = msg.toInt();
    
}

// steer pin callback, sends PWM to steer motor
void steerCb(String& msg) {
    // TODO: Use the Cytron package to arrive at steering angle with PID control
    target_angle = msg.toInt();
    steerController.setSpeed(target_angle);
    
}

// brake pin callback, sends digital to brake channel
void brakeCb(String& msg) {
    int pwm = (msg.toInt() == 0) ? 0 : 255;
    ledcWrite(POWER_PWM_CHANNEL, 0);
    analogWrite(BRAKE_PIN, pwm);
}

// general callback that sorts topics to their specific callbacks
void topicCb(String& topic, String& msg) {
    debug("got message on topic " + topic);
    if(topic.endsWith(POWER_TOPIC)) {
        powerCb(msg);
    }
    else if(topic.endsWith(BRAKE_TOPIC)) {
        brakeCb(msg);
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
    client.subscribe("/" + String(CLIENT_NAME) + "/" + BRAKE_TOPIC);
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
    double vel = hall.getVelocity();
    double angle = hall.getAngle();
    pub(String(vel), HALL_VEL_TOPIC);
    pub(String(angle), HALL_ANGLE_TOPIC);

    int a = digitalRead(HALL_A_PIN);
    int b = digitalRead(HALL_B_PIN);
    int c = digitalRead(HALL_C_PIN);

    pub(String("A: ") + String(a) + String("\nB: ") + String(b) + String("\nC: ") + String(c), "hall");
}



#endif