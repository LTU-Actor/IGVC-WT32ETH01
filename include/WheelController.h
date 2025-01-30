#pragma once

#ifndef WHEEL_CONTROLLER_H_
#define WHEEL_CONTROLLER_H_

#include <Arduino.h>
#include <ETH.h>
#include "MQTT.h"

// Pin definitions
#define POWER_PIN 14    // wheel throttle power
#define BRAKE_PIN 5     // wheel braking toggle
#define STEER_PIN 17    // steering angle
#define REVERSE_PIN 32  // reversing wheel direction
#define ESTOP_IN_PIN 35 // observing E-Stop loop
#define HALLA_PIN 15    // Hall A
#define HALLB_PIN 36    // Hall B
#define HALLC_PIN 39    // Hall C


#define MESSAGE_BUFFERSIZE 1024   // Maximum MQTT message size, in bytes

#define ESTOP_TIMEOUT_MILLIS 50     // Number of milliseconds to wait between messages to trigger an estop

// power pin callback, sends PWM to wheel
void powerCb(String& msg) {
    int pwm = msg.toInt();
    analogWrite(pwm, POWER_PIN);
}

// steer pin callback, sends PWM to steer servo
void steerCb(String& msg) {
    int pwm = msg.toInt();
    analogWrite(pwm, STEER_PIN);
}

// brake pin callback, sends digital to brake channel
void brakeCb(String& msg) {
    int pwm = (msg.toInt() == 0) ? 0 : 255;
    analogWrite(pwm, BRAKE_PIN);
}

void topicCb(String& topic, String& msg) {
    // this->estopCb(msg);
}

// establishes client connection to MQTT and sets callback
bool mqttConnect(const char* name, MQTTClient& client, void *callback(String&, String&)) {
    if (!ETH.linkUp()) {
        return false;
    }

    if (!client.connect(name)) {
        return false;
    }

    client.onMessage(callback);
    return true;
}




#endif