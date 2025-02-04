#pragma once

#ifndef WHEEL_CONTROLLER_H_
#define WHEEL_CONTROLLER_H_

#include <Arduino.h>
#include <ETH.h>
#include "MQTT.h"

// MQTT client name
#define CLIENT_NAME "frontleft"

// Pin definitions
#define POWER_PIN 14    // wheel throttle power
#define BRAKE_PIN 12    // wheel braking toggle
#define STEER_EN_PIN 5  // steering angle
#define STEER_PIN 17    // steering angle
#define REVERSE_PIN 4   // reversing wheel direction
#define ESTOP_IN_PIN 35 // observing E-Stop loop
#define HALLA_PIN 15    // Hall A
#define HALLB_PIN 36    // Hall B
#define HALLC_PIN 39    // Hall C

// MQTT definitions
#define MESSAGE_BUFFERSIZE 1024   // Maximum MQTT message size, in bytes
#define POWER_TOPIC "power"
#define BRAKE_TOPIC "brake"
#define STEER_TOPIC "steer"
#define ESTOP_TOPIC "estop"
#define HALLA_TOPIC "hallA"
#define HALLB_TOPIC "hallB"
#define HALLC_TOPIC "hallC"
#define DEBUG_TOPIC "debug"

#define ESTOP_TIMEOUT_MILLIS 50     // Number of milliseconds to wait between messages to trigger an estop

extern MQTTClient client(MESSAGE_BUFFERSIZE);
extern bool softwareEstop = false;
extern bool hardwareEstop = false;
extern int timeout = 0;

bool estop() {
    return (softwareEstop || hardwareEstop);
}

// publishes a message on to the debug topic
void debug(const String& msg) {
    if(!client.connected()) {
        return;
    }
    client.publish(("/" + String(CLIENT_NAME) + "/" + DEBUG_TOPIC), msg);
}

// power pin callback, sends PWM to wheel
void powerCb(String& msg) {
    Serial.println("got power message: " + msg);
    debug("got power message: " + msg);
    int pwm = msg.toInt();
    if(pwm < 0) {
        analogWrite(REVERSE_PIN, 255);
        analogWrite(POWER_PIN, pwm * -1);
    }
    else {
        analogWrite(REVERSE_PIN, 255);
        analogWrite(POWER_PIN, pwm);
    }
    
}

// steer pin callback, sends PWM to steer servo
void steerCb(String& msg) {
    int pwm = msg.toInt();
    analogWrite(STEER_PIN, pwm);
    
}

// brake pin callback, sends digital to brake channel
void brakeCb(String& msg) {
    int pwm = (msg.toInt() == 0) ? 0 : 255;
    analogWrite(BRAKE_PIN, pwm);
}

// estop pin callback
void estopCb(String& msg) {
    softwareEstop = (msg == "false") ? false : true;
}

// general callback that sorts topics to their specific callbacks
void topicCb(String& topic, String& msg) {
    Serial.println("got message on topic " + topic);
    debug("got message on topic " + topic);
    if(topic.endsWith(ESTOP_TOPIC)) {
        estopCb(msg);
    }
    else if(estop()) {
        if(topic.endsWith(POWER_TOPIC)) {
            powerCb(msg);
        }
        else if(topic.endsWith(BRAKE_TOPIC)) {
            brakeCb(msg);
        }
        else if(topic.endsWith(STEER_TOPIC)) {
            steerCb(msg);
        }
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
    client.subscribe("/" + String(CLIENT_NAME) + "/" + ESTOP_TOPIC);
    client.onMessage(callback);
    return true;
}






#endif