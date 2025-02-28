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
#define BRAKE_PIN 15    // wheel braking toggle
#define STEER_PIN 17    // steering angle
#define REVERSE_PIN 4   // reversing wheel direction

#define POWER_PWM_CHANNEL 0 // LEDC Channel for power
#define STEER_PWM_CHANNEL 1 // LEDC Channel for steering

// MQTT definitions
#define MESSAGE_BUFFERSIZE 1024   // Maximum MQTT message size, in bytes
#define POWER_TOPIC "power" // mqtt topic name for power
#define BRAKE_TOPIC "brake" // mqtt topic name for braking
#define STEER_TOPIC "steer" // mqtt topic name for steering
#define DEBUG_TOPIC "debug" // mqtt topic name for debug info



#define ESTOP_TIMEOUT_MILLIS 50     // Number of milliseconds to wait between messages to trigger an estop

extern MQTTClient client(MESSAGE_BUFFERSIZE);
extern int timeout = 0;

bool direction = true;

// publishes a message on to the debug topic
void debug(const String& msg) { 
    if(!client.connected()) {
        return;
    }
    client.publish(("/" + String(CLIENT_NAME) + "/" + DEBUG_TOPIC), msg);
}

// power pin callback, sends PWM to wheel power
void powerCb(String& msg) {
    int pwm = msg.toInt();
    if(!direction && pwm >= 0) {
        direction = true;
        analogWrite(REVERSE_PIN, 0);
        ledcWrite(POWER_PWM_CHANNEL, 0);
        delay(10);
    }
    else if(direction && pwm < 0) {
        direction = false;
        analogWrite(REVERSE_PIN, 255);
        ledcWrite(POWER_PWM_CHANNEL, 0);
        pwm *= -1;
        delay(10);
    }
    ledcWrite(POWER_PWM_CHANNEL, pwm);
    
}

// steer pin callback, sends PWM to steer servo
void steerCb(String& msg) {
    int pwm = msg.toInt();
    ledcWrite(STEER_PWM_CHANNEL, pwm);
    
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






#endif