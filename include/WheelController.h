#pragma once

#ifndef WHEEL_CONTROLLER_H_
#define WHEEL_CONTROLLER_H_

#include <Arduino.h>
#include <ETH.h>
#include "MQTT.h"
#include <ArduinoJson.h>

#include "Steering.h"
#include "Wheel.h"


/*
    Pin Notes
    - IO35, IO36, IO39 are very unstable input pins
    - IO12 must be low/floating when the device boots
    - IO32, 33 must be used for the encoder's I2C
    - IO5 goes high for a moment when booting 

*/

// MQTT definitions
#define MESSAGE_BUFFERSIZE 1024   // Maximum MQTT message size, in bytes

// input
#define POWER_TOPIC "power" // mqtt topic name for power, wheel-specific
#define STEER_TOPIC "steer" // mqtt topic name for steering, wheel-specific
#define PID_TOPIC "pid" // PID values, as JSON, wheel-agnostic
#define CALIB_STEER_TOPIC "calibrate" // Steering calibration topic, wheel-agnostic

// output
#define DEBUG_TOPIC "debug" //  debug info
#define ENCODER_TOPIC "encoder" // encoder angle
#define HALL_VEL_TOPIC "velocity" // wheel velocity
#define HALL_TOPIC "hall" // raw hall values


#define ESTOP_TIMEOUT_MILLIS 50     // Number of milliseconds to wait between messages to trigger an estop

extern MQTTClient client(MESSAGE_BUFFERSIZE); // mqtt client
extern int timeout = 0; // time in millis until controls time out


String clientName() {
    String mac = ETH.macAddress();
    if (mac == String("A8:48:FA:08:59:57")) {
        return "frontleft";
    }
    if (mac == String("A8:48:FA:08:81:67")) {
        return "frontright";
    }
    if (mac == String("A8:48:FA:08:57:F3")) {
        return "backleft";
    }
    if (mac == String("A8:03:2A:20:C7:9B")) {
        return "backright";
    }
    if (mac == String("94:3C:C6:39:CD:8B")) {
        return "testboard";
    }
    return "error";
}

// publishes a message on a topic
void pub(const String& msg, const char* topic) {
    if(!client.connected()) {
        return;
    }
    client.publish(("/" + clientName() + "/" + topic), msg);
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

// steering angle callback
void steerCb(String& msg) {
    targetAngle = msg.toDouble();
}

// PID tuning callback
void PIDCb(String& msg) {
    try {
        JsonDocument js;
        deserializeJson(js, msg);

        wheel_p = js["linear"]["x"];
        wheel_i = js["linear"]["y"];
        wheel_d = js["linear"]["z"];

        steer_p = js["angular"]["x"];
        steer_i = js["angular"]["y"];
        steer_d = js["angular"]["z"];

        wheelPID.SetTunings(wheel_p, wheel_i, wheel_d);
        steerPID.SetTunings(steer_p, steer_i, steer_d);
        debug("Set PID values successfully!");
    }
    catch(std::exception e) {
        debug("Could not set PID values. Reason: " + String(e.what()));
    }
}

// general callback that sorts topics to their specific callbacks
void topicCb(String& topic, String& msg) {
    if(topic.endsWith(POWER_TOPIC)) {
        powerCb(msg);
    }
    else if(topic.endsWith(STEER_TOPIC)) {
        steerCb(msg);
    }
    else if(topic.endsWith(PID_TOPIC)) {
        debug("here");
        PIDCb(msg);
    }
    else if(topic.endsWith(CALIB_STEER_TOPIC)) {
        setSteerCenter();
    }
    timeout = ESTOP_TIMEOUT_MILLIS; // reset timeout
}


// establishes client connection to MQTT and sets callback
bool mqttConnect(MQTTClient& client, void callback(String&, String&)) {
    if (!ETH.linkUp()) {
        return false;
    }

    if (!client.connect(clientName().c_str())) {
        return false;
    }
    client.subscribe("/" + clientName() + "/" + POWER_TOPIC);
    client.subscribe("/" + clientName() + "/" + STEER_TOPIC);
    client.subscribe("/" + String(PID_TOPIC));
    client.subscribe("/" + String(CALIB_STEER_TOPIC));
    client.onMessage(callback);
    return true;
}

void infoLoop() {
    pub(String(currentAngle), ENCODER_TOPIC);
    pub(String(currentVelocity), HALL_VEL_TOPIC);
    pub(String("A: ") + String(hallA) + String("\nB: ") + String(hallB) + String("\nC: ") + String(hallC), HALL_TOPIC);
    pub(String(outputAngle), "angleSpeed");
}



#endif