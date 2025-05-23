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
#define DIRECTION_TOPIC "direction"  // wheel direction topic, wheel-specific

// output
#define DEBUG_TOPIC "debug" //  debug info
#define ENCODER_TOPIC "encoder" // encoder angle
#define HALL_VEL_TOPIC "velocity" // wheel velocity
#define HALL_TOPIC "hall" // raw hall values

#define ESTOP_ACTIVATION_PIN 4 // pin for hardware estop, HIGH=normal, LOW=stop


#define ESTOP_TIMEOUT_MILLIS 50     // Number of milliseconds to wait between messages to trigger an estop

MQTTClient client(MESSAGE_BUFFERSIZE); // mqtt client
int timeout = 0; // time in millis until controls time out
String clientName;
bool in_reverse = false;

void clientSetup() {
    String mac = ETH.macAddress();
    if (mac == String("A8:48:FA:08:57:F3")) {
        clientName =  "frontleft";
        setSteerCenterValue(1561.0);
        use_odrive = true;
    }
    else if (mac == String("A8:48:FA:08:81:67")) {
        clientName = "frontright";
        setSteerCenterValue(2125.0);
        use_odrive = true;
    }
    else if (mac == String("A8:48:FA:08:59:57")) { // a8:48:fa:08:57:f3, 192.168.0.7
        clientName = "backleft";
        setSteerCenterValue(1822.0);
    }
    else if (mac == String("A8:03:2A:20:C7:9B")) { // a8:03:2a:20:c7:9b, 192.168.0.8
        clientName = "backright";
        setSteerCenterValue(2584.0);
    }
    else if (mac == String("94:3C:C6:39:CD:8B")) {
        clientName = "testboard";
    }
    else {
        clientName = "error";
    }
}

// publishes a message on a topic
void pub(const String& msg, const char* topic) {
    if(!client.connected()) {
        return;
    }
    client.publish(("/" + clientName + "/" + topic), msg);
}

// publishes a message on to the debug topic
void debug(const String& msg) { 
    pub(msg, DEBUG_TOPIC);
}

void triggerStop() {
    wheelStop = true;
    steerStop = true;
}

void releaseStop() {
    wheelStop = false;
    steerStop = false;
}

// power pin callback, sends PWM to wheel power
void powerCb(String& msg) {
    targetVelocity = msg.toFloat();
}

void directionCb(String& msg) {
    int dir = msg.toInt();
    if(in_reverse && dir == 0) {
        if(currentVelocity != 0) {
            targetVelocity = 0;
        }
        else {
            digitalWrite(POWER_DIR_PIN, LOW);
            in_reverse = false;
            wheelDelay = 100;
        }
    }
    else if(!in_reverse && dir == 1) {
        if(currentVelocity != 0) {
            targetVelocity = 0;
        }
        else {
            digitalWrite(POWER_DIR_PIN, HIGH);
            in_reverse = true;
            wheelDelay = 100;
        }
    }
}

// steering angle callback
void steerCb(String& msg) {
    targetAngle = filterEnc((int) radToEnc(msg.toFloat()));
}

// PID tuning callback
void PIDCb(String& msg) {
    try {
        JsonDocument js;
        deserializeJson(js, msg);

        wheel_p = js["linear"]["x"].as<float>();
        wheel_i = js["linear"]["y"].as<float>();
        wheel_d = js["linear"]["z"].as<float>();

        steer_p = js["angular"]["x"].as<float>();
        steer_i = js["angular"]["y"].as<float>();
        steer_d = js["angular"]["z"].as<float>();

        steerPID.SetTunings(steer_p, steer_i, steer_d);
    }
    catch(std::exception e) {
        debug("Could not set PID values. Reason: " + String(e.what()));
    }
}

// general callback that sorts topics to their specific callbacks
void topicCb(String& topic, String& msg) {
    if(topic.endsWith(POWER_TOPIC)) {
        powerCb(msg);
        timeout = ESTOP_TIMEOUT_MILLIS; // reset timeout
        releaseStop();
    }
    else if(topic.endsWith(STEER_TOPIC)) {
        steerCb(msg);
        timeout = ESTOP_TIMEOUT_MILLIS; // reset timeout
        releaseStop();
    }
    else if(topic.endsWith(PID_TOPIC)) {
        PIDCb(msg);
    }
    else if(topic.endsWith(CALIB_STEER_TOPIC)) {
        setSteerCenter();
    }
    else if(topic.endsWith(DIRECTION_TOPIC)) {
        directionCb(msg);
    }
    
}

// reads for an estop on the hardware pin
void estop_pin_check() {
    if(digitalRead(ESTOP_ACTIVATION_PIN) == LOW) {
        triggerStop();
    }
}


// establishes client connection to MQTT and sets callback
bool mqttConnect(MQTTClient& client, void callback(String&, String&)) {
    if (!ETH.linkUp()) {
        return false;
    }

    if (!client.connect(clientName.c_str())) {
        return false;
    }
    client.subscribe("/" + clientName + "/" + POWER_TOPIC);
    client.subscribe("/" + clientName + "/" + STEER_TOPIC);
    client.subscribe("/" + clientName + "/" + DIRECTION_TOPIC);
    client.subscribe("/" + String(PID_TOPIC));
    client.subscribe("/" + String(CALIB_STEER_TOPIC));
    client.onMessage(callback);
    return true;
}

void odriveConnect() {
    if(use_odrive) {
        Serial.print("r vbus_voltage\n");
        float voltage = Serial.parseFloat();
        if(voltage == 0) {
        //   odrv_serial.end();
          odrive_available = false;
        }
        else {
          // // clear the rx buffer
          // while(odrv_serial.available()) {
          //   debug(String("!!!!!!!!!!!!!!!!!!!!   ") + String(odrv_serial.read()));
          // }
          debug(String("!!!!!!!!!!!!!!!!!!!!   ") + String(voltage));
          odrive_available = true;
        }
    }
}

void infoLoop() {
    debug(String("\nName: ") + clientName +
          String("\nWheel: ") + wheelCode +
          String("\nSteer: ") + steerCode +
          String("\nUsing ODrive: ") + String(odrive_available)
    );
    pub(String(encToRad(encRaw)), ENCODER_TOPIC);
    pub(String(encRaw) + String(", ") + String(lastSteerError), "encoder_raw");
    pub(String(radToDegree(encToRad(encRaw))), "encoder_deg");
    pub(String(outputVelocity), "power_output");
    pub(String(currentVelocity), HALL_VEL_TOPIC);
}




#endif