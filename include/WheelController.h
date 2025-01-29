#pragma once

#ifndef WHEEL_CONTROLLER_H_
#define WHEEL_CONTROLLER_H_

#include <Arduino.h>
#include <ETH.h>
#include "MQTT.h"

// Pin definitions
#define POWER_PIN 14    // Pin used for wheel throttle power
#define BRAKE_PIN 15    // Pin used for wheel braking toggle
#define STEER_PIN 17    // Pin used for steering angle
#define ESTOP_PIN 36    // Pin used to inform of E-Stop activation

#define MESSAGE_BUFFERSIZE 1024   // Maximum MQTT message size, in bytes

#define ESTOP_TIMEOUT_MILLIS 50     // Number of milliseconds to wait between messages to trigger an estop


class WheelController {

    private:

        MQTTClient client;
        bool softwareEstop;
        bool hardwareEstop;
        int timeout;

        // whether the bot should be e-stop
        bool estop() {
            return (softwareEstop || hardwareEstop);
        }

        // power pin callback, sends PWM to wheel
        void powerCb(String& topic, String& msg) {
            if(estop) { return; }
            int pwm = msg.toInt();
            analogWrite(pwm, POWER_PIN);
        }

        // steer pin callback, sends PWM to steer servo
        void steerCb(String& topic, String& msg) {
            if(estop) { return; }
            int pwm = msg.toInt();
            analogWrite(pwm, STEER_PIN);
        }

        // brake pin callback, sends digital to brake channel
        void brakeCb(String& topic, String& msg) {
            if(estop) { return; }
            int pwm = (msg.toInt() == 0) ? 0 : 255;
            analogWrite(pwm, BRAKE_PIN);
        }

        void estopCb(String& topic, String& msg) {
            // TODO
        }
 
        void connect() {

        }

    public:

        WheelController(IPAddress mqttAddress, int port, WiFiClient wc) {
            this->client = MQTTClient(MESSAGE_BUFFERSIZE);
            this->client.begin(mqttAddress, port, wc);
        }


};




#endif