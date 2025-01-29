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
#define ESTOP_IN_PIN 36    // Pin used to take observe E-Stop loop

#define MESSAGE_BUFFERSIZE 1024   // Maximum MQTT message size, in bytes

#define ESTOP_TIMEOUT_MILLIS 50     // Number of milliseconds to wait between messages to trigger an estop


class WheelController {

    private:

        MQTTClient client;
        bool softwareEstop;     // whether the software estop was triggered
        bool hardwareEstop;     // whether the hardware estop was triggered
        int timeout;            // timeout millis counter
        String estopDebug;

        // whether the bot should be e-stop
        bool estop() {
            return (softwareEstop || hardwareEstop);
        }

        // power pin callback, sends PWM to wheel
        void powerCb(String& topic, String& msg) {
            if(estop) { return; }
            int pwm = msg.toInt();
            analogWrite(pwm, POWER_PIN);
            this->timeout = ESTOP_TIMEOUT_MILLIS;
        }

        // steer pin callback, sends PWM to steer servo
        void steerCb(String& topic, String& msg) {
            if(estop) { return; }
            int pwm = msg.toInt();
            analogWrite(pwm, STEER_PIN);
            this->timeout = ESTOP_TIMEOUT_MILLIS;
        }

        // brake pin callback, sends digital to brake channel
        void brakeCb(String& topic, String& msg) {
            if(estop) { return; }
            int pwm = (msg.toInt() == 0) ? 0 : 255;
            analogWrite(pwm, BRAKE_PIN);
            this->timeout = ESTOP_TIMEOUT_MILLIS;
        }

        // estop message callback
        void estopCb(String& topic, String& msg) {
            this->softwareEstop = true;
        }
 
        void connect() {
            // TODO: attempt to connect to MQTT, setup pub/sub
        }

        // pin logic for estop 
        void checkEStopLoop() {
            if(analogRead(ESTOP_IN_PIN) < 127) {    // if the estop loop is low
                this->hardwareEstop = true;
            }
            else {
                this->hardwareEstop = false;
            }
            if(estop()) {
                analogWrite(255, BRAKE_PIN);    // turn on brake
            }
            else {
                this->timeout = (this->timeout > 0) ? this->timeout-1 : 0;
                if(this->timeout == 0) {
                    this->softwareEstop = true;
                }
            }
        }

    public:

        WheelController(IPAddress mqttAddress, int port, WiFiClient wc) {
            this->client = MQTTClient(MESSAGE_BUFFERSIZE);
            this->client.begin(mqttAddress, port, wc);
        }

        // MQTT and estop check loop
        void loop() {
            if(!this->client.connected()) {
                this->softwareEstop = true;
                connect();
            }
            else {
                this->client.loop();
                checkEStopLoop();
            }
            delay(1);
        }


};




#endif