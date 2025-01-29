#include <Arduino.h>
#include <ETH.h>
#include "MQTT.h"

#define HOSTNAME "wt32-frontleft"   // Network hostname for the board

// Ethernet Setup Addresses
IPAddress deviceIP(10,42,0,10);
IPAddress deviceGW(10,42,0,1);
IPAddress deviceSN(255,255,255,0);
IPAddress deviceDNS = deviceIP;

// MQTT Setup
MQTTClient client(MESSAGE_BUFFERSIZE);
IPAddress mqttAddress(10,42,0,4);
WiFiClient wc;

int lastMillis = 0;
int pwm_signal = 0;

void relay_message(String &topic, String &message) {
  String relayTopic = topic + "_relay";
  client.publish(relayTopic, message);
}

void pwm_message(String &topic, String &message) {
  try {
    pwm_signal = message.toInt();
  }
  catch(std::exception e) {
    Serial.println(e.what());
    return;
  }
}

void connect() {
  Serial.print("checking wifi...");
  while (!ETH.linkUp()) {
    Serial.print("!");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("wt32")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");
  client.onMessage(pwm_message);
  client.subscribe("test");

}


void setup() {

  //Serial output setup
  Serial.begin(115200);
  while(!Serial);

  pinMode(15, OUTPUT);

  //Ethernet networking setup
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  ETH.config(deviceIP, deviceGW, deviceSN, deviceDNS);

  client.begin(mqttAddress, 1883, wc);
  connect();

  client.onMessage(pwm_message);
  client.subscribe("test");

  analogWriteFrequency(1000);
  
}

void loop() {
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  analogWrite(15, pwm_signal);
  Serial.println(pwm_signal);
  
}

