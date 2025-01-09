#include <Arduino.h>
#include <ETH.h>
#include "MQTT.h"

// Ethernet Setup Addresses
IPAddress deviceIP(10,42,0,10);
IPAddress deviceGW(10,42,0,1);
IPAddress deviceSN(255,255,255,0);
IPAddress deviceDNS = deviceIP;

// MQTT Setup
MQTTClient client;
IPAddress mqttAddress(10,42,0,4);
WiFiClient wc;

int lastMillis = 0;

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

  // client.subscribe("/hello");
  // client.unsubscribe("/hello");
}


void setup() {

  //Serial output setup
  Serial.begin(115200);
  while(!Serial);

  //Ethernet networking setup
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  ETH.config(deviceIP, deviceGW, deviceSN, deviceDNS);

  client.begin(mqttAddress, 1883, wc);
  connect();
  
}

void loop() {
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  // publish a message roughly every second.
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    client.publish("/hello", "world");
  }
  
  
}

