#include <Arduino.h>
#include <ETH.h>
#include <PubSubClient.h>

// Ethernet Setup Addresses
IPAddress deviceIP(10,42,0,10);
IPAddress deviceGW(10,42,0,1);
IPAddress deviceSN(255,255,255,0);
IPAddress deviceDNS(8,8,8,8);

// MQTT Setup
PubSubClient client;
IPAddress server(10,42,0,1);


void setup() {

  //Serial output setup
  Serial.begin(115200);
  while(!Serial);

  //Ethernet networking setup
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  ETH.config(deviceIP, deviceGW, deviceSN, deviceDNS);
  
  WiFiClient wt32client;
  client = PubSubClient(wt32client);
  client.setServer(server, 1883);

  
  
}

void loop() {
  if(client.connect("wt32")) {
    client.publish("test", "hello mqtt!");
  } 
  delay(1000);
  
  
}

