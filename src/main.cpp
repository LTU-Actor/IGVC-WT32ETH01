#include "WheelController.h"

/*
94:3c:c6:39:cd:8b --- frontleft

*/

// MQTT Setup
const IPAddress mqttAddress(192,168,0,3);
WiFiClient wc;



// sets up pins for input/output
void pinSetup() {
    pinMode(POWER_PIN, OUTPUT);
    pinMode(BRAKE_PIN, OUTPUT);
    pinMode(STEER_PIN, OUTPUT);
    pinMode(REVERSE_PIN, OUTPUT);

    pinMode(ESTOP_IN_PIN, INPUT);
    pinMode(HALLA_PIN, INPUT);
    pinMode(HALLB_PIN, INPUT);
    pinMode(HALLC_PIN, INPUT);
}

void estopLoop() {
  hardwareEstop = analogRead(ESTOP_IN_PIN) < 127;
  if(estop()) {
    analogWrite(BRAKE_PIN, 255);
  }
}


void setup() {

  //Serial output setup
  Serial.begin(115200);
  while(!Serial);

  pinSetup();

  // Ethernet networking setup
  
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  while(!ETH.linkUp());


  // MQTT Connection
  client.begin(mqttAddress, 1883, wc);
  while(!mqttConnect(client, topicCb));
  timeout = ESTOP_TIMEOUT_MILLIS;
}

void loop() {

  if (timeout == 0) {
    softwareEstop = true;
    debug("estop activated, timed out after " + String(ESTOP_TIMEOUT_MILLIS) + "ms");
  }
  estopLoop();

  if(!client.connected()) {
    softwareEstop = true;
    mqttConnect(client, topicCb);
  }
  else {
    client.loop();
  }

  timeout = (timeout == 0) ? 0 : timeout-1;
  delay(10);
  
}

