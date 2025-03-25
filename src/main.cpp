#include "WheelController.h"
#include "OTASetup.h"

/*
94:3c:c6:39:cd:8b --- frontleft

*/

#define LOOP_DELAY_MS 10 // how long to delay after each loop

// MQTT Setup
const IPAddress mqttAddress(192,168,0,3);
WiFiClient wc;


// sets up pins for input/output
void deviceSetup() {
    ledcSetup(POWER_PWM_CHANNEL, 40000, 8);
    ledcAttachPin(POWER_PIN, POWER_PWM_CHANNEL);
    pinMode(POWER_DIR_PIN, OUTPUT);
    pinMode(STEER_PIN, OUTPUT);
    pinMode(STEER_DIR_PIN, OUTPUT);

    Wire.begin();
    as5600.begin(ENC_SCL_PIN);  //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

    hall.pullup = Pullup::USE_INTERN;
    hall.init();
    hall.enableInterrupts(doA, doB, doC);

    wheelController.SetMode(AUTOMATIC);

    _delay(1000);
   
}

void deviceLoop() {
  steerLoop();
  hallLoop();
  wheelLoop();
}


void setup() {

  //Serial output setup
  Serial.begin(115200);
  while(!Serial);

  deviceSetup();
  
  // Ethernet networking setup
  
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  while(!ETH.linkUp());

  // MQTT Connection
  client.begin(mqttAddress, 1883, wc);
  while(!mqttConnect(client, topicCb));
  timeout = ESTOP_TIMEOUT_MILLIS;

  ota_setup();
}

void loop() {

  if (timeout == 0) {
    String brake(0);
    powerCb(brake);
    debug("estop activated, timed out after " + String(ESTOP_TIMEOUT_MILLIS) + "ms");
  }

  if(!client.connected()) {
    timeout = 0;
    mqttConnect(client, topicCb);
  }
  else {
    client.loop();
  }

  deviceLoop();
  timeout = max(0, timeout - LOOP_DELAY_MS);
  delay(LOOP_DELAY_MS);
  ElegantOTA.loop();
  
}

