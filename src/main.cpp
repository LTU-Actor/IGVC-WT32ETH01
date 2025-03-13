#include "WheelController.h"

/*
94:3c:c6:39:cd:8b --- frontleft

*/

#define LOOP_DELAY_MS 10 // how long to delay after each loop

// MQTT Setup
const IPAddress mqttAddress(192,168,0,3);
WiFiClient wc;


// sets up pins for input/output
void pinSetup() {
    ledcSetup(POWER_PWM_CHANNEL, 40000, 8);
    ledcAttachPin(POWER_PIN, POWER_PWM_CHANNEL);
    pinMode(POWER_DIR_PIN, OUTPUT);
    pinMode(BRAKE_PIN, OUTPUT);
    pinMode(STEER_PIN, OUTPUT);
    pinMode(STEER_DIR_PIN, OUTPUT);

    Wire.begin();
    as5600.begin(ENC_SCL_PIN);  //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

    // pinMode(HALL_A_PIN, INPUT);
    // pinMode(HALL_B_PIN, INPUT);
    // pinMode(HALL_C_PIN, INPUT);
    hall.pullup = Pullup::USE_INTERN;
    hall.init();
    hall.enableInterrupts(doA, doB, doC);
    _delay(1000);
   
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
    // analogWrite(BRAKE_PIN, 0);
    String brake(0);
    brakeCb(brake);
    debug("estop activated, timed out after " + String(ESTOP_TIMEOUT_MILLIS) + "ms");
  }

  if(!client.connected()) {
    timeout = 0;
    mqttConnect(client, topicCb);
  }
  else {
    client.loop();
  }

  // hall.update();
  steerLoop();
  hallLoop();
  
  timeout = max(0, timeout - LOOP_DELAY_MS);
  delay(LOOP_DELAY_MS);
  
}

