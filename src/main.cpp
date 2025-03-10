#include "WheelController.h"
#include "OTASetup.h"


#define LOOP_DELAY_MS 10 // how long to delay after each loop

// MQTT Setup
const IPAddress mqttAddress(192,168,0,3);
WiFiClient wc;






// sets up pins for input/output
void pinSetup() {
    ledcSetup(POWER_PWM_CHANNEL, 40000, 8); 
    ledcAttachPin(POWER_PIN, POWER_PWM_CHANNEL);
    ledcSetup(STEER_PWM_CHANNEL, 40000, 8);
    ledcAttachPin(STEER_PIN, STEER_PWM_CHANNEL);
    pinMode(BRAKE_PIN, OUTPUT);
    pinMode(REVERSE_PIN, OUTPUT);
}


void setup() {

  //Serial output setup
  Serial.begin(115200);
  while(!Serial);

  pinSetup();

  // Ethernet networking setup
  
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);

  while(!ETH.linkUp()) {
    Serial.println("waiting for ethernet...");
    delay(500);
  }

  ota_setup();


  // MQTT Connection
  client.begin(mqttAddress, 1883, wc);
  mqttConnect(client, topicCb);
  timeout = ESTOP_TIMEOUT_MILLIS;
}

void loop() {

  if(!client.connected()) {
    timeout = 0;
    // mqttConnect(client, topicCb);
  }
  else {
    if (timeout == 0) {
      // analogWrite(BRAKE_PIN, 0);
      String brake(0);
      brakeCb(brake);
      debug("estop activated, timed out after " + String(ESTOP_TIMEOUT_MILLIS) + "ms");
    }
    client.loop();
  }

  timeout = max(0, timeout - LOOP_DELAY_MS);
  delay(LOOP_DELAY_MS);
  ElegantOTA.loop();
  // Serial.println("loop");
}

