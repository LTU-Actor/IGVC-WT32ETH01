#include "WheelController.h"
#include "OTASetup.h"

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

    wheelPID.SetTunings(wheel_p, wheel_i, wheel_d);
    wheelPID.SetMode(1);
    steerPID.SetTunings(steer_p, steer_i, steer_d);
    steerPID.SetMode(1);

    wheelPID.SetOutputLimits(-255, 255);
    steerPID.SetOutputLimits(-100, 100);

    _delay(1000);
   
}

// runs the updaters for the wheel and steering
void deviceLoop() {
  steerLoop();
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

  clientSetup();
  ota_setup();

  // MQTT Connection
  client.begin(mqttAddress, 1883, wc);
  while(!mqttConnect(client, topicCb));
  timeout = ESTOP_TIMEOUT_MILLIS;

  
}

void loop() {


  if (timeout == 0) { // check if message timeout has triggered
    String brake(0);
    powerCb(brake);
    debug("estop activated on " + String(clientName) + ", timed out after " + String(ESTOP_TIMEOUT_MILLIS) + "ms");
  }

  if(!client.connected()) { // attempt to reconnect to MQTT
    timeout = 0;
    mqttConnect(client, topicCb);
  }
  else {
    client.loop();
  }

  // run loops, update timeout
  deviceLoop();
  infoLoop();
  ElegantOTA.loop();
  timeout = max(0, timeout - LOOP_DELAY_MS);
  delay(LOOP_DELAY_MS);
}

