#include "WheelController.h"
#include "OTASetup.h"

#define LOOP_DELAY_MS 5 // how long to delay after each loop

// MQTT Setup
const IPAddress mqttAddress(192,168,0,3);
WiFiClient wc;


// sets up pins for input/output
void deviceSetup() {
    
    pinMode(POWER_DIR_PIN, OUTPUT);
    pinMode(STEER_PIN, OUTPUT);
    pinMode(STEER_DIR_PIN, OUTPUT);

    Wire.begin();
    as5600.begin(ENC_SCL_PIN);  //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
    
    if(use_odrive) {
      Serial.setTimeout(1);
      Serial.setRxTimeout(1);
      odrv.setTimeout(1);
      Serial.begin(115200);
    }

    ledcSetup(POWER_PWM_CHANNEL, 40000, 8);
    ledcAttachPin(POWER_PIN, POWER_PWM_CHANNEL);

    steerPID.SetTunings(steer_p, steer_i, steer_d);
    steerPID.SetMode(1);

    steerPID.SetOutputLimits(-100, 100);

    delay(1000);
   
}

// runs the updaters for the wheel and steering
void deviceLoop() {
  steerLoop();
  wheelLoop();
}


void setup() {
 
  // Ethernet networking setup
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  ETH.setHostname(clientName.c_str());
  while(!ETH.linkUp());

  clientSetup();
  ota_setup();

  // MQTT Connection
  client.begin(mqttAddress, 1883, wc);
  while(!mqttConnect(client, topicCb));

  deviceSetup();
  timeout = ESTOP_TIMEOUT_MILLIS;
  
}

void loop() {

  if(!client.connected()) { // attempt to reconnect to MQTT
    timeout = 0;
    triggerStop();
    deviceLoop();
    mqttConnect(client, topicCb);
  }
  else {
    client.loop();
  }

  odriveConnect();
  // run loops, update timeout
  deviceLoop();
  infoLoop();
  ElegantOTA.loop();
  timeout = max(0, timeout - LOOP_DELAY_MS);

  if (timeout == 0) { // check if message timeout has triggered
    triggerStop();
  }

  // debug(String(odrv_serial.isListening()));
  debug(String("Using ODrive: ") + String(odrive_available));
  delay(LOOP_DELAY_MS);
}

