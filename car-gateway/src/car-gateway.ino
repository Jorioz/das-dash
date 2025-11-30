/*  ============================================================================

     MrDIY.ca CAN Project

    CAN Gateway

    code for MrDIY CAN Shield & DOIT ESP32 DEVKIT V1 (30 pins version)
    https://store.mrdiy.ca/p/esp32-can-bus-shield/


        ***********************  IMPORTANT NOTE ****************************

        For shields v1.0, v1.1, and v1.2, use D5 for CAN RX and D4 for CAN TX
        For shields v1.3 and later, use D4 for CAN RX and D5 for CAN TX



  ============================================================================= */

#define FIRMWARE_VER  "3.1203"


// ---- serial debug -------------------

// uncomment #define DEBUG_FLAG  1 to debug or troubleshoot issues
// leave it commented for normal use

#define DEBUG_FLAG  1
//#define PRINT_CAN_FLAG  1

#ifdef DEBUG_FLAG
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#include <Arduino.h>


// --- shield pins ---------------------

#define SHIELD_LED_PIN  26
#define SHIELD_CAN_RX   4
#define SHIELD_CAN_TX   5

#define SHIELD_VOLTAGE_DIVIDER 32 /* the jumper must be soldered on the v1.1 shield. Voltage divider doesn't work/exist on v1.0 */

uint32_t led_last_on_timestamp = 0;
uint32_t currentMillis;

// --- can ---------------------------

#include "driver/twai.h"
#define POLLING_RATE_MS 500

// --- espnow ----------------------

#include <esp_now.h>
#include <WiFi.h>
int esp_now_mesh_id;
uint8_t broadcastAddress[] = { 0x6C, 0xC8, 0x40, 0x59, 0x4D, 0x48 };

// ---- sleep logic -----------------
unsigned long last_can_msg_timestamp = 0;
#define CAN_IDLE_TIMEOUT 5000    /* duration esp32 has to wait before going to sleep after no CAN avitivites in ( in milliseconds) */

unsigned long last_espnow_msg_timestamp = 0;
#define ESPNOW_IDLE_TIMEOUT 5000 // 5 seconds

#define SLEEP_PERIOD 6          /* duration ESP32 will sleep for (in seconds) */

#define CAR_IS_RUNNING 1
#define CAR_IS_OFF 0
int car_status = CAR_IS_OFF;

// --- message timing ---
unsigned long lastIncrementTime = 0;
unsigned int secondsSinceLastMessage = 0;


typedef struct esp_now_frame_t {
  int mesh_id;
  unsigned long can_id;
  byte len;
  uint8_t d[8];
};

esp_now_frame_t esp_now_frame;
esp_now_peer_info_t peerInfo;

void on_data_recv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  last_espnow_msg_timestamp = 0;
  secondsSinceLastMessage = 0; // Reset seconds counter when data is received
  esp_now_frame_t *frame = (esp_now_frame_t *)incomingData;
  printCAN(frame);
  Serial.println();

  if (frame->len > 0 && frame->len <= 8){
    write_to_can(frame);
  } else {
    Serial.println("Got Invalid CAN Frame from ESP-NOW");
  }
}

void write_to_can(const esp_now_frame_t* frame){
  twai_message_t message = {};
  message.identifier = frame->can_id;
  message.data_length_code = frame->len;
  memcpy(message.data, frame->d, frame->len);
  message.flags = 0;

  if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
        Serial.println("Frame sent to CAN bus.");
  } else {
        Serial.println("Failed to send frame to CAN bus.");
  }
}

/* ================================== Setup ======================================== */

void setup() {

#ifdef DEBUG_FLAG
  Serial.begin(115200);
#endif

  pinMode(SHIELD_LED_PIN, OUTPUT);

  // flash the LED
  digitalWrite(SHIELD_LED_PIN, HIGH);

  debugln("");
  debugln("------------------------");
  debugln("    MrDIY CAN SHIELD");
  debugln("------------------------");

  last_can_msg_timestamp = millis() - CAN_IDLE_TIMEOUT + 5;

  if (!initESPNow() || !initCAN()) {
    debugln(" SYSTEM......... FAIL");
    debugln(" ... restarting");
    delay(60 * 1000);
    ESP.restart();
  }

  digitalWrite(SHIELD_LED_PIN, HIGH);
  esp_now_register_recv_cb(on_data_recv);
}

bool initESPNow() {

  debugln(" ESPNOW............INIT");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    debugln(" ESPNOW.............FAIL");
    return false;
  }
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    debugln(" ESPNOW.............FAIL");
    return false;
  }
  debugln(" ESPNOW..............OK");

  esp_now_mesh_id = getMeshID();
  debug(" MESH ID..........");
  debugln(esp_now_mesh_id);

  return true;
}

// at 1mbps number of CAN msgs are in (9259 - 22727) range

bool initCAN() {

  debugln(" CAN...............INIT");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)SHIELD_CAN_TX, (gpio_num_t)SHIELD_CAN_RX, TWAI_MODE_LISTEN_ONLY);
  g_config.rx_queue_len = 32;
  g_config.tx_queue_len = 2;  // not needed but here for testing

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    debugln(" CAN....Install......OK");

  } else {
    debugln(" CAN....Driver....FAIL");
    return false;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    debugln(" CAN....Start........OK");

  } else {
    debugln(" CAN....Start......FAIL");
    return false;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    debugln(" CAN....Alerts.......OK");
  } else {
    debugln(" CAN....Alerts.....FAIL");
    return false;
  }
  debugln(" CAN.................OK");
  return true;
}

int getMeshID() {

  uint8_t baseMac[6];
  WiFi.macAddress(baseMac);
  uint32_t uniqueID = 0;
  for (int i = 2; i < 6; i++) {
    uniqueID <<= 8;
    uniqueID |= baseMac[i];
  }
  // Limit the integer to the maximum value of an int (32,767)
  uniqueID %= 32768;
  return (int)uniqueID;
}
/* ================================== Loop ======================================== */

void loop() {

  currentMillis = millis();
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

  if (alerts_triggered) {
    // Check if message is received and handle it (them)
    if (TWAI_ALERT_RX_DATA) {
      twai_message_t message;
      while (twai_receive(&message, 0) == ESP_OK) {
        handle_rx_message(message);
      }
      last_can_msg_timestamp = millis();
      // Do NOT return here; allow sleep check to always run
    }

    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    bool ledShouldBeOn = false;
    if (TWAI_ALERT_ERR_PASS) {
      debugln("Alert: TWAI controller has become error passive.");
      ledShouldBeOn = true;
      debugln(" CAN MSG..........ERROR");
    }
    if (TWAI_ALERT_BUS_ERROR) {
      debugln("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      ledShouldBeOn = true;
      debugln(" CAN MSG......BUS ERROR");
    }
    if (TWAI_ALERT_RX_QUEUE_FULL) {
      debugln("Alert: The RX queue is full causing a received frame to be lost.");
      ledShouldBeOn = true;
      debugln(" CAN MSG.........Q FULL");
    }
    if (ledShouldBeOn) {
      digitalWrite(SHIELD_LED_PIN, HIGH);
      led_last_on_timestamp = currentMillis;
    }
  }

  // Increment secondsSinceLastMessage every second if no data received
  if (millis() - lastIncrementTime >= 1000) {
    lastIncrementTime = millis();
    secondsSinceLastMessage++;
    if (secondsSinceLastMessage >= 5) {
      Serial.println("No message for 5 seconds. Entering deep sleep.");
      esp_deep_sleep();
    }
  }
}


static void handle_rx_message(twai_message_t& message) {
  printFrame(message);
  esp_now_frame.mesh_id = esp_now_mesh_id;
  esp_now_frame.can_id = message.identifier;
  esp_now_frame.len = message.data_length_code;
  memcpy(esp_now_frame.d, message.data, message.data_length_code);
  esp_now_send(broadcastAddress, (uint8_t*)&esp_now_frame, sizeof(esp_now_frame));
  debugln(" CAN MSG.............OK");
}

/* ================================== Power ======================================== */

void reduceHeat() {

  btStop();  // make sure BT is disabled
  setCpuFrequencyMhz(80);
  WiFi.setTxPower(WIFI_POWER_11dBm);
}

void esp_deep_sleep() {

  esp_sleep_enable_timer_wakeup(SLEEP_PERIOD * 1000000);
  debugln(" ESP32........... SLEEP");
  debugln("------------------------");
  esp_deep_sleep_start();
}


void update_car_status() {

  // ONLY works for 12v Lead Acid car batteries
  // currently not doing anything, simply updating the car_status and turning on/off the LED

  int avg_voltage = get_vin_voltage();

  if (avg_voltage > 3500) {
    car_status = CAR_IS_RUNNING;
    digitalWrite(SHIELD_LED_PIN, HIGH);

  } else if (avg_voltage > 3250) {
    // gray area, do nothing

  } else {
    car_status = CAR_IS_OFF;
    digitalWrite(SHIELD_LED_PIN, LOW);
    // if( avg_voltage  < 3150 ) battery is less than 70% - danger area, good idea to put ESP32 to sleep
  }
}

double get_vin_voltage() {

  /*
    MEASSURED ADC VALUES for a given voltage
      Volt    ADC
      10.5    2640
      11.31   2850
      11.58   2930
      11.75   2970
      11.9    3000
      12.06   3060
      12.2    3100
      12.32   3150
      12.42   3170
      12.5    3190
      12.6    3220 ---- max 3250 for car to be off

      |||||||||||||||||  gray area   |||||||||||||||

      13.5    3520 ---- min 3500 for car to be running
      14.7    4030
  */

  // 10 samples = 2ms, 50 samples = 10ms
  int average = 0;
  for (int i = 0; i < 10; i++) average = average + analogRead(SHIELD_VOLTAGE_DIVIDER);
  average = average / 10;
  return average;
}

void printFrame(twai_message_t& message) {

  if (message.extd) {
    Serial.print("CAX: 0x");
  } else {
    Serial.print("CAN: 0x");
  }
  Serial.print(message.identifier, HEX);
  Serial.print(" (");
  Serial.print(message.identifier, DEC);
  Serial.print(")");
  Serial.print(" [");
  Serial.print(message.data_length_code, DEC);
  Serial.print("] <");
  for (int i = 0; i < message.data_length_code; i++) {
    if (i != 0) Serial.print(":");
    Serial.print(message.data[i], HEX);
  }
  Serial.println(">");
}

void printCAN(esp_now_frame_t* message) {

  Serial.print("ESP ");
  Serial.print(message->mesh_id);
  Serial.print(": ");
  Serial.print(" 0x");
  Serial.print(message->can_id, HEX);
  Serial.print(" (");
  Serial.print(message->can_id, DEC);
  Serial.print(")");
  Serial.print(" [");
  Serial.print(message->len, DEC);
  Serial.print("] <");
  for (int i = 0; i < message->len; i++) {
    if (i != 0) Serial.print(":");
    Serial.print(message->d[i], HEX);
  }
  Serial.print(">");
}