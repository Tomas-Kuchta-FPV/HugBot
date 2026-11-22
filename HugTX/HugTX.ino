#include <esp_now.h>
#include <WiFi.h>
#include "secrets.h"

constexpr uint8_t LED_PIN = 8; // Active-low status LED
constexpr uint8_t LEFT_JOYSTICK_PIN = 0;
constexpr uint8_t RIGHT_JOYSTICK_PIN = 3;
constexpr uint32_t PACKET_MAGIC = 0x48554742;
constexpr uint32_t TRANSMIT_INTERVAL_MS = 20;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t sequence;
  int16_t leftY;
  int16_t rightY;
} ControlPacket;

static_assert(sizeof(ControlPacket) == 12, "ControlPacket must remain 12 bytes");

ControlPacket myData{};
esp_now_peer_info_t peerInfo;

uint32_t lastTransmitTime = 0;
uint32_t packetSequence = 0;
uint32_t lastPrintTime = 0;
bool radioReady = false;
bool calibrationInProgress = false;

// Initial ADC calibration
int leftMin = 0, leftCenter = 2048, leftMax = 4095;
int rightMin = 0, rightCenter = 2048, rightMax = 4095;

void OnDataSent(const esp_now_send_info_t *txInfo, esp_now_send_status_t status) {
  (void)txInfo;
  digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? LOW : HIGH);
}

// Map a joystick using its calibrated center instead of assuming ADC center.
int mapJoystick(int raw, int minVal, int centerVal, int maxVal) {
  if (minVal >= maxVal || centerVal <= minVal || centerVal >= maxVal) {
    return 0;
  }

  if (raw < centerVal) {
    return constrain(map(raw, minVal, centerVal, -1000, 0), -1000, 0);
  }

  return constrain(map(raw, centerVal, maxVal, 0, 1000), 0, 1000);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  if (esp_now_register_send_cb(OnDataSent) != ESP_OK) {
    Serial.println("Failed to register send callback");
    return;
  }

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    return;
  }

  radioReady = true;

  Serial.println("\n=========================================");
  Serial.println("        TX BATTLEBOT CONTROLLER");
  Serial.println("=========================================");
  Serial.println("Current calibration (Min / Center / Max):");
  Serial.printf("Left Y : %d / %d / %d\n", leftMin, leftCenter, leftMax);
  Serial.printf("Right Y: %d / %d / %d\n", rightMin, rightCenter, rightMax);
  Serial.println("-----------------------------------------");
  Serial.println("To calibrate, type 'calibrate' and press Enter.");
  Serial.println("=========================================\n");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (!calibrationInProgress && (cmd == "calibrate" || cmd == "calibration")) {
      calibrationInProgress = true;
      Serial.println("\n--- CALIBRATION STARTED ---");
      Serial.println("STEP 1: Release both joysticks. Do not touch them.");
      Serial.println("Measuring center position for 2 seconds...");
      delay(2000);

      long sumL = 0, sumR = 0;
      for (int i = 0; i < 10; ++i) {
        sumL += analogRead(LEFT_JOYSTICK_PIN);
        sumR += analogRead(RIGHT_JOYSTICK_PIN);
        delay(10);
      }
      leftCenter = sumL / 10;
      rightCenter = sumR / 10;

      Serial.printf("Center saved -> Left: %d | Right: %d\n", leftCenter, rightCenter);
      Serial.println("\nSTEP 2: Move both joysticks through all extreme positions for 5 seconds.");

      leftMin = leftCenter;
      leftMax = leftCenter;
      rightMin = rightCenter;
      rightMax = rightCenter;

      uint32_t calibStart = millis();
      while (millis() - calibStart < 5000) {
        int rawL = analogRead(LEFT_JOYSTICK_PIN);
        int rawR = analogRead(RIGHT_JOYSTICK_PIN);

        if (rawL < leftMin) leftMin = rawL;
        if (rawL > leftMax) leftMax = rawL;
        if (rawR < rightMin) rightMin = rawR;
        if (rawR > rightMax) rightMax = rawR;
        delay(5);
      }

      Serial.println("\n--- CALIBRATION COMPLETE ---");
      Serial.printf("Left Y (min/center/max) : %d / %d / %d\n", leftMin, leftCenter, leftMax);
      Serial.printf("Right Y (min/center/max): %d / %d / %d\n", rightMin, rightCenter, rightMax);
      Serial.println("Normal operation resumed.\n");
      calibrationInProgress = false;
    }
  }

  if (!radioReady || calibrationInProgress) {
    if (radioReady) {
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, HIGH);
    }
    return;
  }

  if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL_MS) {
    lastTransmitTime = millis();

    int rawLeft = analogRead(LEFT_JOYSTICK_PIN);
    int rawRight = analogRead(RIGHT_JOYSTICK_PIN);

    myData.magic = PACKET_MAGIC;
    myData.sequence = ++packetSequence;
    myData.leftY = static_cast<int16_t>(mapJoystick(rawLeft, leftMin, leftCenter, leftMax));
    myData.rightY = static_cast<int16_t>(mapJoystick(rawRight, rightMin, rightCenter, rightMax));

    esp_now_send(broadcastAddress, reinterpret_cast<const uint8_t *>(&myData), sizeof(myData));

    if (millis() - lastPrintTime >= 250) {
      lastPrintTime = millis();
      Serial.print("TX sending -> Left: ");
      Serial.print(myData.leftY / 10.0f, 1);
      Serial.print("% | Right: ");
      Serial.print(myData.rightY / 10.0f, 1);
      Serial.println("%");
    }
  }
}
