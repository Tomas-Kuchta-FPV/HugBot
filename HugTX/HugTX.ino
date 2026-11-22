#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "secrets.h"

constexpr uint8_t LEFT_JOYSTICK_PIN = 0;
constexpr uint8_t RIGHT_JOYSTICK_PIN = 3;
constexpr uint8_t OLED_SDA_PIN = 8;
constexpr uint8_t OLED_SCL_PIN = 9;
constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr uint32_t OLED_UPDATE_INTERVAL_MS = 50;
constexpr uint32_t PACKET_MAGIC = 0x48554742;
constexpr uint32_t TRANSMIT_INTERVAL_MS = 20;
constexpr uint32_t CALIBRATION_MAGIC = 0x43414C42;
constexpr uint16_t CALIBRATION_VERSION = 1;
constexpr int ADC_MIN_VALUE = 0;
constexpr int ADC_MAX_VALUE = 4095;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t sequence;
  int16_t leftY;
  int16_t rightY;
} ControlPacket;

static_assert(sizeof(ControlPacket) == 12, "ControlPacket must remain 12 bytes");

ControlPacket myData{};
esp_now_peer_info_t peerInfo;
Preferences calibrationPreferences;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

uint32_t lastTransmitTime = 0;
uint32_t packetSequence = 0;
uint32_t lastPrintTime = 0;
uint32_t lastDisplayUpdateTime = 0;
bool radioReady = false;
bool calibrationInProgress = false;
bool calibrationStorageReady = false;
bool displayReady = false;

// Initial ADC calibration
int leftMin = 0, leftCenter = 2048, leftMax = 4095;
int rightMin = 0, rightCenter = 2048, rightMax = 4095;

struct CalibrationRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  int32_t leftMin;
  int32_t leftCenter;
  int32_t leftMax;
  int32_t rightMin;
  int32_t rightCenter;
  int32_t rightMax;
};

bool isCalibrationValid(const CalibrationRecord &record) {
  return record.leftMin >= ADC_MIN_VALUE && record.leftMin < record.leftCenter &&
         record.leftCenter < record.leftMax && record.leftMax <= ADC_MAX_VALUE &&
         record.rightMin >= ADC_MIN_VALUE && record.rightMin < record.rightCenter &&
         record.rightCenter < record.rightMax && record.rightMax <= ADC_MAX_VALUE;
}

bool loadCalibration() {
  if (!calibrationStorageReady) {
    return false;
  }

  if (calibrationPreferences.getBytesLength("calibration") != sizeof(CalibrationRecord)) {
    return false;
  }

  CalibrationRecord record{};
  if (calibrationPreferences.getBytes("calibration", &record, sizeof(record)) != sizeof(record) ||
      record.magic != CALIBRATION_MAGIC ||
      record.version != CALIBRATION_VERSION ||
      !isCalibrationValid(record)) {
    Serial.println("Saved calibration is invalid; using default calibration.");
    return false;
  }

  leftMin = record.leftMin;
  leftCenter = record.leftCenter;
  leftMax = record.leftMax;
  rightMin = record.rightMin;
  rightCenter = record.rightCenter;
  rightMax = record.rightMax;
  return true;
}

bool saveCalibration() {
  if (!calibrationStorageReady) {
    return false;
  }

  CalibrationRecord record{
      CALIBRATION_MAGIC,
      CALIBRATION_VERSION,
      0,
      leftMin,
      leftCenter,
      leftMax,
      rightMin,
      rightCenter,
      rightMax};
  return calibrationPreferences.putBytes("calibration", &record, sizeof(record)) == sizeof(record);
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

void drawJoystickBar(int y, char label, int value) {
  constexpr int BAR_X = 52;
  constexpr int BAR_Y_OFFSET = -1;
  constexpr int BAR_WIDTH = 76;
  constexpr int BAR_HEIGHT = 12;

  value = constrain(value, -1000, 1000);
  display.setCursor(0, y);
  display.print(label);
  display.setCursor(10, y);
  display.print(value / 10);
  display.print('%');

  const int barY = y + BAR_Y_OFFSET;
  const int centerX = BAR_X + BAR_WIDTH / 2;
  const int barLength = map(abs(value), 0, 1000, 0, BAR_WIDTH / 2 - 2);
  display.drawRect(BAR_X, barY, BAR_WIDTH, BAR_HEIGHT, SSD1306_WHITE);
  display.drawFastVLine(centerX, barY, BAR_HEIGHT, SSD1306_WHITE);
  if (barLength > 0) {
    const int fillX = value < 0 ? centerX - barLength : centerX + 1;
    display.fillRect(fillX, barY + 2, barLength, BAR_HEIGHT - 4, SSD1306_WHITE);
  }
}

void updateDisplay(int leftValue, int rightValue) {
  if (!displayReady) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("HugBot TX");
  drawJoystickBar(20, 'L', leftValue);
  drawJoystickBar(43, 'R', rightValue);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(OLED_ADDRESS);
  if (Wire.endTransmission() == 0 &&
      display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    displayReady = true;
    updateDisplay(0, 0);
  } else {
    Serial.println("OLED initialization failed; continuing without display.");
  }

  calibrationStorageReady = calibrationPreferences.begin("hugbot", false);
  if (!calibrationStorageReady) {
    Serial.println("Failed to initialize calibration storage; changes will not persist.");
  } else if (loadCalibration()) {
    Serial.println("Loaded calibration from non-volatile storage.");
  } else {
    Serial.println("No valid saved calibration found; using defaults.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
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
      const CalibrationRecord previousCalibration{
          CALIBRATION_MAGIC,
          CALIBRATION_VERSION,
          0,
          leftMin,
          leftCenter,
          leftMax,
          rightMin,
          rightCenter,
          rightMax};
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
      CalibrationRecord calibrated{
          CALIBRATION_MAGIC,
          CALIBRATION_VERSION,
          0,
          leftMin,
          leftCenter,
          leftMax,
          rightMin,
          rightCenter,
          rightMax};
      if (!isCalibrationValid(calibrated)) {
        leftMin = previousCalibration.leftMin;
        leftCenter = previousCalibration.leftCenter;
        leftMax = previousCalibration.leftMax;
        rightMin = previousCalibration.rightMin;
        rightCenter = previousCalibration.rightCenter;
        rightMax = previousCalibration.rightMax;
        Serial.println("Calibration was invalid; previous calibration retained.");
      } else if (saveCalibration()) {
        Serial.println("Calibration saved to non-volatile storage.");
      } else {
        Serial.println("Failed to save calibration; it will be lost after reboot.");
      }
      Serial.println("Normal operation resumed.\n");
      calibrationInProgress = false;
    }
  }

  if (!radioReady || calibrationInProgress) {
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

    if (millis() - lastDisplayUpdateTime >= OLED_UPDATE_INTERVAL_MS) {
      lastDisplayUpdateTime = millis();
      updateDisplay(myData.leftY, myData.rightY);
    }

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
