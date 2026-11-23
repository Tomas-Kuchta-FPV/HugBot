#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TxTypes.h"
#include "secrets.h"

constexpr uint8_t STICK_1_Y_PIN = 0;
constexpr uint8_t STICK_1_X_PIN = 1;
constexpr uint8_t STICK_2_Y_PIN = 3;
constexpr uint8_t STICK_2_X_PIN = 4;
constexpr uint8_t BATTERY_VOLTAGE_PIN = 2;
constexpr uint8_t OLED_SDA_PIN = 8;
constexpr uint8_t OLED_SCL_PIN = 9;
constexpr uint8_t BUTTON_1_PIN = 10;
constexpr uint8_t BUTTON_2_PIN = 21;
constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr uint32_t OLED_UPDATE_INTERVAL_MS = 50;
constexpr uint32_t BATTERY_READ_INTERVAL_MS = 500;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t SETTINGS_HOLD_MS = 1000;
constexpr uint32_t FAILSAFE_TIMEOUT_MS = 500;
constexpr uint32_t RIGHT_STICK_GRACE_MS = 500;
constexpr int MENU_STICK_THRESHOLD = 450;
constexpr int MENU_STICK_RELEASE_THRESHOLD = 250;
constexpr uint32_t MENU_NAV_REPEAT_MS = 300;
constexpr uint32_t VALUE_ADJUST_SLOW_REPEAT_MS = 300;
constexpr uint32_t VALUE_ADJUST_FAST_REPEAT_MS = 40;
constexpr uint32_t PACKET_MAGIC = 0x48554742;
constexpr uint32_t TRANSMIT_INTERVAL_MS = 20;
constexpr uint32_t CALIBRATION_MAGIC = 0x43414C42;
constexpr uint16_t CALIBRATION_VERSION = 2;
constexpr int ADC_MIN_VALUE = 0;
constexpr int ADC_MAX_VALUE = 4095;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr float DEFAULT_BATTERY_VOLTAGE_MULTIPLIER = 2.0f;
constexpr float BATTERY_MULTIPLIER_MIN = 1.0f;
constexpr float BATTERY_MULTIPLIER_MAX = 10.0f;
constexpr float BATTERY_MULTIPLIER_STEP = 0.01f;
constexpr uint8_t SETTINGS_ITEM_COUNT = 6;
constexpr uint8_t PRECODED_MODEL_COUNT = sizeof(modelList) / sizeof(modelList[0]);
static_assert(PRECODED_MODEL_COUNT > 0, "At least one model must be configured");

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t sequence;
  int16_t leftY;
  int16_t rightY;
} ControlPacket;

static_assert(sizeof(ControlPacket) == 12, "ControlPacket must remain 12 bytes");

ControlPacket myData{};
Preferences calibrationPreferences;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

uint32_t lastTransmitTime = 0;
uint32_t packetSequence = 0;
uint32_t lastPrintTime = 0;
uint32_t lastDisplayUpdateTime = 0;
uint32_t lastBatteryReadTime = 0;
uint32_t lastMenuNavigationTime = 0;
uint32_t lastValueAdjustTime = 0;
uint32_t rightStickGraceUntil = 0;
uint32_t controlGraceUntil = 0;
volatile uint32_t lastSendStatusTime = 0;
volatile bool sendStatusKnown = false;
volatile bool lastSendSucceeded = false;
int leftXValue = 0;
int leftYValue = 0;
int rightXValue = 0;
int rightYValue = 0;
bool radioReady = false;
bool calibrationInProgress = false;
bool calibrationStorageReady = false;
bool displayReady = false;
float batteryVoltageMultiplier = DEFAULT_BATTERY_VOLTAGE_MULTIPLIER;
float editedBatteryVoltageMultiplier = DEFAULT_BATTERY_VOLTAGE_MULTIPLIER;
float batterySenseVoltage = 0.0f;
float batteryVoltage = 0.0f;
uint8_t activeModelSelection = 0;
uint8_t editedModelSelection = 0;

void OnDataSent(const esp_now_send_info_t *txInfo, esp_now_send_status_t status) {
  (void)txInfo;
  lastSendSucceeded = status == ESP_NOW_SEND_SUCCESS;
  lastSendStatusTime = millis();
  sendStatusKnown = true;
}

bool hasRecentAcknowledgement() {
  return sendStatusKnown && lastSendSucceeded &&
         millis() - lastSendStatusTime <= FAILSAFE_TIMEOUT_MS;
}

const uint8_t *getModelAddress(uint8_t index) {
  return modelList[index].mac;
}

const char *getModelName(uint8_t index) {
  return modelList[index].friendlyName;
}

struct AxisCalibration {
  uint8_t pin;
  int min;
  int center;
  int max;
};

AxisCalibration leftYCalibration{STICK_1_Y_PIN, 0, 2048, 4095};
AxisCalibration leftXCalibration{STICK_1_X_PIN, 0, 2048, 4095};
AxisCalibration rightYCalibration{STICK_2_Y_PIN, 0, 2048, 4095};
AxisCalibration rightXCalibration{STICK_2_X_PIN, 0, 2048, 4095};

ButtonState button1{BUTTON_1_PIN};
ButtonState button2{BUTTON_2_PIN};

enum class ScreenMode {
  HOME,
  SETTINGS,
  BATTERY_MULTIPLIER,
  CONTROL_MODE,
  MODEL_SELECTION,
  ERASE_CONFIRM
};

ScreenMode screenMode = ScreenMode::HOME;
ControlMode controlMode = ControlMode::TANK;
ControlMode editedControlMode = ControlMode::TANK;
uint8_t settingsSelection = 0;
uint32_t bothButtonsPressedAt = 0;
bool bothButtonsHoldHandled = false;
int menuNavigationDirection = 0;
int valueAdjustDirection = 0;
bool rightStickActionReady = true;

bool isAxisCalibrationValid(int minValue, int centerValue, int maxValue) {
  return minValue >= ADC_MIN_VALUE && minValue < centerValue &&
         centerValue < maxValue && maxValue <= ADC_MAX_VALUE;
}

bool isCalibrationValid(const CalibrationRecord &record) {
  return isAxisCalibrationValid(record.leftYMin, record.leftYCenter, record.leftYMax) &&
         isAxisCalibrationValid(record.leftXMin, record.leftXCenter, record.leftXMax) &&
         isAxisCalibrationValid(record.rightYMin, record.rightYCenter, record.rightYMax) &&
         isAxisCalibrationValid(record.rightXMin, record.rightXCenter, record.rightXMax);
}

CalibrationRecord currentCalibration() {
  return CalibrationRecord{
      CALIBRATION_MAGIC,
      CALIBRATION_VERSION,
      0,
      leftYCalibration.min, leftYCalibration.center, leftYCalibration.max,
      leftXCalibration.min, leftXCalibration.center, leftXCalibration.max,
      rightYCalibration.min, rightYCalibration.center, rightYCalibration.max,
      rightXCalibration.min, rightXCalibration.center, rightXCalibration.max};
}

void applyCalibration(const CalibrationRecord &record) {
  leftYCalibration.min = record.leftYMin;
  leftYCalibration.center = record.leftYCenter;
  leftYCalibration.max = record.leftYMax;
  leftXCalibration.min = record.leftXMin;
  leftXCalibration.center = record.leftXCenter;
  leftXCalibration.max = record.leftXMax;
  rightYCalibration.min = record.rightYMin;
  rightYCalibration.center = record.rightYCenter;
  rightYCalibration.max = record.rightYMax;
  rightXCalibration.min = record.rightXMin;
  rightXCalibration.center = record.rightXCenter;
  rightXCalibration.max = record.rightXMax;
}

void loadBatteryMultiplier() {
  if (!calibrationStorageReady) {
    return;
  }

  const float savedMultiplier = calibrationPreferences.getFloat(
      "voltMult", DEFAULT_BATTERY_VOLTAGE_MULTIPLIER);
  if (!(savedMultiplier >= BATTERY_MULTIPLIER_MIN &&
        savedMultiplier <= BATTERY_MULTIPLIER_MAX)) {
    Serial.println("Saved battery multiplier is invalid; using the default.");
    return;
  }
  batteryVoltageMultiplier = savedMultiplier;
}

bool saveBatteryMultiplier(float multiplier) {
  if (!calibrationStorageReady) {
    return false;
  }

  return calibrationPreferences.putFloat("voltMult", multiplier) == sizeof(float);
}

void loadControlMode() {
  if (!calibrationStorageReady) {
    return;
  }

  const uint8_t savedMode = calibrationPreferences.getUChar(
      "driveMode", static_cast<uint8_t>(ControlMode::TANK));
  if (savedMode > static_cast<uint8_t>(ControlMode::MIXED)) {
    Serial.println("Saved control mode is invalid; using tank steering.");
    return;
  }
  controlMode = static_cast<ControlMode>(savedMode);
}

bool saveControlMode(ControlMode mode) {
  if (!calibrationStorageReady) {
    return false;
  }

  return calibrationPreferences.putUChar("driveMode", static_cast<uint8_t>(mode)) == sizeof(uint8_t);
}

void loadModelSelection() {
  if (!calibrationStorageReady) {
    return;
  }

  const uint8_t savedSelection = calibrationPreferences.getUChar("modelIdx", 0);
  if (savedSelection >= PRECODED_MODEL_COUNT) {
    Serial.println("Saved model selection is invalid; using the first model.");
    return;
  }
  activeModelSelection = savedSelection;
}

bool addEspNowPeer(const uint8_t *address) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, address, 6);
  peer.channel = 1;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

bool saveModelSelection(uint8_t selection) {
  if (!calibrationStorageReady ||
      selection >= PRECODED_MODEL_COUNT) {
    return false;
  }
  return calibrationPreferences.putUChar("modelIdx", selection) == sizeof(uint8_t);
}

bool selectModel(uint8_t selection) {
  if (selection >= PRECODED_MODEL_COUNT) {
    return false;
  }

  const uint8_t *oldAddress = getModelAddress(activeModelSelection);
  const uint8_t *newAddress = getModelAddress(selection);
  const bool addressChanged = memcmp(oldAddress, newAddress, 6) != 0;
  const bool peerAlreadyExists = esp_now_is_peer_exist(newAddress);
  if (addressChanged && !peerAlreadyExists &&
      !addEspNowPeer(newAddress)) {
    Serial.println("Failed to add selected model as an ESP-NOW peer.");
    return false;
  }

  if (!saveModelSelection(selection)) {
    if (addressChanged && !peerAlreadyExists) {
      esp_now_del_peer(newAddress);
    }
    Serial.println("Failed to save model selection; selection not applied.");
    return false;
  }

  if (addressChanged && esp_now_is_peer_exist(oldAddress) &&
      esp_now_del_peer(oldAddress) != ESP_OK) {
    Serial.println("Could not remove the previous ESP-NOW peer.");
  }
  activeModelSelection = selection;
  return true;
}

bool eraseSavedSettings() {
  if (!calibrationStorageReady) {
    return false;
  }

  const uint8_t *oldAddress = getModelAddress(activeModelSelection);
  const uint8_t *defaultAddress = getModelAddress(0);
  const bool addressChanged = memcmp(oldAddress, defaultAddress, 6) != 0;
  const bool defaultPeerAlreadyExists = esp_now_is_peer_exist(defaultAddress);
  if (addressChanged && !defaultPeerAlreadyExists &&
      !addEspNowPeer(defaultAddress)) {
    Serial.println("Failed to restore the default ESP-NOW peer.");
    return false;
  }
  if (!calibrationPreferences.clear()) {
    if (addressChanged && !defaultPeerAlreadyExists) {
      esp_now_del_peer(defaultAddress);
    }
    return false;
  }
  if (addressChanged && esp_now_is_peer_exist(oldAddress) &&
      esp_now_del_peer(oldAddress) != ESP_OK) {
    Serial.println("Could not remove the previous ESP-NOW peer.");
  }

  leftYCalibration = {STICK_1_Y_PIN, 0, 2048, 4095};
  leftXCalibration = {STICK_1_X_PIN, 0, 2048, 4095};
  rightYCalibration = {STICK_2_Y_PIN, 0, 2048, 4095};
  rightXCalibration = {STICK_2_X_PIN, 0, 2048, 4095};
  batteryVoltageMultiplier = DEFAULT_BATTERY_VOLTAGE_MULTIPLIER;
  editedBatteryVoltageMultiplier = batteryVoltageMultiplier;
  controlMode = ControlMode::TANK;
  editedControlMode = controlMode;
  activeModelSelection = 0;
  editedModelSelection = 0;
  updateBatteryVoltage();
  return true;
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

  applyCalibration(record);
  return true;
}

bool saveCalibration() {
  if (!calibrationStorageReady) {
    return false;
  }

  const CalibrationRecord record = currentCalibration();
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

void drawJoystick(int centerX, int centerY, const char *label, int xValue, int yValue) {
  constexpr int RADIUS = 17;
  constexpr int DOT_RADIUS = 2;
  const int dotX = centerX + map(constrain(xValue, -1000, 1000), -1000, 1000, -13, 13);
  const int dotY = centerY - map(constrain(yValue, -1000, 1000), -1000, 1000, -13, 13);

  display.setCursor(centerX - 3, 11);
  display.print(label);
  display.drawCircle(centerX, centerY, RADIUS, SSD1306_WHITE);
  display.drawFastHLine(centerX - RADIUS + 3, centerY, (RADIUS - 3) * 2, SSD1306_WHITE);
  display.drawFastVLine(centerX, centerY - RADIUS + 3, (RADIUS - 3) * 2, SSD1306_WHITE);
  display.fillCircle(dotX, dotY, DOT_RADIUS, SSD1306_WHITE);
}

void drawButtonBadge(int x, int y, const char *label, bool pressed) {
  if (pressed) {
    display.fillRoundRect(x, y, 20, 8, 2, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.drawRoundRect(x, y, 20, 8, 2, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(x + 4, y);
  display.print(label);
  display.setTextColor(SSD1306_WHITE);
}

void drawHeader() {
  display.setCursor(0, 0);
  display.print("TX");
  display.setCursor(22, 0);
  display.print(hasRecentAcknowledgement() ? "ACK" : "FAILSAFE");
  display.setCursor(88, 0);
  const float displayedBatteryVoltage =
      screenMode == ScreenMode::BATTERY_MULTIPLIER
          ? batterySenseVoltage * editedBatteryVoltageMultiplier
          : batteryVoltage;
  display.print(displayedBatteryVoltage, 2);
  display.print('V');
}

void updateDisplay() {
  if (!displayReady) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  drawHeader();

  if (screenMode == ScreenMode::HOME) {
    drawJoystick(32, 37, "L", leftXValue, leftYValue);
    drawJoystick(96, 37, "R", rightXValue, rightYValue);
    drawButtonBadge(35, 56, "B1", button1.pressed);
    drawButtonBadge(73, 56, "B2", button2.pressed);
  } else if (screenMode == ScreenMode::SETTINGS) {
    display.setCursor(0, 8);
    display.print("Settings");
    const char *items[] = {
        "Model select", "Control mode", "Calibrate sticks",
        "Battery factor", "Erase settings", "Back"};
    for (uint8_t i = 0; i < SETTINGS_ITEM_COUNT; ++i) {
      display.setCursor(0, 16 + i * 8);
      display.print(i == settingsSelection ? '>' : ' ');
      display.print(items[i]);
    }
  } else if (screenMode == ScreenMode::BATTERY_MULTIPLIER) {
    display.setCursor(0, 17);
    display.print("Battery factor");
    display.setCursor(35, 29);
    display.print(editedBatteryVoltageMultiplier, 2);
    display.print('x');
  } else if (screenMode == ScreenMode::CONTROL_MODE) {
    display.setCursor(0, 14);
    display.print("Control mode");
    display.setCursor(0, 27);
    display.print(editedControlMode == ControlMode::TANK ? "> Tank" : "  Tank");
    display.setCursor(0, 38);
    display.print(editedControlMode == ControlMode::MIXED ? "> Mixed" : "  Mixed");
  } else if (screenMode == ScreenMode::MODEL_SELECTION) {
    display.setCursor(0, 14);
    display.print("Select model");
    display.setCursor(0, 29);
    const char *modelName = getModelName(editedModelSelection);
    for (uint8_t i = 0; modelName[i] != '\0' && i < 21; ++i) {
      display.print(modelName[i]);
    }
    display.setCursor(0, 44);
    display.print(editedModelSelection + 1);
    display.print('/');
    display.print(PRECODED_MODEL_COUNT);
  } else {
    display.setCursor(0, 14);
    display.print("Erase all settings?");
    display.setCursor(0, 29);
    display.print("Right: erase");
    display.setCursor(0, 40);
    display.print("Left: cancel");
  }
  display.display();
}

void showCalibrationStatus(const char *message) {
  if (!displayReady) {
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  drawHeader();
  display.setCursor(0, 28);
  display.print(message);
  display.display();
}

void updateBatteryVoltage() {
  uint32_t totalMillivolts = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    totalMillivolts += analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
  }
  batterySenseVoltage = (totalMillivolts / 4.0f) / 1000.0f;
  batteryVoltage = batterySenseVoltage * batteryVoltageMultiplier;
}

void updateButton(ButtonState &button) {
  const bool rawPressed = digitalRead(button.pin) == LOW;
  button.justPressed = false;
  if (rawPressed != button.rawPressed) {
    button.rawPressed = rawPressed;
    button.rawChangedAt = millis();
  }
  if (button.rawPressed != button.pressed &&
      millis() - button.rawChangedAt >= BUTTON_DEBOUNCE_MS) {
    button.pressed = button.rawPressed;
    button.justPressed = button.pressed;
  }
}

void runCalibration();

int getMenuDirection(int horizontal, int vertical) {
  if (abs(horizontal) > abs(vertical)) {
    return horizontal > MENU_STICK_THRESHOLD ? -1 :
           horizontal < -MENU_STICK_THRESHOLD ? 1 : 0;
  }
  return vertical > MENU_STICK_THRESHOLD ? -1 :
         vertical < -MENU_STICK_THRESHOLD ? 1 : 0;
}

void serviceButtons() {
  updateButton(button1);
  updateButton(button2);

  const bool bothPressed = button1.pressed && button2.pressed;
  if (!bothPressed) {
    bothButtonsPressedAt = 0;
    bothButtonsHoldHandled = false;
  } else if (bothButtonsPressedAt == 0) {
    bothButtonsPressedAt = millis();
  } else if (!bothButtonsHoldHandled &&
             millis() - bothButtonsPressedAt >= SETTINGS_HOLD_MS) {
    bothButtonsHoldHandled = true;
    if (screenMode == ScreenMode::HOME) {
      screenMode = ScreenMode::SETTINGS;
      settingsSelection = 0;
      menuNavigationDirection = 0;
      valueAdjustDirection = 0;
      rightStickActionReady = true;
    } else if (screenMode == ScreenMode::SETTINGS) {
      screenMode = ScreenMode::HOME;
      controlGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
    }
  }

}

void serviceJoystickMenu() {
  if (screenMode == ScreenMode::HOME ||
      (button1.pressed && button2.pressed)) {
    return;
  }

  const int leftHorizontal = leftXValue;
  const int navigationDirection =
      getMenuDirection(leftHorizontal, leftYValue);
  if (abs(leftHorizontal) < MENU_STICK_RELEASE_THRESHOLD &&
      abs(leftYValue) < MENU_STICK_RELEASE_THRESHOLD) {
    menuNavigationDirection = 0;
  }

  if (screenMode == ScreenMode::SETTINGS && navigationDirection != 0 &&
      (navigationDirection != menuNavigationDirection ||
       millis() - lastMenuNavigationTime >= MENU_NAV_REPEAT_MS)) {
    settingsSelection =
        (settingsSelection + SETTINGS_ITEM_COUNT + navigationDirection) %
        SETTINGS_ITEM_COUNT;
    menuNavigationDirection = navigationDirection;
    lastMenuNavigationTime = millis();
  } else if (screenMode == ScreenMode::CONTROL_MODE && navigationDirection != 0 &&
             (navigationDirection != menuNavigationDirection ||
              millis() - lastMenuNavigationTime >= MENU_NAV_REPEAT_MS)) {
    editedControlMode = navigationDirection == -1
        ? ControlMode::TANK
        : ControlMode::MIXED;
    menuNavigationDirection = navigationDirection;
    lastMenuNavigationTime = millis();
  } else if (screenMode == ScreenMode::MODEL_SELECTION &&
             navigationDirection != 0 &&
             (navigationDirection != menuNavigationDirection ||
              millis() - lastMenuNavigationTime >= MENU_NAV_REPEAT_MS)) {
    editedModelSelection = static_cast<uint8_t>(
        (editedModelSelection + PRECODED_MODEL_COUNT +
         navigationDirection) %
        PRECODED_MODEL_COUNT);
    menuNavigationDirection = navigationDirection;
    lastMenuNavigationTime = millis();
  }

  const int rightHorizontal = rightXValue;
  if (abs(rightHorizontal) < MENU_STICK_RELEASE_THRESHOLD &&
      (screenMode != ScreenMode::SETTINGS ||
       static_cast<int32_t>(millis() - rightStickGraceUntil) >= 0)) {
    rightStickActionReady = true;
  }

  if (screenMode == ScreenMode::SETTINGS) {
    if (rightStickActionReady && rightHorizontal > MENU_STICK_THRESHOLD) {
      rightStickActionReady = false;
      if (settingsSelection == 0) {
        screenMode = ScreenMode::MODEL_SELECTION;
        editedModelSelection = activeModelSelection;
        rightStickActionReady = false;
      } else if (settingsSelection == 1) {
        screenMode = ScreenMode::CONTROL_MODE;
        editedControlMode = controlMode;
        rightStickActionReady = false;
      } else if (settingsSelection == 2) {
        runCalibration();
      } else if (settingsSelection == 3) {
        screenMode = ScreenMode::BATTERY_MULTIPLIER;
        editedBatteryVoltageMultiplier = batteryVoltageMultiplier;
        lastValueAdjustTime = millis();
        valueAdjustDirection = 0;
        rightStickActionReady = false;
      } else if (settingsSelection == 4) {
        screenMode = ScreenMode::ERASE_CONFIRM;
        rightStickActionReady = false;
      } else {
        screenMode = ScreenMode::HOME;
        controlGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
      }
    }
  } else if (screenMode == ScreenMode::BATTERY_MULTIPLIER) {
    if (rightStickActionReady &&
        rightHorizontal > MENU_STICK_THRESHOLD) {
      if (saveBatteryMultiplier(editedBatteryVoltageMultiplier)) {
        batteryVoltageMultiplier = editedBatteryVoltageMultiplier;
        updateBatteryVoltage();
        screenMode = ScreenMode::SETTINGS;
        rightStickGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
        rightStickActionReady = false;
      } else {
        Serial.println("Failed to save battery multiplier; value not applied.");
      }
      rightStickActionReady = false;
    } else if (rightStickActionReady &&
               rightHorizontal < -MENU_STICK_THRESHOLD) {
      editedBatteryVoltageMultiplier = batteryVoltageMultiplier;
      screenMode = ScreenMode::SETTINGS;
      rightStickGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
      rightStickActionReady = false;
    }

    const int leftHorizontal = leftXValue;
    if (abs(leftHorizontal) < MENU_STICK_RELEASE_THRESHOLD) {
      valueAdjustDirection = 0;
    } else if (abs(leftHorizontal) > MENU_STICK_THRESHOLD &&
               ((leftHorizontal > 0 && valueAdjustDirection < 1) ||
                (leftHorizontal < 0 && valueAdjustDirection > -1))) {
      valueAdjustDirection = leftHorizontal > 0 ? 1 : -1;
    }
    const int stickMagnitude =
        constrain(abs(leftHorizontal), MENU_STICK_THRESHOLD, 1000);
    const float pushStrength =
        (stickMagnitude - MENU_STICK_THRESHOLD) /
        static_cast<float>(1000 - MENU_STICK_THRESHOLD);
    const uint32_t repeatIntervalMs = VALUE_ADJUST_SLOW_REPEAT_MS -
        static_cast<uint32_t>(
            (VALUE_ADJUST_SLOW_REPEAT_MS - VALUE_ADJUST_FAST_REPEAT_MS) *
            pushStrength * pushStrength);
    if (valueAdjustDirection != 0 &&
        millis() - lastValueAdjustTime >= repeatIntervalMs) {
      editedBatteryVoltageMultiplier = constrain(
          editedBatteryVoltageMultiplier +
              valueAdjustDirection * BATTERY_MULTIPLIER_STEP,
          BATTERY_MULTIPLIER_MIN,
          BATTERY_MULTIPLIER_MAX);
      lastValueAdjustTime = millis();
    }
  } else if (screenMode == ScreenMode::CONTROL_MODE) {
    if (rightStickActionReady &&
        rightHorizontal > MENU_STICK_THRESHOLD) {
      if (saveControlMode(editedControlMode)) {
        controlMode = editedControlMode;
        screenMode = ScreenMode::SETTINGS;
        rightStickGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
      } else {
        Serial.println("Failed to save control mode; mode not applied.");
      }
      rightStickActionReady = false;
    } else if (rightStickActionReady &&
               rightHorizontal < -MENU_STICK_THRESHOLD) {
      editedControlMode = controlMode;
      screenMode = ScreenMode::SETTINGS;
      rightStickGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
      rightStickActionReady = false;
    }

  } else if (screenMode == ScreenMode::MODEL_SELECTION) {
    if (rightStickActionReady &&
        rightHorizontal > MENU_STICK_THRESHOLD) {
      if (selectModel(editedModelSelection)) {
        screenMode = ScreenMode::SETTINGS;
        rightStickGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
      }
      rightStickActionReady = false;
    } else if (rightStickActionReady &&
               rightHorizontal < -MENU_STICK_THRESHOLD) {
      editedModelSelection = activeModelSelection;
      screenMode = ScreenMode::SETTINGS;
      rightStickGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
      rightStickActionReady = false;
    }

  } else if (screenMode == ScreenMode::ERASE_CONFIRM) {
    if (rightStickActionReady &&
        rightHorizontal > MENU_STICK_THRESHOLD) {
      if (eraseSavedSettings()) {
        Serial.println("Saved settings erased; defaults restored.");
        screenMode = ScreenMode::SETTINGS;
        rightStickGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
      } else {
        Serial.println("Failed to erase saved settings.");
      }
      rightStickActionReady = false;
    } else if (rightStickActionReady &&
               rightHorizontal < -MENU_STICK_THRESHOLD) {
      screenMode = ScreenMode::SETTINGS;
      rightStickGraceUntil = millis() + RIGHT_STICK_GRACE_MS;
      rightStickActionReady = false;
    }
  }
}

void setControlOutputs(int throttle, int turn) {
  int leftOutput = throttle + turn;
  int rightOutput = throttle - turn;
  const int largestMagnitude = max(abs(leftOutput), abs(rightOutput));
  if (largestMagnitude > 1000) {
    leftOutput = leftOutput * 1000 / largestMagnitude;
    rightOutput = rightOutput * 1000 / largestMagnitude;
  }
  myData.leftY = static_cast<int16_t>(leftOutput);
  myData.rightY = static_cast<int16_t>(rightOutput);
}

void runCalibration() {
  calibrationInProgress = true;
  const CalibrationRecord previousCalibration = currentCalibration();
  AxisCalibration *axes[] = {
      &leftYCalibration, &leftXCalibration, &rightYCalibration, &rightXCalibration};
  const char *axisNames[] = {"Left Y", "Left X", "Right Y", "Right X"};

  Serial.println("\n--- CALIBRATION STARTED ---");
  Serial.println("Release both sticks; measuring centers for 2 seconds.");
  showCalibrationStatus("Release sticks");
  delay(2000);

  long sums[4] = {0, 0, 0, 0};
  for (int sample = 0; sample < 10; ++sample) {
    for (uint8_t axis = 0; axis < 4; ++axis) {
      sums[axis] += analogRead(axes[axis]->pin);
    }
    delay(10);
  }
  for (uint8_t axis = 0; axis < 4; ++axis) {
    axes[axis]->center = sums[axis] / 10;
    axes[axis]->min = axes[axis]->center;
    axes[axis]->max = axes[axis]->center;
    Serial.printf("%s center: %d\n", axisNames[axis], axes[axis]->center);
  }

  Serial.println("Move both sticks through all extremes for 5 seconds.");
  showCalibrationStatus("Move sticks fully");
  const uint32_t calibrationStartedAt = millis();
  while (millis() - calibrationStartedAt < 5000) {
    for (uint8_t axis = 0; axis < 4; ++axis) {
      const int rawValue = analogRead(axes[axis]->pin);
      axes[axis]->min = min(axes[axis]->min, rawValue);
      axes[axis]->max = max(axes[axis]->max, rawValue);
    }
    delay(5);
  }

  const CalibrationRecord calibrated = currentCalibration();
  if (!isCalibrationValid(calibrated)) {
    applyCalibration(previousCalibration);
    Serial.println("Calibration was invalid; previous calibration retained.");
  } else if (saveCalibration()) {
    Serial.println("Calibration saved to non-volatile storage.");
  } else {
    Serial.println("Failed to save calibration; it will be lost after reboot.");
  }
  for (uint8_t axis = 0; axis < 4; ++axis) {
    Serial.printf("%s min/center/max: %d / %d / %d\n",
                  axisNames[axis], axes[axis]->min, axes[axis]->center, axes[axis]->max);
  }
  Serial.println("Normal operation resumed.\n");
  calibrationInProgress = false;
  updateDisplay();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  analogReadResolution(12);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(OLED_ADDRESS);
  if (Wire.endTransmission() == 0 &&
      display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    displayReady = true;
    updateDisplay();
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
  loadBatteryMultiplier();
  loadControlMode();
  loadModelSelection();
  updateBatteryVoltage();

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

  if (!addEspNowPeer(getModelAddress(activeModelSelection))) {
    Serial.println("Failed to add ESP-NOW peer");
    return;
  }

  radioReady = true;

  Serial.println("\n=========================================");
  Serial.println("        TX BATTLEBOT CONTROLLER");
  Serial.println("=========================================");
  Serial.println("Hold both buttons for 1 second to open settings.");
  Serial.printf("Battery voltage multiplier: %.2f\n", batteryVoltageMultiplier);
  Serial.println("-----------------------------------------");
  Serial.println("To calibrate, type 'calibrate' and press Enter.");
  Serial.println("=========================================\n");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "calibrate" || cmd == "calibration") {
      runCalibration();
    }
  }

  serviceButtons();

  if (millis() - lastBatteryReadTime >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryReadTime = millis();
    updateBatteryVoltage();
  }

  if (millis() - lastDisplayUpdateTime >= OLED_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateTime = millis();
    updateDisplay();
  }

  if (!radioReady || calibrationInProgress) {
    return;
  }

  if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL_MS) {
    lastTransmitTime = millis();

    leftYValue = mapJoystick(
        analogRead(leftYCalibration.pin),
        leftYCalibration.min, leftYCalibration.center, leftYCalibration.max);
    leftXValue = mapJoystick(
        analogRead(leftXCalibration.pin),
        leftXCalibration.min, leftXCalibration.center, leftXCalibration.max);
    rightYValue = mapJoystick(
        analogRead(rightYCalibration.pin),
        rightYCalibration.min, rightYCalibration.center, rightYCalibration.max);
    rightXValue = mapJoystick(
        analogRead(rightXCalibration.pin),
        rightXCalibration.min, rightXCalibration.center, rightXCalibration.max);

    serviceJoystickMenu();

    myData.magic = PACKET_MAGIC;
    myData.sequence = ++packetSequence;
    if (screenMode != ScreenMode::HOME ||
        static_cast<int32_t>(millis() - controlGraceUntil) < 0) {
      myData.leftY = 0;
      myData.rightY = 0;
    } else if (controlMode == ControlMode::TANK) {
      myData.leftY = static_cast<int16_t>(leftYValue);
      myData.rightY = static_cast<int16_t>(rightYValue);
    } else {
      setControlOutputs(leftYValue, rightXValue);
    }

    if (esp_now_send(getModelAddress(activeModelSelection),
                     reinterpret_cast<const uint8_t *>(&myData),
                     sizeof(myData)) != ESP_OK) {
      lastSendSucceeded = false;
      lastSendStatusTime = millis();
      sendStatusKnown = true;
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
