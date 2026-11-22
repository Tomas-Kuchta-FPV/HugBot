#include <esp_now.h>
#include <WiFi.h>

constexpr uint8_t LED_PIN = 8;

// DRV8833 motor pins
constexpr uint8_t IN1 = 0;  // Left forward
constexpr uint8_t IN2 = 1;  // Left reverse
constexpr uint8_t IN3 = 20; // Right forward
constexpr uint8_t IN4 = 10; // Right reverse

constexpr uint32_t PACKET_MAGIC = 0x48554742;
constexpr uint32_t FAILSAFE_TIMEOUT_MS = 500;
constexpr uint32_t RX_PRINT_INTERVAL_MS = 250;
constexpr int MOTOR_DEADZONE = 30;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t sequence;
  int16_t leftY;
  int16_t rightY;
} ControlPacket;

static_assert(sizeof(ControlPacket) == 12, "ControlPacket must remain 12 bytes");

ControlPacket myData{};
volatile bool newDataReady = false;
uint32_t lastSequence = 0;
uint32_t lastRecvTime = 0;
uint32_t lastPrintTime = 0;
bool ledState = false;
bool radioReady = false;

void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  (void)recvInfo;

  if (len != sizeof(ControlPacket)) {
    return;
  }

  ControlPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  if (packet.magic != PACKET_MAGIC ||
      packet.leftY < -1000 || packet.leftY > 1000 ||
      packet.rightY < -1000 || packet.rightY > 1000 ||
      packet.sequence == lastSequence) {
    return;
  }

  noInterrupts();
  myData = packet;
  lastSequence = packet.sequence;
  newDataReady = true;
  interrupts();
}

void driveMotor(int pinFwd, int pinRev, int speed) {
  speed = constrain(speed, -255, 255);

  if (abs(speed) < MOTOR_DEADZONE) {
    analogWrite(pinFwd, 0);
    analogWrite(pinRev, 0);
    return;
  }

  if (speed > 0) {
    analogWrite(pinFwd, speed);
    analogWrite(pinRev, 0);
  } else {
    analogWrite(pinFwd, 0);
    analogWrite(pinRev, abs(speed));
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWriteResolution(8);

  driveMotor(IN1, IN2, 0);
  driveMotor(IN3, IN4, 0);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  delay(2000);

  Serial.println("\n=========================================");
  Serial.println("        RX BATTLEBOT RECEIVER");
  Serial.println("=========================================");
  Serial.println("Copy this MAC address into the transmitter:");

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("{");
  for (int i = 0; i < 6; ++i) {
    Serial.printf("0x%02X", mac[i]);
    if (i < 5) {
      Serial.print(", ");
    }
  }
  Serial.println("}");

  Serial.print("Standard MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("-----------------------------------------");
  Serial.println("Pin configuration (DRV8833):");
  Serial.printf("Left motor:  IN1 (forward) = %d, IN2 (reverse) = %d\n", (int)IN1, (int)IN2);
  Serial.printf("Right motor: IN3 (forward) = %d, IN4 (reverse) = %d\n", (int)IN3, (int)IN4);
  Serial.println("Failsafe timeout: 500 ms");
  Serial.println("=========================================\n");

  delay(2000);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK) {
    Serial.println("Failed to register receive callback");
    return;
  }

  radioReady = true;
  Serial.println("Radio active. Waiting for controller signal...");
  lastRecvTime = millis();
}

void loop() {
  if (!radioReady) {
    driveMotor(IN1, IN2, 0);
    driveMotor(IN3, IN4, 0);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    return;
  }

  if (newDataReady) {
    ControlPacket packet;
    noInterrupts();
    packet = myData;
    newDataReady = false;
    interrupts();

    lastRecvTime = millis();
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);

    int leftSpeed = map(packet.leftY, -1000, 1000, -255, 255);
    int rightSpeed = map(packet.rightY, -1000, 1000, -255, 255);

    driveMotor(IN1, IN2, leftSpeed);
    driveMotor(IN3, IN4, rightSpeed);

    if (millis() - lastPrintTime >= RX_PRINT_INTERVAL_MS) {
      lastPrintTime = millis();
      Serial.print("Received -> Left: ");
      Serial.print(packet.leftY / 10.0f, 1);
      Serial.print("% | Right: ");
      Serial.print(packet.rightY / 10.0f, 1);
      Serial.print("% | PWM: ");
      Serial.print(leftSpeed);
      Serial.print(" / ");
      Serial.println(rightSpeed);
    }
  }

  if (millis() - lastRecvTime > FAILSAFE_TIMEOUT_MS) {
    driveMotor(IN1, IN2, 0);
    driveMotor(IN3, IN4, 0);
    digitalWrite(LED_PIN, LOW);
  }
}
