#include <esp_now.h>
#include <WiFi.h>

const int LED_PIN = 8; 

// Piny motorů (DRV8833)
const int IN1 = 0;  // Levy Vpred
const int IN2 = 1;  // Levy Vzad
const int IN3 = 20; // Pravy Vpred
const int IN4 = 10; // Pravy Vzad

const int transmitRateMs = 20; // 50Hz

typedef struct struct_message {
  int leftY;
  int rightY;
} struct_message;

struct_message myData;
volatile bool newDataReady = false;

unsigned long lastRecvTime = 0;
bool ledState = false;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(myData)) {
    memcpy((void*)&myData, incomingData, sizeof(myData));
    newDataReady = true; 
  }
}

void driveMotor(int pinFwd, int pinRev, int speed) {
  if (abs(speed) < 30) {
    analogWrite(pinFwd, 0);
    analogWrite(pinRev, 0);
    return;
  }
  
  speed = constrain(speed, -255, 255);

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

  // Bezpečnostní zamknutí motorů při startu
  driveMotor(IN1, IN2, 0);
  driveMotor(IN3, IN4, 0);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  // --- PRVNÍ COOLDOWN (Před vypsáním infa) ---
  delay(2000);
  
  // --- BOOT INFO & MAC ADDRESS ---
  Serial.println("\n=========================================");
  Serial.println("        RX BATTLEBOT RECEIVER");
  Serial.println("=========================================");
  Serial.println(">>> KOPIROVATELNA MAC ADRESA PRO TX:");
  
  String macStr = WiFi.macAddress(); // Získá MAC jako řetězec např. "B8:1F:3F:2C:BA:08"
  Serial.print("{");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    String byteStr = macStr.substring(i * 3, i * 3 + 2);
    Serial.print(byteStr);
    if (i < 5) Serial.print(", ");
  }
  Serial.println("}");
  
  Serial.print("Standardni MAC: "); Serial.println(macStr);
  Serial.println("-----------------------------------------");
  Serial.println("Konfigurace pinu (DRV8833):");
  Serial.printf("Levy motor:  IN1 (Vpred) = %d, IN2 (Vzad) = %d\n", IN1, IN2);
  Serial.printf("Pravy motor: IN3 (Vpred) = %d, IN4 (Vzad) = %d\n", IN3, IN4);
  Serial.println("Failsafe timeout: 500ms");
  Serial.println("=========================================\n");

  // --- DRUHÝ COOLDOWN (Před spuštěním rádia) ---
  delay(2000);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Radio aktivni. Cekam na signal z ovladace...");
  
  lastRecvTime = millis(); // Reset failsafe časovače před vstupem do loopu
}

void loop() {
  if (newDataReady) {
    newDataReady = false;
    
    // VALIDACE: Kontrola celočíselných hranic (-1000 až 1000)
    if (myData.leftY < -1000 || myData.leftY > 1000 || myData.rightY < -1000 || myData.rightY > 1000) {
      Serial.print("IGNOROVANO - Neplatna data! L: "); 
      Serial.print((float)myData.leftY / 10.0, 1); 
      Serial.print(" R: "); 
      Serial.println((float)myData.rightY / 10.0, 1);
    } else {
      lastRecvTime = millis(); 
      
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      
      int leftSpeed = map(myData.leftY, -1000, 1000, -255, 255);
      int rightSpeed = map(myData.rightY, -1000, 1000, -255, 255);
      
      Serial.print("Data (%) -> Levy: "); Serial.print((float)myData.leftY / 10.0, 1);
      Serial.print(" Pravy: "); Serial.print((float)myData.rightY / 10.0, 1);
      Serial.print("  ||  PWM -> Levy: "); Serial.print(leftSpeed);
      Serial.print(" Pravy: "); Serial.println(rightSpeed);
      
      driveMotor(IN1, IN2, leftSpeed);
      driveMotor(IN3, IN4, rightSpeed);
    }
  }

  // --- FAILSAFE ---
  if (millis() - lastRecvTime > transmitRateMs * 4) {
    driveMotor(IN1, IN2, 0);
    driveMotor(IN3, IN4, 0);
    digitalWrite(LED_PIN, HIGH); // Vypnout LED
  }
}