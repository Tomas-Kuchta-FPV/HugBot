#include <esp_now.h>
#include <WiFi.h>

// MAC ADRESA PŘIJÍMAČE
uint8_t broadcastAddress[] = {0xB8, 0x1F, 0x3F, 0x2C, 0xBA, 0x08};

const int LED_PIN = 8; // Pin pro TX LED (Active-Low)

typedef struct struct_message {
  int leftY;
  int rightY;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

unsigned long lastTransmitTime = 0;
const int transmitRateMs = 20; // 50Hz

// Výchozí hodnoty (ideální stav ADC na ESP32)
int leftMin = 0, leftCenter = 2048, leftMax = 4095;
int rightMin = 0, rightCenter = 2048, rightMax = 4095;

void OnDataSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(LED_PIN, LOW);  // Spojeno -> LED SVÍTÍ (Active-Low)
  } else {
    digitalWrite(LED_PIN, HIGH); // Rozpojeno -> LED NESVÍTÍ
  }
}

// Speciální funkce pro mapování od skutečného středu
int mapJoystick(int raw, int minVal, int centerVal, int maxVal) {
  int result;
  if (raw < centerVal) {
    // Spodní polovina dráhy (-1000 až 0)
    result = map(raw, minVal, centerVal, -1000, 0);
  } else {
    // Horní polovina dráhy (0 až 1000)
    result = map(raw, centerVal, maxVal, 0, 1000);
  }
  return constrain(result, -1000, 1000); // Pojistka proti překročení
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Krátká pauza, aby se bezpečně otevřel terminál

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1; 
  peerInfo.encrypt = false;
  
  esp_now_add_peer(&peerInfo);
  
  // --- BOOT INFO ---
  Serial.println("\n=========================================");
  Serial.println("        TX BATTLEBOT CONTROLLER");
  Serial.println("=========================================");
  Serial.println("Aktualni kalibrace (Min / Stred / Max):");
  Serial.printf("Levy Y : %d / %d / %d\n", leftMin, leftCenter, leftMax);
  Serial.printf("Pravy Y: %d / %d / %d\n", rightMin, rightCenter, rightMax);
  Serial.println("-----------------------------------------");
  Serial.println("Pro novou kalibraci napiste 'calibration' do terminalu a dejte Enter.");
  Serial.println("=========================================\n");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); 
    
    if (cmd == "calibration") {
      Serial.println("\n--- KALIBRACE SPUSTENA ---");
      Serial.println("KROK 1: Pustte oba joysticky (nedotykat se!).");
      Serial.println("Zameruji klidovy stred za 2 sekundy...");
      delay(2000);
      
      // Měření středu (průměr z 10 měření pro lepší přesnost)
      long sumL = 0, sumR = 0;
      for(int i = 0; i < 10; i++) {
        sumL += analogRead(0);
        sumR += analogRead(1);
        delay(10);
      }
      leftCenter = sumL / 10;
      rightCenter = sumR / 10;
      
      Serial.printf("Stred ulozen -> Levy: %d | Pravy: %d\n", leftCenter, rightCenter);
      
      Serial.println("\nKROK 2: Nyni hybejte obema joysticky do vsech krajnich poloh (nahoru a dolu)... mate 5 sekund.");
      
      // Reset limitů na právě zjištěný střed
      leftMin = leftCenter; leftMax = leftCenter;
      rightMin = rightCenter; rightMax = rightCenter;
      
      unsigned long calibStart = millis();
      while (millis() - calibStart < 5000) { 
        int rawL = analogRead(0);
        int rawR = analogRead(1);
        
        if (rawL < leftMin) leftMin = rawL;
        if (rawL > leftMax) leftMax = rawL;
        if (rawR < rightMin) rightMin = rawR;
        if (rawR > rightMax) rightMax = rawR;
        delay(5);
      }
      
      Serial.println("\n--- KALIBRACE DOKONCENA ---");
      Serial.printf("Levy Y (min/stred/max) : %d / %d / %d\n", leftMin, leftCenter, leftMax);
      Serial.printf("Pravy Y (min/stred/max): %d / %d / %d\n", rightMin, rightCenter, rightMax);
      Serial.println("Nyni bezi normalni provoz.\n");
    }
  }

  if (millis() - lastTransmitTime >= transmitRateMs) {
    lastTransmitTime = millis();
    
    int rawLeft = analogRead(0);  
    int rawRight = analogRead(1); 

    // Výpočet z centrované křivky
    myData.leftY = mapJoystick(rawLeft, leftMin, leftCenter, leftMax);
    myData.rightY = mapJoystick(rawRight, rightMin, rightCenter, rightMax);

    // VÝPIS DO SÉRIOVÉ LINKY
    Serial.print("TX Odesila -> Levy: "); Serial.print((float)myData.leftY / 10.0, 1);
    Serial.print(" | Pravy: "); Serial.println((float)myData.rightY / 10.0, 1);

    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
}