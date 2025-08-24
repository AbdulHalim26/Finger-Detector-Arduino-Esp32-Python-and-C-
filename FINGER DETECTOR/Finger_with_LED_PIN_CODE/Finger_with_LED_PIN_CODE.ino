// Upload Kode ini ke Esp32 atau Mikrokontroller kamu
// ESP32 Hand Detection Control
// Pin definitions untuk LED
const int ledPins[] = {18, 19, 21, 22, 23};
const int numLeds = 5;

void setup() {
  // Inisialisasi serial communication
  Serial.begin(115200);
  
  // Setup LED pins sebagai output
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  
  // Test LED saat startup
  testLeds();
  
  Serial.println("ESP32 pendeteksi tangan siap!");
  Serial.println("menunggu untuk data...");
}

void loop() {
  // Cek apakah ada data dari serial
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim();
    
    // Convert string ke integer
    int fingerCount = receivedData.toInt();
    
    // Validasi input (0-5)
    if (fingerCount >= 0 && fingerCount <= 5) {
      controlLeds(fingerCount);
      Serial.print("Fingers detected: ");
      Serial.println(fingerCount);
    } else {
      Serial.println("Invalid finger count received");
    }
  }
  
  delay(50); // Small delay untuk stabilitas
}

void controlLeds(int fingerCount) {
  // Matikan semua LED terlebih dahulu
  for (int i = 0; i < numLeds; i++) {
    digitalWrite(ledPins[i], LOW);
  }
  
  // Nyalakan LED sesuai jumlah jari yang terdeteksi
  for (int i = 0; i < fingerCount && i < numLeds; i++) {
    digitalWrite(ledPins[i], HIGH);
  }
}

void testLeds() {
  Serial.println("Testing LEDs...");
  
  // Nyalakan satu per satu
  for (int i = 0; i < numLeds; i++) {
    digitalWrite(ledPins[i], HIGH);
    delay(200);
  }
  
  delay(500);
  
  // Matikan semua
  for (int i = 0; i < numLeds; i++) {
    digitalWrite(ledPins[i], LOW);
  }
  
  Serial.println("LED test completed");
}
