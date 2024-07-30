#include <Wire.h>
#include <Adafruit_PN532.h>
#include <BluetoothSerial.h>

// Pin untuk SPI
#define PN532_SCK (18)
#define PN532_MISO (19)
#define PN532_MOSI (23)
#define PN532_SS (5)

Adafruit_PN532 nfc(PN532_SS);
BluetoothSerial SerialBT;

bool bluetoothConnected = false;
unsigned long lastCheck = 0;
const unsigned long checkInterval = 1000;   // Interval untuk memeriksa koneksi Bluetooth
const unsigned long nfcReadTimeout = 1000;  // Timeout untuk pembacaan NFC

#define ledBT 4
#define ledBat 25
// #define gndLedBat 2
#define buzzer 27
#define BATTERY_PIN 32

const int ledValue = 25;
// Baterai Setup
// #define bateraiIndikator 32
// const int BATTERY_PIN = 32; // Pin ADC yang digunakan untuk membaca tegangan
const float maxVoltage = 4.1;        // Tegangan maksimal
const float minVoltage = 3.4;        // Tegangan minimal
const float referenceVoltage = 4.1;  // Tegangan referensi
const int adcMaxValue = 4095;        // Nilai maksimum ADC (12-bit ADC)
const float ralatBaterai = 0.04;
const int numSamples = 500;  // Jumlah sampel untuk averaging

String nfcID = "";

void bateraiIndikator() {
  long adcSum = 0;
  for (int i = 0; i < numSamples; i++) {
    adcSum += analogRead(BATTERY_PIN);
    delay(1);  // Delay pendek untuk memberikan waktu antara pembacaan sampel
  }

  int adcValue = adcSum / numSamples;
  float voltage = (adcValue * referenceVoltage) / adcMaxValue;

  if (voltage > maxVoltage) voltage = voltage;
  if (voltage < minVoltage) voltage = voltage;

  int percentage = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100;
  if(percentage <= 20)
  {
    analogWrite(ledBat,ledValue);
    // digitalWrite(gndLedBat,LOW);
  }
  else if(percentage >= 80)
  {
    analogWrite(ledBat, 0);
    Serial.println("masuk");
    // digitalWrite(gndLedBat,LOW);
  }

  // Print ke Serial Monitor
  Serial.print("ADC : ");
  Serial.print(adcValue);
  Serial.print(" ");
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, ");
  Serial.print("Battery Percentage: ");
  Serial.print(percentage);
  Serial.println("masuk ini ");
  Serial.println("masuk ini 2");
  Serial.println("masuk 5");
  SerialBT.println(voltage);
  // SerialBT.println(percentage);
  Serial.println(" %");
}

void setup(void) {
  Serial.begin(115200);
  SerialBT.begin("borplecit");  // Nama Bluetooth
  Serial.println("NFC reader initializing...");
  pinMode(ledBT, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(ledBat, OUTPUT);
  // pinMode(gndLedBat,OUTPUT);
  analogReadResolution(12);  // ADC diubah jadi 12-bit
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1)
      ;  // halt
  }
  nfc.SAMConfig();
  Serial.println("System Ready Waiting for an NFC card ...");
  // digitalWrite(gndLedBat,LOW);
  for (int x = 1; x <= 3; x++) {
    analogWrite(ledBT, ledValue);
    analogWrite(ledBat, ledValue);
    digitalWrite(buzzer, LOW);
    delay(100);
    digitalWrite(buzzer, HIGH);
    analogWrite(ledBT, 0);
    analogWrite(ledBat, 0);
    delay(100);
  }
}

void loop(void) {
  // while(1)
  // {
    
  //   analogWrite(ledBat,ledValue);
  //   // digitalWrite(gndLedBat,LOW);
  //   delay(300);
  //   analogWrite(ledBat,0);
  //   // digitalWrite(gndLedBat,LOW);
  //   delay(300);
  // }
  unsigned long currentMillis = millis();

  // Memeriksa koneksi Bluetooth setiap interval tertentu
  if (currentMillis - lastCheck >= checkInterval) {
    lastCheck = currentMillis;
    bateraiIndikator();
    bluetoothConnected = SerialBT.hasClient();
  }

  if (bluetoothConnected) {
    uint8_t uid[7];  // Maximum size of the UID
    uint8_t uidLength;
    // bateraiIndikator();

    // Memulai pendeteksian non-blocking
    nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);

    unsigned long startTime = millis();
    bool cardDetected = false;

    // Coba membaca kartu NFC dengan timeout
    while (millis() - startTime < nfcReadTimeout) {
      // Coba baca apakah ada kartu yang terdeteksi
      if (nfc.readDetectedPassiveTargetID(uid, &uidLength)) {
        cardDetected = true;
        break;
      }

      // Periksa kembali koneksi Bluetooth selama menunggu
      if (!SerialBT.hasClient()) {
        bluetoothConnected = false;
        break;
      }

      delay(10);  // Tambahkan sedikit delay untuk mengurangi beban CPU
    }

    if (cardDetected) {
      Serial.print("Found an NFC card with ID: ");
      String CardID = "";
      String Link = "";
      unsigned int hex_num;
      hex_num = uid[3] << 24;
      hex_num += uid[2] << 16;
      hex_num += uid[1] << 8;
      hex_num += uid[0];
      CardID = String(hex_num);
      if (CardID.length() == 8) {
        CardID = '0' + CardID;
      }
      if (CardID.length() == 9)  //panjang karakter untuk nfc id
      {
        CardID = '0' + CardID;
      } else {
        nfcID = CardID;
      }
      nfcID = CardID;
      delay(50);

        digitalWrite(buzzer, LOW);
        delay(300);
        digitalWrite(buzzer, HIGH);
        delay(300);
      
      digitalWrite(ledBT, LOW);
      delay(100);
      Serial.println(nfcID);
      SerialBT.print(nfcID + String("\n"));
      analogWrite(ledBT, ledValue);
    }
  } else {
    Serial.println("Bluetooth disconnected, waiting for connection...");
    for (int x = 1; x <= 4; x++) {
      digitalWrite(buzzer, LOW);
      delay(200);
      digitalWrite(buzzer, HIGH);
      delay(200);
    }

    while (!SerialBT.hasClient()) {
      delay(500);  // Menunggu hingga ada koneksi Bluetooth
      analogWrite(ledBT, ledValue);
      delay(100);
      analogWrite(ledBT, 0);
      delay(100);
      bateraiIndikator();
      delay(1000);
    }
    Serial.println("Bluetooth connected, resuming NFC reading...");

    analogWrite(ledBT, ledValue);
    for (int x = 1; x <= 3; x++) {
      digitalWrite(buzzer, LOW);
      delay(200);
      digitalWrite(buzzer, HIGH);
      delay(200);
    }
  }
  delay(500);  // Delay untuk menghindari loop terlalu cepat
}
