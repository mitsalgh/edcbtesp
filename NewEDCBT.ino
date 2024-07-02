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
#define buzzer 27
String nfcID = "";

void setup(void) {
  Serial.begin(115200);
  SerialBT.begin("EDC BT");  // Nama Bluetooth
  Serial.println("NFC reader initializing...");
  pinMode(ledBT, OUTPUT);
  pinMode(buzzer, OUTPUT);
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1)
      ;  // halt
  }
  nfc.SAMConfig();
  Serial.println("System Ready Waiting for an NFC card ...");
  for (int x = 1; x <= 3; x++) {
    digitalWrite(ledBT, HIGH);
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    digitalWrite(ledBT, LOW);
    delay(100);
  }
}

void loop(void) {
  unsigned long currentMillis = millis();

  // Memeriksa koneksi Bluetooth setiap interval tertentu
  if (currentMillis - lastCheck >= checkInterval) {
    lastCheck = currentMillis;
    bluetoothConnected = SerialBT.hasClient();
  }

  if (bluetoothConnected) {
    uint8_t uid[7];  // Maximum size of the UID
    uint8_t uidLength;

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
      digitalWrite(buzzer, HIGH);
      delay(300);
      digitalWrite(buzzer, LOW);
      delay(300);
      digitalWrite(ledBT, LOW);
      delay(100);
      Serial.println(nfcID);
      SerialBT.print(nfcID + String("\n"));
      digitalWrite(ledBT,HIGH);
    }
  } else {
    Serial.println("Bluetooth disconnected, waiting for connection...");
    for (int x = 1; x <= 4; x++) {
      digitalWrite(buzzer, HIGH);
      delay(200);
      digitalWrite(buzzer, LOW);
      delay(200);
    }
    
    while (!SerialBT.hasClient()) {
      delay(500);  // Menunggu hingga ada koneksi Bluetooth
      digitalWrite(ledBT,HIGH);
      delay(100);
      digitalWrite(ledBT,LOW);
      delay(100);
    }
    Serial.println("Bluetooth connected, resuming NFC reading...");
    digitalWrite(ledBT, HIGH);
    for (int x = 1; x <= 3; x++) {
      digitalWrite(buzzer, HIGH);
      delay(100);
      digitalWrite(buzzer, LOW);
      delay(100);
    }
    
  }
  delay(500);  // Delay untuk menghindari loop terlalu cepat
}
