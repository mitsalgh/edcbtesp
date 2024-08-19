#include <Wire.h>
#include <Adafruit_PN532.h>
#include <BluetoothSerial.h>

// Pins for SPI
#define PN532_SCK (18)
#define PN532_MISO (19)
#define PN532_MOSI (23)
#define PN532_SS (5)

Adafruit_PN532 nfc(PN532_SS);
BluetoothSerial SerialBT;

bool bluetoothConnected = false;
unsigned long lastCheck = 0;
const unsigned long checkInterval = 1000;   // Interval to check Bluetooth connection
const unsigned long nfcReadTimeout = 1000;  // Timeout to read NFC ID

#define ledBT 4
#define ledBat 25
// #define gndLedBat 2
#define buzzer 27
#define BATTERY_PIN 32

const int ledValue = 25;
// Baterai Setup
// #define bateraiIndikator 32
// const int BATTERY_PIN = 32; // "ADC pin used to read voltage"
const float maxVoltage = 4.1;        // Maximum Voltage
const float minVoltage = 3.4;        // Minimum Voltage
const float referenceVoltage = 4.1;  // Reference Voltage
const int adcMaxValue = 4095;        // Maximum Value ADC (12-bit ADC)
const float ralatBaterai = 0.04;
const int numSamples = 500;  // sum sample for averaging

String nfcID = "";

void bateraiIndikator() {
  long adcSum = 0;
  for (int i = 0; i < numSamples; i++) {
    adcSum += analogRead(BATTERY_PIN);
    delay(1);  // Short delay to allow time between sample readings
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

  // prints to Serial Monitor
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
  SerialBT.begin("apa aja deh");// name of bluetooth
  Serial.println("NFC reader initializing...");
  pinMode(ledBT, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(ledBat, OUTPUT);
  // pinMode(gndLedBat,OUTPUT);
  analogReadResolution(12);  // ADC changes to 12-bit
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

  // Checking Bluetooth connection at regular intervals
  if (currentMillis - lastCheck >= checkInterval) {
    lastCheck = currentMillis;
    bateraiIndikator();
    bluetoothConnected = SerialBT.hasClient();
  }

  if (bluetoothConnected) {
    uint8_t uid[7];  // Maximum size of the UID
    uint8_t uidLength;
    // bateraiIndikator();

    // start for detection non-blocking
    nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);

    unsigned long startTime = millis();
    bool cardDetected = false;

    // Try to read NFC with timeout
    while (millis() - startTime < nfcReadTimeout) {
      // "Try to read if a card is detected"
      if (nfc.readDetectedPassiveTargetID(uid, &uidLength)) {
        cardDetected = true;
        break;
      }

      // Recheck the Bluetooth connection while waiting
      if (!SerialBT.hasClient()) {
        bluetoothConnected = false;
        break;                                                                                                                                                                                                                                     //Cr by Mitsal Ghapiqi
      }

      delay(10);  // Add a short delay to reduce CPU load
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
      if (CardID.length() == 9)  
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
      delay(500);  // Waiting for a Bluetooth connection
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
  delay(500);  // "Delay to avoid looping too quickly"
}
// cr by Mitsal Ghapiqi
