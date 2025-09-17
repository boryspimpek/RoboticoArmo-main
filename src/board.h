#include <Wire.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"   // <- potrzebne do esp_bt_dev_get_address()
#include <Adafruit_SSD1306.h>
#include <SMS_STS.h>

BluetoothSerial SerialBT;
TaskHandle_t ScreenUpdateHandle;
TaskHandle_t ClientCmdHandle;

// Konfiguracja UART dla serw
#define S_RXD 18
#define S_TXD 19

// Konfiguracja wyświetlacza
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Zmienne globalne
const int MAX_SERVOS = 10;
bool servosFound[MAX_SERVOS] = {false};
int foundCount = 0;
bool scanComplete = false;
String btAddress = "";


extern SMS_STS st;

void InitScreen() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 5);
  display.println("ROBOTICS");

  display.setTextSize(1);
  display.setCursor(25, 25);
  display.println("CONTROL SYSTEM");
  display.drawLine(0, 35, 128, 35, SSD1306_WHITE);

  display.display(); 
  delay(1000); 
}

void scanServos() {
  foundCount = 0; // reset licznika
  for (int id = 1; id <= MAX_SERVOS; id++) {
    int result = st.Ping(id);
    if (result != -1) {
      servosFound[id-1] = true;
      foundCount++;
    } else {
      servosFound[id-1] = false;
    }
    delay(50);
  }
}

void btMac() {
  if (!SerialBT.begin("ESP32_BT")) {   // Nazwa urządzenia BT
    Serial.println("Błąd inicjalizacji Bluetooth!");
    while (true);
  }

  // Pobranie adresu MAC Bluetooth
  const uint8_t* mac = esp_bt_dev_get_address();
  if (mac) {
    btAddress = "";
    for (int i = 0; i < 6; i++) {
      if (i != 0) btAddress += ":";
      if (mac[i] < 16) btAddress += "0";
      btAddress += String(mac[i], HEX);
    }
    btAddress.toUpperCase();

    Serial.print("Adres MAC Bluetooth ESP32: ");
    Serial.println(btAddress);
  } else {
    Serial.println("Nie udało się odczytać adresu MAC.");
    btAddress = "N/A";
    delay(50);
  }
}

void displayResultsScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("Scan results:");
  display.println("-------------------");
  
  if (foundCount == 0) {
    display.setCursor(0, 16);
    display.println("No servos detected!");
  } else {
    display.setCursor(0, 16);
    display.print("Servos: ");
    for (int i = 0; i < MAX_SERVOS; i++) {
      if (servosFound[i]) {
        display.print(i+1);  // żeby ID zgadzało się z rzeczywistością
        display.print(" ");
      }
    }
  }

  // Adres Bluetooth na dole ekranu
  display.setCursor(0, 25);
  display.print("BT: ");
  display.println(btAddress);

  display.display();
}