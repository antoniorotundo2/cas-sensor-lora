#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LoRa.h>
#include "packet.h"

#define PINLED 25
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */
#define LORA_PIN_SS 18
#define LORA_PIN_RST 23
#define LORA_PIN_DIO0 26
#define LORA_BAND 866E6

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26

RTC_DATA_ATTR int bootCount = 0; /* locazione di memoria che ci servirà per capire se è stata già avviato una prima volta */

Adafruit_SSD1306 display;
Adafruit_BME680 bme;
LoRaPacket packet;

void setup() {
  packet.type=TYPEAPPLICATION;
  // put your setup code here, to run once:
  // inizializzazione seriale UART
  Serial.begin(115200);
  pinMode(PINLED, OUTPUT);
  digitalWrite(PINLED, LOW);
  uint16_t idtemp = (ESP.getEfuseMac()&0x00000000FFFF); // ottengo MAC address e selezione ultime quaatro cifre
  packet.id = idtemp;
  // inizializzazione LoRa
  SPI.begin(SCK, MISO, MOSI, LORA_PIN_SS);
  LoRa.setPins(LORA_PIN_SS, LORA_PIN_RST, LORA_PIN_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("LoRa initialization failed. Check the configuration.");
    while (true);                       
  }
  //LoRa.onTxDone(onTxDone);
  //LoRa.setSyncWord(0xFE); // word di sincronizzazione
  Serial.println("LoRa initialized");
  //inizializzo I2C
  Wire.begin(21, 22);
  if(bootCount<=0) { /* inizializzo solamente quando l'ho avviato una prima volta */
    display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.clearDisplay();
    display.setCursor(0,0);
    Serial.println("Device ID");
    bootCount++;
    display.print(idtemp, HEX);
    display.display();
    delay(5000);
    display.clearDisplay();
    display.display();
  }
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  Serial.println("Sensor initialized");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  digitalWrite(PINLED, HIGH);
  // stampo lo stato di accensione/spegnimento led
  // Serial.printf("Il valore dell'LED è %d\n", digitalRead(PINLED));
  // display.clearDisplay();
  // display.setCursor(0,0);
  // display.printf("Il LED e' %d\n", digitalRead(PINLED));
  // display.display();
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

//  display.display();
packet.temperature=bme.temperature;
packet.pressure=bme.pressure/100.0;
packet.humidity=bme.humidity;
packet.gas=bme.gas_resistance/1000.0;

  Serial.print("Temperature = "); Serial.print(packet.temperature); Serial.println(" *C");

  Serial.print("Pressure = "); Serial.print(packet.pressure); Serial.println(" hPa");

  Serial.print("Humidity = "); Serial.print(packet.humidity); Serial.println(" %");

  Serial.print("Gas = "); Serial.print(packet.gas); Serial.println(" KOhms");

  Serial.println();

// definzione del protocollo
// TEMP_PRESS_HUM_GAS
String message = String(packet.temperature) + "_" + String(packet.pressure) + "_" + 
          String(packet.humidity) + "_" + String(packet.gas);
//sendLoRaMessage(message);
LoRa.beginPacket();
//LoRa.print(message);
LoRa.write((uint8_t *)&packet,sizeof(LoRaPacket));
LoRa.endPacket();
digitalWrite(PINLED, LOW);
delay(1000);
esp_deep_sleep_start();
}