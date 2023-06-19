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
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 5        /* Time ESP32 will go to sleep (in seconds) */
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

#define MAX_READ_COUNTS 5
#define HUMIDITY_BASELINE 40.0
#define HUMIDITY_WEIGHT 0.25

RTC_DATA_ATTR int bootCount = 0; /* locazione di memoria che ci servirà per capire se è stata già avviato una prima volta */

Adafruit_SSD1306 display;
Adafruit_BME680 bme;
LoRaPacket packet;

float gas_reads[MAX_READ_COUNTS];
int read_count = 0;

void setup()
{
  packet.type = TYPEAPPLICATION;
  // put your setup code here, to run once:
  // inizializzazione seriale UART
  Serial.begin(115200);
  pinMode(PINLED, OUTPUT);
  digitalWrite(PINLED, LOW);
  uint16_t idtemp = (ESP.getEfuseMac() & 0x00000000FFFF); // ottengo MAC address e selezione ultime quaatro cifre
  packet.id = idtemp;
  // inizializzazione LoRa
  SPI.begin(SCK, MISO, MOSI, LORA_PIN_SS);
  LoRa.setPins(LORA_PIN_SS, LORA_PIN_RST, LORA_PIN_DIO0);
  if (!LoRa.begin(LORA_BAND))
  {
    Serial.println("LoRa initialization failed. Check the configuration.");
    while (true)
      ;
  }
  // LoRa.onTxDone(onTxDone);
  // LoRa.setSyncWord(0xFE); // word di sincronizzazione
  Serial.println("LoRa initialized");
  // inizializzo I2C
  Wire.begin(21, 22);
  if (bootCount <= 0)
  { /* inizializzo solamente quando l'ho avviato una prima volta */
    display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.clearDisplay();
    display.setCursor(0, 0);
    Serial.println("Device ID");
    bootCount++;
    display.print(idtemp, HEX);
    display.display();
    delay(5000);
    display.clearDisplay();
    display.display();
  }
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  Serial.println("Sensor initialized");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(1000);
  digitalWrite(PINLED, HIGH);
  // stampo lo stato di accensione/spegnimento led
  // Serial.printf("Il valore dell'LED è %d\n", digitalRead(PINLED));
  // display.clearDisplay();
  // display.setCursor(0,0);
  // display.printf("Il LED e' %d\n", digitalRead(PINLED));
  // display.display();
  if (!bme.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return;
  }

  if (read_count < MAX_READ_COUNTS)
  {
    while (read_count < MAX_READ_COUNTS)
    {
      if (!bme.performReading())
      {
        Serial.println("Failed to perform reading :(");
        return;
      }
      gas_reads[read_count] = bme.gas_resistance;
      read_count++;
      delay(500);
    }
  }
  float all_sums = 0;
  for (int i = 0; i <= MAX_READ_COUNTS - 1; i++)
  {
    all_sums += gas_reads[i];
    if (i > 0)
    {
      gas_reads[i - 1] = gas_reads[i];
    }
  }
  delay(500);
  if (!bme.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return;
  }
  gas_reads[MAX_READ_COUNTS - 1] = bme.gas_resistance;
  all_sums += gas_reads[MAX_READ_COUNTS - 1];
  float mean_reads = all_sums / (MAX_READ_COUNTS * 1.0);
  float gas_offset = mean_reads - gas_reads[MAX_READ_COUNTS - 1];
  float humidity_offset = bme.humidity - HUMIDITY_BASELINE;
  float humidity_score = 0;
  float gas_score = 0;

  if(humidity_offset > 0){
    humidity_score = ((100-HUMIDITY_BASELINE-humidity_offset)/(100-HUMIDITY_BASELINE))*(HUMIDITY_WEIGHT*100);
  } else {
    humidity_score = ((HUMIDITY_BASELINE+humidity_offset)/(HUMIDITY_BASELINE))*(HUMIDITY_WEIGHT*100);
  }

  if(gas_offset > 0){
    gas_score = ((gas_reads[MAX_READ_COUNTS - 1]/mean_reads)*(100-(HUMIDITY_WEIGHT*100)));
  } else {
    gas_score = 100-(HUMIDITY_WEIGHT*100);
  }

  float air_quality_score = humidity_score+gas_score;

  //  display.display();
  packet.temperature = bme.temperature;
  packet.pressure = bme.pressure / 100.0;
  packet.humidity = bme.humidity;
  packet.gas = air_quality_score; // bme.gas_resistance / 1000.0;

  Serial.print("Temperature = ");
  Serial.print(packet.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(packet.pressure);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(packet.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(packet.gas);
  Serial.println(" KOhms");

  Serial.println();

  // definzione del protocollo
  // TEMP_PRESS_HUM_GAS
  String message = String(packet.temperature) + "_" + String(packet.pressure) + "_" +
                   String(packet.humidity) + "_" + String(packet.gas);
  // sendLoRaMessage(message);
  LoRa.beginPacket();
  // LoRa.print(message);
  LoRa.write((uint8_t *)&packet, sizeof(LoRaPacket));
  LoRa.endPacket();
  digitalWrite(PINLED, LOW);
  delay(1000);
  esp_deep_sleep_start();
}