/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings
  
  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2652

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <driver/i2s.h>
#include <esp_task_wdt.h>
#include "I2SMicSampler.h"
#include "config.h"
#include "Application.h"
#include "SPIFFS.h"


Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

HX711 scale;

uint8_t dataPin = 2;
uint8_t clockPin = 0;
volatile float f;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

// i2s config for reading from both channels of I2S
i2s_config_t i2sMemsConfigBothChannels = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_MIC_CHANNEL,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s microphone pins
i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

// This task does all the heavy lifting for our application
void applicationTask(void *param)
{
  Application *application = static_cast<Application *>(param);

  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
  while (true)
  {
    // wait for some audio samples to arrive
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      application->run();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  //delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.print("WiFi con");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSWD);

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    display.print("\nFailed...");
    display.display();
    delay(5000);
    ESP.restart();
  }
  // display.println();
  display.println("\nConnected!");
  display.println(WiFi.localIP());
  display.display();

  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());

  // make sure we don't get killed for our long running tasks
  esp_task_wdt_init(10, false);

  I2SSampler *i2s_sampler = new I2SMicSampler(i2s_mic_pins, false);

  // create our application
  Application *application = new Application(i2s_sampler);

  Serial.println(F("BME280 Sensor event test"));

  if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }
  
  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  scale.begin(dataPin, clockPin);
  scale.set_scale(9140);
  scale.tare();

  // set up the i2s sample writer task
  TaskHandle_t applicationTaskHandle;
  xTaskCreate(applicationTask, "Application Task", 8192, application, 1, &applicationTaskHandle);

  i2s_sampler->start(I2S_NUM_0, i2sMemsConfigBothChannels, applicationTaskHandle);

  // delay(2000);
}

void loop() {
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);
  
  display.clearDisplay();
  display.setCursor(0, 0);
  
  display.print(F("Temperature = "));
  display.print(temp_event.temperature);
  display.println("C");

  display.print(F("Humidity = "));
  display.print(humidity_event.relative_humidity);
  display.println("%");

  display.print(F("Pressure = "));
  display.print(pressure_event.pressure);
  display.println("hPa");

  float wght = scale.get_units(5);
  display.print(F("Weight = "));
  display.print(wght);
  display.println("kg");

  display.display();

  // f = scale.get_units(5);
  // Serial.println(f);

  delay(1000);

  if (WiFi.status() == WL_CONNECTED){
    HTTPClient client;

    DynamicJsonDocument doc(1024);
    doc["temperature"] = temp_event.temperature;
    doc["humidity"] = humidity_event.relative_humidity;
    doc["pressure"] = pressure_event.pressure;
    doc["weight"] = wght;
    doc["hive"] = HIVE_ID;

    String pbody;
    serializeJson(doc, pbody);

    client.begin("https://hivemanager.azurewebsites.net/api/data");
    client.addHeader("Content-Type", "application/json");
    int httpCode = client.POST(pbody);

    if (httpCode > 0)
    {
      String response = client.getString();
      Serial.println("\nStatuscode: " + String(httpCode));
      Serial.println(response);

      // DynamicJsonDocument doc(1024);
      // deserializeJson(doc, response);

      // const char* hive = doc["hive"];
      // double temp = doc["temperature"];
      // double weight = doc["weight"];

      // Serial.println(hive);
      // Serial.println(temp);
      // Serial.println(weight);
    }
    else {
      Serial.println("Error on HTTP request");
    }
    
  } 
  else {
    Serial.println("Conncetion lost");
  }

  delay(30000);
}