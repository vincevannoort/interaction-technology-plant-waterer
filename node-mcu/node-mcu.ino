#include "env-variables.h"

// OLED
#include <SSD1306Wire.h>

// WIFI AND MQTT
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY); 

// BME280
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

/**
 * Publishers
 */
Adafruit_MQTT_Publish soil_moisture_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/soil-moisture");
Adafruit_MQTT_Publish light_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/light");
Adafruit_MQTT_Publish pressure_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/pressure");
Adafruit_MQTT_Publish temperature_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/temperature");
Adafruit_MQTT_Publish humidity_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/humidity");

/**
 * Subscribers
 */
Adafruit_MQTT_Subscribe plant_sensor_wildcard_subscriber = Adafruit_MQTT_Subscribe(&mqtt, "plant/treeminator/give-water");

uint32_t x = 0;
void MQTT_connect();

// PINS
int PIN_MULTIPLEXER = D2;
int PIN_SOIL_MOISTURE = A0;
int PIN_LIGHT = A0;
int PIN_SDA = D6;
int PIN_SCL = D7;

/**
 * Soil Moisture sensor
 */
class soil_moisture_sensor {
  public:
  static int read() {
    // set multiplexer board to HIGH
    digitalWrite(PIN_MULTIPLEXER, HIGH);
    return analogRead(PIN_SOIL_MOISTURE);
  }
};

/**
 * Soil Moisture sensor
 */
class light_sensor {
  public:
  static int read() {
    // set multiplexer board to LOW
    digitalWrite(PIN_MULTIPLEXER, LOW);
    return analogRead(PIN_LIGHT);
  }
};

/**
 * OLED display
 */
SSD1306Wire ssd1306_oled_display(0x3c, D3, D5);
class oled_display {
  public:
  static void initialize() {
    ssd1306_oled_display.init();
    ssd1306_oled_display.flipScreenVertically();
    ssd1306_oled_display.setFont(ArialMT_Plain_10);
  }
};

class outside_connection {
  public:
  static void connect_internet() {
    // return if connected
    if (WiFi.status() == WL_CONNECTED) {
      return;
    }

    Serial.print("Connecting to ");
    Serial.println(WLAN_SSID);
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected");
    Serial.println("IP address: "); Serial.println(WiFi.localIP());
    mqtt.subscribe(&plant_sensor_wildcard_subscriber);
  }

  static void connect_mqtt() {
    // return if connected
    if (mqtt.connected()) {
      return;
    }

    int8_t ret;
    Serial.print("Connecting to MQTT... ");
    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0) {
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 5 seconds...");
        mqtt.disconnect();
        delay(5000);
        retries--;
        if (retries == 0) {
          while (1);
        }
    }
    Serial.println("MQTT Connected!");
  }

  static void check_subscription() {
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(100))) {
      if (subscription == &plant_sensor_wildcard_subscriber) {
        Serial.print("Got: ");
        Serial.println((char *)plant_sensor_wildcard_subscriber.lastread);
      }
    }
  }

  static void publish(Adafruit_MQTT_Publish publisher, int value) {
    if (! publisher.publish(value)) {
      Serial.println("Sending to broker failed.");
    }
  }
};

void setup() {
  Serial.begin(115200);
  // Initialise oled screen
  oled_display::initialize();
  // Set pin mode for onboard led
  pinMode(PIN_MULTIPLEXER, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  // Analog pin for light sensor and soil moisture sensor
  pinMode(A0, INPUT);

  // Setup wire protocol
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!bme.begin(&Wire)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }
}

void loop() {
  ssd1306_oled_display.clear();
  ssd1306_oled_display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  ssd1306_oled_display.setFont(ArialMT_Plain_16);
  ssd1306_oled_display.drawString(64, 32, "status: analysing");
  ssd1306_oled_display.display();

  // connect mqtt and wifi
  outside_connection::connect_internet();
  outside_connection::connect_mqtt();

  // publish found values to mqtt broker
  outside_connection::publish(soil_moisture_publisher, soil_moisture_sensor::read());
  outside_connection::publish(light_publisher, light_sensor::read());
  outside_connection::publish(pressure_publisher, bme.readPressure());
  outside_connection::publish(temperature_publisher, bme.readTemperature());
  outside_connection::publish(humidity_publisher, bme.readHumidity());
  delay(100);

  outside_connection::check_subscription();
}