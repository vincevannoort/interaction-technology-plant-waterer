#include "env-variables.h"

// OLED
#include <SSD1306Wire.h>

// WIFI AND MQTT
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, "/plant/treeminator");
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, "/plant/treeminator");
uint32_t x = 0;
void MQTT_connect();

// PINS
int PIN_MULTIPLEXER = D2;
int PIN_SOIL_MOISTURE = A0;
int PIN_LIGHT = A0;

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
    mqtt.subscribe(&onoffbutton);
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
    while ((subscription = mqtt.readSubscription(5000))) {
      if (subscription == &onoffbutton) {
        Serial.print("Got: ");
        Serial.println((char *)onoffbutton.lastread);
      }
    }
  }

  static void publish(Adafruit_MQTT_Publish publisher) {
    if (! publisher.publish(0)) {
      Serial.println("Failed");
    } else {
      Serial.println("OK!");
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

  // outside_connection::check_subscription();
  outside_connection::publish(photocell);
  delay(500);

  Serial.print("soil_moisture_sensor: ");
  Serial.println(soil_moisture_sensor::read());
  Serial.print("light_sensor: ");
  Serial.println(light_sensor::read());
  // outside_connection::publish(photocell);
}