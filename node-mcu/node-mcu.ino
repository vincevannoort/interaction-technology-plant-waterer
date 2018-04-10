#include "env-variables.h"

// EEPROM
#include <EEPROM.h>

// Bounce
#include <Bounce2.h>

// Timer
#include <SimpleTimer.h>
SimpleTimer publish_sensor_values_timer;

// OLED
#include <Wire.h>
#include <SSD1306Wire.h>

// UI
#include <OLEDDisplayUi.h>

// WIFI AND MQTT
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <WiFiUdp.h>
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY); 

// NTP client
#include <NTPClient.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.cs.uu.nl"); // UU FOR THE WIN!

// Time
#include <Time.h>
#include <TimeLib.h>
time_t last_time_water_given;
time_t time_passed_since_last_time_water_given;
int eeprom_last_time_water_given_address = 0;

// BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

// Watering system
#include <Servo.h>

/**
 * Publishers
 */
Adafruit_MQTT_Publish soil_moisture_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/soil-moisture", 0);
Adafruit_MQTT_Publish light_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/light", 0);
Adafruit_MQTT_Publish pressure_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/pressure", 0);
Adafruit_MQTT_Publish temperature_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/temperature", 0);
Adafruit_MQTT_Publish humidity_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/humidity", 0);
Adafruit_MQTT_Publish mode_publisher = Adafruit_MQTT_Publish(&mqtt, "plant/treeminator/mode", 0);

/**
 * Subscribers
 */
Adafruit_MQTT_Subscribe button_subscriber = Adafruit_MQTT_Subscribe(&mqtt, "plant/treeminator/give-water", 0);
Adafruit_MQTT_Subscribe gesture_subscriber = Adafruit_MQTT_Subscribe(&mqtt, "plant/treeminator/gesture", 0);
Adafruit_MQTT_Subscribe renew_sensor_values_subscriber = Adafruit_MQTT_Subscribe(&mqtt, "plant/treeminator/renew-sensors", 0);

uint32_t x = 0;
void MQTT_connect();

// PINS
int PIN_MULTIPLEXER = D2;
int PIN_SOIL_MOISTURE = A0;
int PIN_LIGHT = A0;
int PIN_SDA = D6;
int PIN_SCL = D7;

enum system_status {
  manual,
  automatic
};
int current_system_status = system_status::automatic;
Bounce system_status_switch = Bounce();

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
int global_soil_moisture = 0;
int global_light = 0;
int global_pressure = 0;
int global_temperature = 0;
int global_humidity = 0;
SSD1306Wire ssd1306_oled_display(0x3c, D6, D7);
OLEDDisplayUi ssd1306_oled_display_ui(&ssd1306_oled_display);

/**
 * Watering system
 */
Servo watering_servo;
class watering_system {
  public:
  static int position;
  static void go_to(int value) {
    // watering_servo.write(value);
    if (value != position) {
      watering_servo.write(value);
      watering_system::position = value;
      if (value >= 130) {
        last_time_water_given = now();
        EEPROM.put(eeprom_last_time_water_given_address, last_time_water_given);
        EEPROM.commit();
      }
    }
  }
  static void go_to_rest_position() {
    if (now() - last_time_water_given > 1) {
      watering_system::go_to(0);
    }
  }
  static void go_to_watering_position() {
    if (now() - last_time_water_given > 5) {
      watering_system::go_to(130);
    }
  }
  static void has_enough_water(int moisture_level) {
    // if the mode is not automatic, it should not do anything
    if (current_system_status != system_status::automatic) {
      return;
    }
    if (moisture_level < 80) {
      Serial.println("Under moisture level...");
      watering_system::go_to_watering_position();
    } else {
      Serial.println("Above moisture level...");
      watering_system::go_to_rest_position();
    }
  }
};
int watering_system::position = 0;

/**
 * Soil Moisture sensor
 */
class soil_moisture_sensor {
  public:
  static int read_once() {
    digitalWrite(PIN_MULTIPLEXER, HIGH);
    int value = analogRead(PIN_SOIL_MOISTURE);
    return value;
  }
  static int read() {
    // set multiplexer board to HIGH
    int total = 0;
    int total_readings = 5;

    for(int i = 0; i <= total_readings; i++) {
      total += soil_moisture_sensor::read_once();
      delay(10);
    }
    watering_system::has_enough_water(total / total_readings);
    return total / total_readings;
  }
};

/**
 * System status LED
 */
class system_status_led {
  public:
  static void display_current_status() {
    // check if button has updated
    system_status_switch.update();
    if (!system_status_switch.fell()) {
      return;
    }

    // turn led in the correct status
    current_system_status = !current_system_status;
    if (current_system_status == system_status::automatic) {
      mode_publisher.publish("automatic");
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    } else {
      mode_publisher.publish("manual");
      watering_system::go_to_rest_position();
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
  }
};

/**
 * Outside connection
 */
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
    mqtt.subscribe(&button_subscriber);
    mqtt.subscribe(&gesture_subscriber);
    mqtt.subscribe(&renew_sensor_values_subscriber);
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
    while ((subscription = mqtt.readSubscription(50))) {
      if (subscription == &button_subscriber) {
        watering_system::go_to(String((char *)button_subscriber.lastread).toInt());
      }
      if (subscription == &gesture_subscriber) {
        Serial.println("Recieved a gesture of finger count");
        int finger_count = ((char *)gesture_subscriber.lastread)[10] - '0';
        if (finger_count == 2) {
          watering_system::go_to_watering_position();
        } else {
          watering_system::go_to_rest_position();
        }
      }
      if (subscription == &renew_sensor_values_subscriber) {
        Serial.println("Renewing sensor values on command");
        outside_connection::publish_sensor_values();
      }
    }
  }

  static void publish_sensor_values() {
    global_soil_moisture = soil_moisture_sensor::read();
    global_light = light_sensor::read();
    global_pressure = bme.readPressure() / 1000;
    global_temperature = bme.readTemperature();
    global_humidity = bme.readHumidity();
    outside_connection::publish(soil_moisture_publisher, soil_moisture_sensor::read());
    outside_connection::publish(light_publisher, light_sensor::read());
    outside_connection::publish(pressure_publisher, bme.readPressure() / 1000);
    outside_connection::publish(temperature_publisher, bme.readTemperature());
    outside_connection::publish(humidity_publisher, bme.readHumidity());
  }

  static void publish(Adafruit_MQTT_Publish publisher, int value) {
    if (! publisher.publish(value)) {
      Serial.println("Sending to broker failed.");
    }
  }
};

/**
 * UI frames
 */
void screen_test1(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(15 + 5 + x, 0 + y,   "moisture:");
  display->drawString(15 + 75 + x, 0 + y,  String(global_soil_moisture));
  display->drawString(15 + 5 + x, 10 + y,  "light:");
  display->drawString(15 + 75 + x, 10 + y, String(global_light));
  display->drawString(15 + 5 + x, 20 + y,  "pressure:");
  display->drawString(15 + 75 + x, 20 + y, String(global_pressure));
  display->drawString(15 + 5 + x, 30 + y,  "temperature:");
  display->drawString(15 + 75 + x, 30 + y, String(global_temperature));
  display->drawString(15 + 5 + x, 40 + y,  "humidity:");
  display->drawString(15 + 75 + x, 40 + y, String(global_humidity));
}
void screen_test2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  time_passed_since_last_time_water_given = now() - last_time_water_given;
  display->drawString(64 + x, 8 + y, "time passed");
  display->drawString(64 + x, 18 + y, "since last watering:");
  int i_hour = hour(time_passed_since_last_time_water_given);
  String s_hour = (i_hour < 10) ? "0" + String(i_hour) : String(i_hour);
  int i_minute = minute(time_passed_since_last_time_water_given);
  String s_minute = (i_minute < 10) ? "0" + String(i_minute) : String(i_minute);
  int i_second = second(time_passed_since_last_time_water_given);
  String s_second = (i_second < 10) ? "0" + String(i_second) : String(i_second);
  display->drawString(64 + x, 36 + y, s_hour + ":" + s_minute + ":" + s_second);
}
FrameCallback frames[] = { screen_test1, screen_test2 };
int frames_length = 2;
const uint8_t activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const uint8_t inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};

/**
 * Setup
 */
void setup() {
  Serial.begin(115200);
  // flash button
  pinMode(D3, INPUT_PULLUP);
  system_status_switch.attach(D3);
  system_status_switch.interval(5);
  // Initialise oled screen
  ssd1306_oled_display_ui.setTargetFPS(60);
  ssd1306_oled_display_ui.setActiveSymbol(activeSymbol);
  ssd1306_oled_display_ui.setInactiveSymbol(inactiveSymbol);
  ssd1306_oled_display_ui.setIndicatorPosition(BOTTOM);
  ssd1306_oled_display_ui.setIndicatorDirection(LEFT_RIGHT);
  ssd1306_oled_display_ui.setFrameAnimation(SLIDE_LEFT);
  ssd1306_oled_display_ui.setFrames(frames, frames_length);
  ssd1306_oled_display_ui.setTimePerFrame(2000);
  ssd1306_oled_display_ui.setTimePerTransition(0);
  ssd1306_oled_display_ui.enableAutoTransition();
  ssd1306_oled_display_ui.init();
  ssd1306_oled_display.flipScreenVertically();
  // Set pin mode for multiplexer pin
  pinMode(PIN_MULTIPLEXER, OUTPUT);
  // Set pin mode for onboard led
  pinMode(LED_BUILTIN, OUTPUT);
  // Analog pin for light sensor and soil moisture sensor
  pinMode(A0, INPUT);
  // Set pin for watering system
  watering_servo.attach(D8);
  watering_servo.write(0);
  // Setup wire protocol
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!bme.begin(&Wire)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }
  // EEPROM
  EEPROM.begin(512);
  EEPROM.get(eeprom_last_time_water_given_address, last_time_water_given);
  // Set timer for publishing values
  publish_sensor_values_timer.setInterval(500, outside_connection::publish_sensor_values);
  // connect mqtt and wifi
  outside_connection::connect_internet();
  outside_connection::connect_mqtt();
  // Set time from NTP library
  timeClient.begin();
  timeClient.update();
  setTime(timeClient.getEpochTime());
}

/**
 * Loop
 */
void loop() {
  // connect mqtt and wifi
  outside_connection::connect_internet();
  outside_connection::connect_mqtt();

  // update the oled ui
  ssd1306_oled_display_ui.update();
  system_status_led::display_current_status();

  // publish found values to mqtt broker
  publish_sensor_values_timer.run();

  // check if there are any new values from subscriptions
  outside_connection::check_subscription();
}