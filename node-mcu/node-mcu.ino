#include "env-variables.h"

// OLED
#include <SSD1306Wire.h>


// WIFI AND MQTT
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, "/feeds/photocell");
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, "/feeds/onoff");
uint32_t x = 0;
void MQTT_connect();

/**
 * Soil Moisture sensor
 */
class soil_moisture_sensor {

};

/**
 * OLED display
 */
SSD1306Wire ssd1306_oled_display(0x3c, D3, D5);
class oled_display {

};

void setup() {
  Serial.begin(9600);

  // OLED
  ssd1306_oled_display.init();
  ssd1306_oled_display.flipScreenVertically();
  ssd1306_oled_display.setFont(ArialMT_Plain_10);

  // WIFI AND MQTT
  Serial.begin(115200);
  delay(10);
  Serial.println(F("Adafruit MQTT demo"));
  Serial.println(); Serial.println();
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

  // LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  ssd1306_oled_display.clear();
  ssd1306_oled_display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  ssd1306_oled_display.setFont(ArialMT_Plain_16);
  ssd1306_oled_display.drawString(64, 32, "status: analysing");
  ssd1306_oled_display.display();

  // MQTT
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);
    }
  }
  Serial.print(F("\nSending photocell val "));
  Serial.print(x);
  Serial.print("...");
  if (! photocell.publish(x++)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(5000);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(5000);
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}