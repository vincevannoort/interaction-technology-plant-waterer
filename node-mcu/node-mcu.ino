// #include <OLEDDisplay.h>
// #include <OLEDDisplayFonts.h>
// #include <OLEDDisplayUi.h>
// #include <SH1106.h>
// #include <SH1106Spi.h>
// #include <SH1106Wire.h>
// #include <SSD1306.h>
// #include <SSD1306Spi.h>
#include <SSD1306Wire.h>

/**
 * Soil Moisture sensor
 */
class soil_moisture_sensor {

};

/**
 * Servo motor
 */
class servo_motor {

};

/**
 * OLED display
 */
SSD1306Wire ssd1306_oled_display(0x3c, D3, D5);
class oled_display {

};

void setup() {
  ssd1306_oled_display.init();
  ssd1306_oled_display.flipScreenVertically();
  ssd1306_oled_display.setFont(ArialMT_Plain_10);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  ssd1306_oled_display.clear();
  ssd1306_oled_display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  ssd1306_oled_display.setFont(ArialMT_Plain_16);
  ssd1306_oled_display.drawString(64, 32, "status: analysing");
  ssd1306_oled_display.display();

  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(5000);
}