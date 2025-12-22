#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include "TimeProportionalPWM.h"

// ピン設定
const int SSRPin = 2;
const int dataPin = 8;
const int clockPin = 10;
const int selectPin = 9;


MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);
TimeProportionalPWM heater(SSRPin, 1000, true);

void setup() {
  Serial.begin(115200);

  // ヒーター初期化
  heater.begin();

  // 熱電対初期化
  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);

  // LCD初期化
  lcd.init();
  lcd.backlight();
}

String rxBuf;


void loop() {

  // 温度読み取り
  thermoCouple.read();
  float temp = thermoCouple.getCelsius();
  float duty;

  Serial.println(String(temp, 2));

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) {
        duty = rxBuf.toFloat(); // expects 0.00 - 1.00
        heater.setDuty(duty);
        rxBuf = "";
      }
    } else {
      rxBuf += c;
    }
  }

  // LCD表示更新
  char buf[21]; // 20文字+null終端
  lcd.setCursor(0, 1);
  snprintf(buf, sizeof(buf), "%.f,%.f", temp, duty);
  lcd.print(buf);
  heater.update();

  delay(300);
}