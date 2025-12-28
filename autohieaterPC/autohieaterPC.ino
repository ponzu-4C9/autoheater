#include "MAX6675.h"

#include <LiquidCrystal_I2C.h>

const int SSRPin = 2;
const int dataPin = 8;
const int clockPin = 10;
const int selectPin = 9;


MAX6675 thermoCouple(selectPin, dataPin, clockPin);

LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {

  Serial.begin(115200);

  pinMode(SSRPin,OUTPUT);

  SPI.begin();

  thermoCouple.begin();

  thermoCouple.setSPIspeed(4000000);
  lcd.init();

  lcd.backlight();

}
String rxBuf;
float duty = 0;
unsigned long pret = millis();

// 追記：SSR制御用の変数
unsigned long cycleStartTime = 0;
const unsigned long cycleDuration = 1000; // 制御周期を1000ms (1秒) に設定

void loop() {
  unsigned long now = millis();

  // シリアル通信受信処理（現在のコードをそのまま残します）
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) {
        // 受信した値を duty に設定
        duty = rxBuf.toFloat(); // expects 0.00 - 1.00
        // duty が 0.0 から 1.0 の範囲外にならないようにクランプ
        if (duty < 0.0) duty = 0.0;
        if (duty > 1.0) duty = 1.0;

        rxBuf = "";
      }
    } else {
      rxBuf += c;
    }
  }

  // 1秒ごとの温度読み取りとLCD表示更新
  if (now - pret >= 1000) {
    pret = now;
    // 温度読み取り
    thermoCouple.read();
    float temp = thermoCouple.getCelsius();

    Serial.println(String(temp, 2));

    // LCD表示更新
    char buf[21]; // 20文字+null終端
    lcd.setCursor(0, 1);
    // デューティ比をパーセントで表示する例 (duty * 100)
    snprintf(buf, sizeof(buf), "T:%.2f D:%.0f%%", temp, duty * 100.0);
    lcd.print(buf);
  }

  // --- SSR制御ロジック ---

  // 1秒周期のソフトPWM制御
  if (now - cycleStartTime < cycleDuration) {
    // 制御サイクルの実行中

    // High 期間の計算 (ms)
    // duty = 0.25 なら highDuration は 250ms
    unsigned long highDuration = (unsigned long)(duty * (float)cycleDuration); 

    // 現在のサイクル経過時間
    unsigned long elapsedTime = now - cycleStartTime;

    if (elapsedTime < highDuration) {
      // High 期間
      digitalWrite(SSRPin, HIGH);
    } else {
      // Low 期間
      digitalWrite(SSRPin, LOW);
    }
  } else {
    // 制御サイクルが終了したら、次のサイクルを開始
    cycleStartTime = now;
    // 初期の状態を duty に基づいて設定
    // ここで duty=0 の場合は次のサイクルで High にならないので Low に設定する
    if (duty > 0.0) {
        digitalWrite(SSRPin, HIGH);
    } else {
        digitalWrite(SSRPin, LOW);
    }
    // 注: 1秒周期で動作するため、この条件はすぐに `now - cycleStartTime < cycleDuration` で処理される
  }
}