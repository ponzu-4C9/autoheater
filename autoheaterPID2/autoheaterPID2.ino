

#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>

const int SSRPin = 15;
const int reset_button = 17;

const int dataPin   = 8;  // SO
const int clockPin  = 10; // SCK
const int selectPin = 9; // CS

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27,20,4);


void DTcont(double duty){
  if(duty < 0.0) duty = 0.0;
  if(duty > 1.0) duty = 1.0;
  ledcWrite(SSRPin, duty*225);
}

void setup() {
  Serial.begin(115200);
  pinMode(SSRPin, OUTPUT);

  ledcAttach(SSRPin,4/*周波数*/,8/*ビット数*/);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();

  // --- 制御パラメータ ---
  double Kp = 0.15;
  double Kd = 0.5;
  double Ki = 0.01;    
  const double Iterm_limit = 0.4; 
  // --- 状態変数 ---
  double DT = 0.0;     
  double integral = 0.0; 
  double pre_e = 0.0;    

  const double rc = 3.0 / 60.0; //一分に3度

  float temp = 0.0;

  while(1){
    thermoCouple.read();
    temp = thermoCouple.getCelsius();
  

    if(20 < temp && temp < 1000){
      if(temp > 50){
        break;
      }
    }

    DT = 0.02;//初期出力
    DTcont(DT);
    delay(1000);

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.2f", temp);
    lcd.print(buf);
    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "D:%.3f", DT);
    lcd.print(buf);

  }

  // === ランプ:  -> 90 ===
  double target = 50.0;
  double pret = (double)millis() / 1000.0;
  while (target < 90.0) {
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    double now = (double)millis() / 1000.0;
    double dt = now - pret;
    if (dt <= 0.0) dt = 1e-3;
    pret = now;

    target += dt * rc;

    double e = target - temp;
    double de_dt = (e - pre_e) / dt;
    DT = Kp * e + Kd * de_dt;
    pre_e = e;


    DTcont(DT);
    delay(1000);


    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, target);
    lcd.print(buf);
    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "D:%.3f", DT);
    lcd.print(buf);

    Serial.printf("%f\t temp:%f\t target:%f\t DT:%f\n", now, temp, target, DT);
  }

  // === ホールド: target=90 を 10 分維持（ここで積分制御を有効化） ===
  target = 90.0;
  double start = (double)millis() / 1000.0;
  pret = start;
  pre_e = target - thermoCouple.getCelsius(); // 微分立ち上がりを防ぐ
  integral = 0.0; // 積分初期化
  lcd.clear();

  while ((double)millis() / 1000.0 - start < 10.0 * 60.0) {
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    double now = (double)millis() / 1000.0;
    double dt = now - pret;
    pret = now;

    double e = target - temp;

    // 積分計算（アンチワインドアップ）
    integral += e * dt;
    // 制御上の積分項上限を超えないように制限
    double Iterm = Ki * integral;
    if (Iterm > Iterm_limit) { Iterm = Iterm_limit; integral = Iterm_limit / Ki; }
    if (Iterm < -Iterm_limit) { Iterm = -Iterm_limit; integral = -Iterm_limit / Ki; }

    double de_dt = (e - pre_e) / dt;
    DT = Kp * e + Kd * de_dt + Iterm;
    pre_e = e;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, target);
    lcd.print(buf);
    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "D:%.3f I:%.3f", DT, Iterm);
    lcd.print(buf);

    Serial.printf("%f\t temp:%f\t DT:%f\t Iterm:%f\n", now, temp, DT, Iterm);

    DTcont(DT);
    delay(1000);

  }

  // === ランプ: 90 -> 130 ===
  target = 90.0; // start from 90
  pret = (double)millis() / 1000.0;
  pre_e = target - thermoCouple.getCelsius();
  integral = 0.0; // ランプ時は積分をクリア（ホールド時のみ使う方針）
  lcd.clear();

  while (target < 130.0) {
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    double now = (double)millis() / 1000.0;
    double dt = now - pret;
    pret = now;

    target += dt * rc;

    double e = target - temp;
    double de_dt = (e - pre_e) / dt;
    DT = Kp * e + Kd * de_dt;
    pre_e = e;

    DTcont(DT);
    delay(1000);


    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, target);
    lcd.print(buf);
    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "D:%.3f", DT);
    lcd.print(buf);

    Serial.printf("%f\t temp:%f\t target:%f\t DT:%f\n", now, temp, target, DT);
  }

  // === ホールド: target=130 を 10 分維持（積分有効） ===
  target = 130.0;
  start = (double)millis() / 1000.0;
  pret = start;
  pre_e = target - thermoCouple.getCelsius();
  integral = 0.0;
  lcd.clear();

  while ((double)millis() / 1000.0 - start < 10.0 * 60.0) {
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    double now = (double)millis() / 1000.0;
    double dt = now - pret;
    pret = now;

    double e = target - temp;

    // 積分計算（アンチワインドアップ）
    integral += e * dt;
    double Iterm = Ki * integral;
    if (Iterm > Iterm_limit) { Iterm = Iterm_limit; integral = Iterm_limit / Ki; }
    if (Iterm < -Iterm_limit) { Iterm = -Iterm_limit; integral = -Iterm_limit / Ki; }

    double de_dt = (e - pre_e) / dt;
    DT = Kp * e + Kd * de_dt + Iterm;
    pre_e = e;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, target);
    lcd.print(buf);
    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "D:%.3f I:%.3f", DT, Iterm);
    lcd.print(buf);

    Serial.printf("%f\t temp:%f\t DT:%f\t Iterm:%f\n", now, temp, DT, Iterm);

    DTcont(DT);
    delay(1000);

  }

}

void loop() {
  double temp;
  thermoCouple.read();
  temp = thermoCouple.getCelsius();
  lcd.clear();
  char buf[64];
  lcd.setCursor(0,0);
  snprintf(buf, sizeof(buf), "%.2f", temp);
  lcd.print(buf);
}