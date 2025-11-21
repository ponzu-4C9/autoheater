#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include "linearRegressionSlope.h"

//SCL GPIO22
//SDA GPIO21
const int PWM_Freq = 5;
const int PWM_Bit = 8;

const int SSRPin = 15;

const int dataPin = 4;     //so
const int clockPin = 23;   //sck
const int selectPin = 19;  //co

#define target 90         //90℃
#define t2_to_t1 30 * 60  //30分
#define target2 130       //130℃
#define t4_to_t3 90 * 30  //一時間半

#define roomtemp 20  //室温

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);

double DT;
double temp;
int state = 0;
double k;
double t;

void DTcont(double duty) {
  if (duty < 0) duty = 0;
  if (duty > 1) duty = 1;
  digitalWrite(SSRPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS((int)(1000 * duty)));
  digitalWrite(SSRPin, LOW);
  vTaskDelay(pdMS_TO_TICKS((int)(1000 * (1 - duty))));
}

double pret = 0;
void ControlTask(void *pvParameters) {
  const int N = 2000;
  static Point ps[N] = { 0 };

  //窯の温度上昇度合を測定
  DT = 1;
  double start = (double)millis() / 1000;
  while ((double)millis() / 1000 - start < 60) {  //60秒まつ
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%.2f", (double)millis() / 1000 - start);
    lcd.print(buf);

    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    t = (double)millis() / 1000;
    if (t != pret) {
      Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);
    }
    pret = t;

    DTcont(DT);
  }
  state++;

  start = (double)millis() / 1000;

  int i = 0;

  while ((double)millis() / 1000 - start < 70) {  //測定
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    ps[i].temp = (double)temp;
    ps[i].timestamp = (double)millis() / 1000 - start;

    k = linearRegressionSlope(ps, N) / DT;

    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%.2f|%f", (double)millis() / 1000 - start, k);
    lcd.print(buf);

    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.4f", temp, DT);
    lcd.print(buf);

    t = (double)millis() / 1000;
    if (t != pret) {
      Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);
    }
    pret = t;

    DTcont(DT);

    i++;
  }
  state++;

  DT = (3.0 / 60) / k;


  double timestamp0 = (double)millis() / 1000;  //一分に一回出力見直し用タイムスタンプ
  start = (double)millis() / 1000;

  psclear(ps, N);
  i = 0;

  while (1) {  //target度まで温度上昇
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if (10 < temp && temp < 500) {
      if (temp > target) {
        break;
      }
    }

    if (i < N) {
      ps[i].temp = (double)temp;
      ps[i].timestamp = (double)millis() / 1000 - start;
      i++;
    }


    if ((double)millis() / 1000 - timestamp0 > 60) {
      if (linearRegressionSlope(ps, N) > (3.0 / 60)) {
        DT -= 0.01;
      } else {
        DT += 0.005;
      }
      psclear(ps, N);
      start = (double)millis() / 1000;
      i = 0;
      timestamp0 = (double)millis() / 1000;
    }

    double t = (double)millis() / 1000;
    if (t != pret) {
      Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);
    }
    pret = t;

    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%f", k);
    lcd.print(buf);

    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  DT = 0;

  psclear(ps, N);

  double max_temp = 0;
  int max_i = 0;
  start = (double)millis() / 1000;
  i = 0;
  while (1) {  //降下測定
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if (temp < target - 2 || i >= N) {  //目標温度より2度下がったらbreak
      break;
    }

    if (temp > max_temp) {
      max_temp = temp;
      max_i = i;
    }

    ps[i].timestamp = (double)millis() / 1000 - start;
    ps[i].temp = temp;
    i++;

    t = (double)millis() / 1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);

    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "....");
    lcd.print(buf);

    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  //最大値を迎えた時間より以前をすべて無効点にする
  for (i = 0; i < max_i; i++) {
    ps[i].timestamp = 0.0;
    ps[i].temp = 0.0;
  }

  //dT/dtを調べる
  double dk = linearRegressionSlope(ps, N);

  //ニュートンの冷却法則の式の比例定数を出す
  double l = dk / (target - roomtemp);  // T-T（室温）

  //最適な出力を
  DT = -dk / k;  // T-(T（室温）)

  start = (double)millis() / 1000;
  lcd.clear();
  while ((double)millis() / 1000 - start < t2_to_t1) {  //target度で30分維持
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if ((double)millis() / 1000 - timestamp0 > 180) {
      if (temp > target) {
        DT -= 0.01;
      } else if (temp < target) {
        DT += 0.01;
      }
      timestamp0 = (double)millis() / 1000;
    }

    t = (double)millis() / 1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);

    char buf[64];
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  DT = ((3.0 / 60) - l * (temp - roomtemp)) / k;

  psclear(ps, N);
  start = (double)millis() / 1000;
  i = 0;

  while (1) {  //target2度まで温度上昇
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if (10 < temp && temp < 500) {
      if (temp > target2) {
        break;
      }
    }
    if (i < N) {
      ps[i].temp = (double)temp;
      ps[i].timestamp = (double)millis() / 1000 - start;
      i++;
    }


    if ((double)millis() / 1000 - timestamp0 > 60) {
      if (linearRegressionSlope(ps, N) > (3.0 / 60)) {
        DT -= 0.01;
      } else {
        DT += 0.01;
      }
      psclear(ps, N);
      start = (double)millis() / 1000;
      i = 0;
      timestamp0 = (double)millis() / 1000;
    }

    t = (double)millis() / 1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);

    char buf[64];
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  DT = (-l / k) * (target2 - roomtemp);

  start = (double)millis() / 1000;
  timestamp0 = start;
  lcd.clear();
  while ((double)millis() / 1000 - start < t4_to_t3) {  //target2度で90分維持
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if ((double)millis() / 1000 - timestamp0 > 180) {
      if (temp > target2) {
        DT -= 0.01;
      } else if (temp < target2) {
        DT += 0.01;
      }
      timestamp0 = (double)millis() / 1000;
    }

    t = (double)millis() / 1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);

    char buf[64];
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  char buf[64];
  lcd.setCursor(0, 0);
  snprintf(buf, sizeof(buf), "Finished!");
  lcd.print(buf);
}

void webTask(void *pvParameters) {
}

void setup() {
  Serial.begin(115200);

  pinMode(SSRPin, OUTPUT);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();

  xTaskCreatePinnedToCore(
    ControlTask,  // 実行する関数
    "Control",    // タスク名
    10000,        // スタックサイズ (必要に応じて調整)
    NULL,         // パラメータ
    2,            // 優先度
    NULL,         // タスクハンドル
    0             // コア (ESP32C3は実質0)
  );

  xTaskCreatePinnedToCore(
    webTask,
    "Communication",
    10000,
    NULL,
    1,
    NULL,
    0);
}

void loop() {
}
