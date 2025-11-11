#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include "linearRegressionSlope.h"
#include "wifi.h"

const int SSRPin = 2;

const int dataPin   = 8;//so
const int clockPin  = 10;//sck
const int selectPin = 9;//co

#define target 90
#define target2 130

#define roomtemp 30

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27,20,4);

WiFiServer server(80);

double DT;
double temp;
int state = 0;
double k;
double t;

void DTcont(double duty){
  if(duty < 0) duty = 0;
  if(duty > 1) duty = 1;
  ledcWrite(SSRPin, (int)duty*225);
  delay(500);
}

double pret = 0;

void setup() {
  Serial.begin(115200);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();

  const int N = 2000;
  static Point ps[N] = {0};

  //窯の温度上昇度合を測定
  DT = 1;
  double start = (double)millis()/1000;
  while ((double)millis()/1000 - start < 60){//60秒まつ
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.2f", (double)millis()/1000 - start);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    t = (double)millis()/1000;
    if(t != pret){
      Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);
    }
    pret = t;

    DTcont(DT);
  }

  start = (double)millis()/1000;

  int i = 0;
  
  while ((double)millis()/1000 - start < 70){//測定
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    ps[i].temp = (double)temp;
    ps[i].timestamp = (double)millis()/1000 - start;

    k = linearRegressionSlope(ps,N)/DT;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.2f|%f", (double)millis()/1000 - start,k);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.4f", temp, DT);
    lcd.print(buf);

    t = (double)millis()/1000;
    if(t != pret){
      Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);
    }
    pret = t;

    DTcont(DT);

    i++;
  }

  DT = (3.0/60)/k;

  //終了時間表示用
  double t0 = (double)millis()/1000;
  double t1 = ((target - temp)/k) + t0;
  double t2 = t1 + 30*60;
  double t3 = (target2 - target)/k + t2;
  double t4 = t3 + 90*60;


  double timestamp0 = (double)millis()/1000;//一分に一回出力見直し用タイムスタンプ
  start = (double)millis()/1000;

  psclear(ps,N);
  i = 0;

  while (1){//target度まで温度上昇
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if(10 < temp && temp < 500){
      if(temp > target){
        break;
      }
    }

    ps[i].temp = (double)temp;
    ps[i].timestamp = (double)millis()/1000 - start;
    i++;

    if((double)millis()/1000 - timestamp0 > 60){
      if(linearRegressionSlope(ps,N) > (3.0/60)){
        DT -= 0.01;
      }else{
        DT += 0.005;
      }
      psclear(ps,N);
      start = (double)millis()/1000;
      i = 0;
      timestamp0 = (double)millis()/1000;
    }
    
    double t = (double)millis()/1000;
    if(t != pret){
      Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);
    }
    pret = t;

    int remtime = t4 - t;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.3f|%dm%ds", k, remtime/60,remtime%60);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }

  DT = 0;

  psclear(ps,N);

  double max_temp = 0;
  int max_i = 0;
  start = (double)millis()/1000;
  i = 0;
  while (1){//降下測定
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if(temp < target - 2 || i >= N){//目標温度より2度下がったらbreak
      break;
    }

    if(temp > max_temp){
      max_temp = temp;
      max_i = i;
    }

    ps[i].timestamp = (double)millis()/1000 - start;
    ps[i].temp = temp;
    i++;

    t = (double)millis()/1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);

    int remtime = t4 - t;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "....|%dm%ds",remtime/60,remtime%60);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }

  //最大値を迎えた時間より以前をすべて無効点にする
  for(i = 0; i < max_i;i++){
    ps[i].timestamp = 0.0;
    ps[i].temp = 0.0;
  }

  //dT/dtを調べる
  double dk = linearRegressionSlope(ps,N);

  //ニュートンの冷却法則の式の比例定数を出す
  double l = dk/(target - roomtemp); // T-T（室温）

  //最適な出力を
  DT = -dk/k;// T-(T（室温）)

  start = (double)millis()/1000;
  lcd.clear();
  while ((double)millis()/1000 - start < t2 - t1){//target度で30分維持
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if((double)millis()/1000 - timestamp0 > 180){
      if(temp > target){
        DT -= 0.01;
      }else if(temp < target){
        DT += 0.01;
      }
      timestamp0 = (double)millis()/1000;
    }

    t = (double)millis()/1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);

    int remtime = t4 - t;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%dm%ds",remtime/60,remtime%60);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }

  DT = ((3.0/60) - l*(temp - roomtemp))/k;

  psclear(ps,N);
  start = (double)millis()/1000;
  i = 0;

  while (1){//target2度まで温度上昇
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if(10 < temp && temp < 500){
      if(temp > target2){
        break;
      }
    }

    ps[i].temp = (double)temp;
    ps[i].timestamp = (double)millis()/1000 - start;
    i++;

    if((double)millis()/1000 - timestamp0 > 60){
      if(linearRegressionSlope(ps,N) > (3.0/60)){
        DT -= 0.01;
      }else{
        DT += 0.01;
      }
      psclear(ps,N);
      start = (double)millis()/1000;
      i = 0;
      timestamp0 = (double)millis()/1000;
    }

    t = (double)millis()/1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);

    int remtime = t4 - t;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%dm%ds", remtime/60,remtime%60);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }

  DT = (-l/k)*(target2 - roomtemp);

  start = (double)millis()/1000;
  timestamp0 = start;
  lcd.clear();
  while ((double)millis()/1000 - start < t4 - t3){//target2度で90分維持
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if((double)millis()/1000 - timestamp0 > 180){
      if(temp > target2){
        DT -= 0.01;
      }else if(temp < target2){
        DT += 0.01;
      }
      timestamp0 = (double)millis()/1000;
    }

    t = (double)millis()/1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);

    int remtime = 90*60 - ((double)millis()/1000 - start);

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%dm%ds", remtime/60,remtime%60);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }

  char buf[64];
  lcd.setCursor(0,0);
  snprintf(buf, sizeof(buf), "finish");
  lcd.print(buf);

}

void loop() {
}
