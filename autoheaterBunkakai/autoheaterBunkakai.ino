#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>

//SCL GPIO22
//SDA GPIO21
const int PWM_Freq  =   5;
const int PWM_Bit   =   8;

const int SSRPin = 15;

const int dataPin   = 4;//so
const int clockPin  = 23;//sck
const int selectPin = 19;//co

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27,20,4);

typedef struct point {
  double temp;
  double timestamp;
} Point;

double linearRegressionSlope(Point ps[], int n) {
  double sumx = 0.0;
  double sumy = 0.0;
  double sumxy = 0.0;
  double sumx2 = 0.0;
  int count = 0;

  for (int i = 0; i < n; ++i) {
    // 無効点のスキップ (x==0 && y==0)
    if (ps[i].timestamp == 0.0 && ps[i].temp == 0.0) continue;

    double xi = ps[i].timestamp;
    double yi = ps[i].temp;

    sumx += xi;
    sumy += yi;
    sumxy += xi * yi;
    sumx2 += xi * xi;
    ++count;
  }

  if (count < 2) {
    // データ不足
    return NAN;
  }

  double meanx = sumx / count;
  double meany = sumy / count;

  // 分子: Σ(xy) - N * meanx * meany
  double numerator = sumxy - (double)count * meanx * meany;
  // 分母: Σ(x^2) - N * meanx^2
  double denominator = sumx2 - (double)count * meanx * meanx;

  const double EPSp = 1e-12;
  if (fabs(denominator) < EPSp) {
    // x の分散が実質ゼロ -> 傾き決定不能
    return NAN;
  }

  return numerator / denominator;
}


double DT;
double temp;
int state = 0;
double k;

void DTcont(double duty){
  if(duty < 0) duty = 0;
  if(duty > 1) duty = 1;
  digitalWrite(SSRPin, HIGH);
  delay((int)(900 * duty));
  digitalWrite(SSRPin, LOW);
  delay((int)(900 * (1 - duty)));
}

double pret = 0;

void setup() {
  Serial.begin(115200);

  pinMode(SSRPin,OUTPUT);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();

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

    double t = (double)millis()/1000;
    if(t != pret){
      Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);
    }
    pret = t;

    DTcont(DT);
  }

  start = (double)millis()/1000;

  const int N = 100;
  Point ps[N] = {0};

  int i = 0;
  
  while ((double)millis()/1000 - start < 80){//測定
    thermoCouple.read();
    temp = thermoCouple.getCelsius();
    ps[i].temp = (double)temp;
    ps[i].timestamp = (double)millis()/1000 - start;

    k = linearRegressionSlope(ps,N);

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.2f|%f", (double)millis()/1000 - start,k);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    double t = (double)millis()/1000;
    if(t != pret){
      Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);
    }
    pret = t;

    DTcont(DT);

    i++;
  }

  k = k/DT;
  DT = 0.05/k;

  double timestamp0 = (double)millis()/1000;//一分に一回出力見直し用タイムスタンプ
  double pre_temp = temp;//前回の温度 一分に一回出力見直し用

  while (1){//90度まで温度上昇
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if(10 < temp && temp < 200){
      if(temp > 130){
        break;
      }
    }

    if((double)millis()/1000 - timestamp0 > 60){
      if(temp - pre_temp > 3){
        DT -= 0.01;
      }else{
        DT += 0.01;
      }
      timestamp0 = (double)millis()/1000;
      pre_temp = temp;
    }


    
    double t = (double)millis()/1000;
    if(t != pret){
      Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);
    }
    pret = t;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "k=%f", k);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }

  DT = 0;

  const int ps1size = 1024;
  Point ps1[ps1size] = {0};
  double max_temp = 0;
  int max_i = 0;
  start = (double)millis()/1000;
  i = 0;
  while (1){//降下測定
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if(temp < 130 - 2){//目標温度より二度下がったらbreak
      break;
    }

    if(temp > max_temp){
      max_temp = temp;
      max_i = i;
    }

    ps1[i].timestamp = (double)millis()/1000 - start;
    ps1[i].temp = temp;
    i++;
    
    double t = (double)millis()/1000;
    if(t != pret){
      Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);
    }
    pret = t;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "Measuring...%d",i);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }

  //最大値を迎えた時間より以前をすべて無効点にする
  for(i = 0; i < max_i;i++){
    ps1[i].timestamp = 0.0;
    ps1[i].temp = 0.0;
  }

  //傾きを調べる
  double l = linearRegressionSlope(ps1,ps1size);

  //最適な出力を
  DT = (l/k)*(130 - 30);

  while (1){//130度で維持
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if((double)millis()/1000 - timestamp0 > 60){
      if(temp > 130){
        DT -= 0.01;
      }else{
        DT += 0.01;
      }
      timestamp0 = (double)millis()/1000;
    }


    
    double t = (double)millis()/1000;
    if(t != pret){
      Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);
    }
    pret = t;

    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "%.3f %.3f", k,l);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }

}

void loop() {
}
