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

double linearRegressionSlope(double x[], int n, double y[]) {
  double sumx = 0.0;
  double sumy = 0.0;
  double sumxy = 0.0;
  double sumx2 = 0.0;
  int count = 0;

  for (int i = 0; i < n; ++i) {
    // 無効点のスキップ (x==0 && y==0)
    if (x[i] == 0.0 && y[i] == 0.0) continue;

    double xi = x[i];
    double yi = y[i];

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

void DTcont(double DT){
  digitalWrite(SSRPin, HIGH);
  delay((int)(900 * DT));
  digitalWrite(SSRPin, LOW);
  delay((int)(900 * (1 - DT)));
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
  double temps[N] = {0};
  double timestamp[N] = {0};
  int i = 0;
  
  while ((double)millis()/1000 - start < 80){//測定
    thermoCouple.read();
    temp = thermoCouple.getCelsius();
    temps[i] = (double)temp;
    timestamp[i] = (double)millis()/1000 - start;

    k = linearRegressionSlope(timestamp,N,temps);

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

    if(DT < 0) DT = 0;
    if(DT > 1) DT = 1;
    


    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "k=%f", k);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }

  double max_temp = 0;
  int flg = 0;
  double DTbuf = DT;
  DT = 0;

  while (1){//90度まで温度上昇
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if(temp > max_temp){
      max_temp = temp;
    }

    if(DT == 0 && temp < 130 - (max_temp - 130) -1){
      DT = DTbuf;
    }
    if(DT == DTbuf && temp > 130 - (max_temp - 130)){
      DT = 0;
    }


    
    double t = (double)millis()/1000;
    if(t != pret){
      Serial.printf("%f\ttemp:%f\tDT:%f\n",t,temp,DT);
    }
    pret = t;


    if(DT < 0) DT = 0;
    if(DT > 1) DT = 1;


    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "maxtemp %.2f", max_temp);
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
}

void loop() {
}
