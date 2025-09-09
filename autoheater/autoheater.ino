#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>

//SCL GPIO22
//SDA GPIO21

const int SSRPin = 15;
const int reset_button = 17;

const int dataPin   = 4;//so
const int clockPin  = 23;//sck
const int selectPin = 19;//co

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27,20,4);

uint32_t elapsed_time = 0; // state に変わった瞬間からの経過時間
uint32_t pre_time = 0; // 1loopの経過時間（ms）を計測するためのもの
int state = 0; // 状態

// // 以前保存した値と比較して変更がある場合のみ保存するための変数
int prev_state = -1;
uint32_t prev_elapsed_time = 0;

volatile bool buttonPressed = true;
float temp;

void setup()
{
  Serial.begin(115200);

  pinMode(SSRPin,OUTPUT);
  pinMode(reset_button,INPUT_PULLUP);
  pinMode(2,OUTPUT);

  Serial.println();
  Serial.println("=== boot ===");
  delay(250);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();

}

double DT = 0;
double target = 130;
double pre_e = 0; // 前回の偏差
double sum_e = 0;
double Kp = 0.1;
double Ki = 0;
double Kd = 0.5;


// シリアルから受け取る1行バッファ（Kp, Kd を変更するため）

static char lineBuf[32];
static size_t lineLen = 0;

static void trimLineInPlace(char* buf, size_t& len) {
  while (len > 0) {
    char c = buf[len - 1];
    if (c == '\r' || c == '\n' || c == ' ' || c == '\t') {
      len--;
    } else break;
  }
  buf[len] = '\0';
  size_t start = 0;
  while (start < len && (buf[start] == ' ' || buf[start] == '\t')) start++;
  if (start > 0) {
    for (size_t i = 0; i + start <= len; ++i) buf[i] = buf[i + start];
    len -= start;
  }
}

static void processLine() {
  lineBuf[lineLen] = '\0';
  trimLineInPlace(lineBuf, lineLen);
  if (lineBuf[0] == 't') {
    target = strtod(&lineBuf[1], nullptr);
  }else if(lineBuf[0] == 'p') {
    Kp = strtod(&lineBuf[1], nullptr);
  }else if(lineBuf[0] == 'i') {
    Ki = strtod(&lineBuf[1], nullptr);
  }else if(lineBuf[0] == 'd') {
    Kd = strtod(&lineBuf[1], nullptr);
  }else {
    Serial.printf("unknown serial input: '%s'\n", lineBuf);
  }
  lineLen = 0;
}

void loop()
{
  // --- シリアル受信処理 ---
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) processLine();
    } else {
      if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
    }
  }

  //温度計読み込み
  thermoCouple.read();
  temp = thermoCouple.getCelsius();

  double e = target - temp;
  sum_e += e;
  DT = Kp*e + Ki*sum_e + Kd * (e - pre_e);

  Serial.printf("DT:%f\ttarget:%.0f\tKp:%f\tKi:%f\tKd:%f\td:%f\n", DT,target,Kp,Ki,Kd,Kd*(e-pre_e));
  
  pre_e = e;

  if (DT < 0) DT = 0;
  if (DT > 1) DT = 1;

  char buf[64];
  lcd.setCursor(0,0);
  //snprintf(buf, sizeof(buf), "%d|%07d", state, elapsed_time);
  lcd.print(buf);

  lcd.setCursor(0,1);
  snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
  lcd.print(buf);

  digitalWrite(SSRPin, HIGH);
  digitalWrite(2, HIGH);
  delay((int)(500 * DT));
  digitalWrite(SSRPin, LOW);
  digitalWrite(2, LOW);
  delay((int)(500 * (1 - DT)));
}
