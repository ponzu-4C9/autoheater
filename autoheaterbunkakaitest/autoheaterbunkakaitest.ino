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

double DT = 1;
double temp;
double pre_temp = 0;

void setup() {
  Serial.begin(115200);

  pinMode(SSRPin,OUTPUT);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();

  thermoCouple.read();
  pre_temp = thermoCouple.getCelsius();
}

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
  if (lineBuf[0] == 'd') {
    DT = strtod(&lineBuf[1], nullptr);
    Serial.printf("DT:%f\n",DT);
  } else {
    Serial.printf("unknown serial input: '%s'\n", lineBuf);
  }
  lineLen = 0;
}


uint32_t pre_time = 0; // 1loopの経過時間（ms）を計測するためのもの
uint32_t timestmp20 = 0;

void loop() {
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

  Serial.printf("%d\ttemp:%f\tDT:%f\n",millis(),temp,DT);

  if (DT < 0) DT = 0;
  if (DT > 1) DT = 1;

  char buf[64];

  lcd.setCursor(0,1);
  snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
  lcd.print(buf);

  digitalWrite(SSRPin, HIGH);
  delay((int)(500 * DT));
  digitalWrite(SSRPin, LOW);
  delay((int)(500 * (1 - DT)));
}
