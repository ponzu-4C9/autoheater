#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

//SCL GPIO22
//SDA GPIO21

const int SSRPin = 15;
const int reset_button = 17;

const int dataPin   = 4;//so
const int clockPin  = 23;//sck
const int selectPin = 19;//co

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27,20,4);

Preferences preferences;
uint32_t elapsed_time = 0; // state に変わった瞬間からの経過時間
uint32_t pre_time = 0; // 1loopの経過時間（ms）を計測するためのもの
int state = 0; // 状態

// // 以前保存した値と比較して変更がある場合のみ保存するための変数
int prev_state = -1;
uint32_t prev_elapsed_time = 0;

volatile bool buttonPressed = true;
float temp;
float temp0;

void IRAM_ATTR reset()
{
  buttonPressed = true;
}

void setup()
{
  Serial.begin(115200);

  pinMode(SSRPin,OUTPUT);
  pinMode(reset_button,INPUT_PULLUP);
  pinMode(2,OUTPUT);

  Serial.println();
  Serial.println("=== boot ===");
  attachInterrupt(reset_button, reset, FALLING);
  delay(250);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();

  // Preferences を開く（読み書きモード）
  preferences.begin("my-app", false);

  // 起動時に保存値を読み出す（キーがなければデフォルト）
  state = preferences.getUInt("state", 1);
  elapsed_time = preferences.getUInt("elapsed_time", 0);

  // デバッグ出力
  Serial.printf("prefs read: state=%d, elapsed_time=%u\n", state, elapsed_time);

  // 初期 prev 保存値をセット
  prev_state = state;
  prev_elapsed_time = elapsed_time;
  pre_time = millis();
}

double DT = 0;
double target = 89;
double pre_e = 0; // 前回の偏差


// シリアルから受け取る1行バッファ（Kp, Kd を変更するため）
double Kp = 0.15;
double Kd = 0.5;
double Ki = 0;
double e_sum = 0;

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
  if (lineBuf[0] == 'p') {
    Kp = strtod(&lineBuf[1], nullptr);
    Serial.printf("Kp set to %f\n", Kp);
  } else if (lineBuf[0] == 'd') {
    Kd = strtod(&lineBuf[1], nullptr);
    Serial.printf("Kd set to %f\n", Kd);
  }else if(lineBuf[0] == 'i'){
    Ki = strtod(&lineBuf[1],nullptr);
    Serial.printf("Ki set to %f\n", Ki);
  } else {
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

  //リセットボタン処理
  if (buttonPressed) {
    target = temp;
    state=0;
    elapsed_time = 0;
    buttonPressed = false;
  }

  //状態遷移処理
  uint32_t dt = millis()-pre_time;//一loopの経過時間
  elapsed_time += dt;
  pre_time = millis();


  if (state == 0 && target > 90) {
    state = 1;
    elapsed_time = 0;
  }
  if (state == 1 && elapsed_time > 30*60*1000) {//単位はmsなのでかける1000
    state = 2;
    elapsed_time = 0;
  }
  if (state == 2 && target > 130) {
    state = 3;
    elapsed_time = 0;
  }
  if (state == 3 && elapsed_time > 90*60*1000) {
    state = 4;
    elapsed_time = 0;
  }


  if (state == 0 || state == 2) {
    target += (3.0/(60*1000)) * dt;
  } else if (state == 1) {
    target = 90;
  } else if (state == 3) {
    target = 130;
  } else if (state == 4) {
    target = 0;
  }

  // --- Preferences に変更があったら保存（前回と比べて変化がある場合のみ） ---
  if (state != prev_state) {
    preferences.putUInt("state", (uint32_t)state);
    Serial.printf("prefs write: state=%d\n", state);
    prev_state = state;
  }
  if (abs((int)(elapsed_time - prev_elapsed_time)) > 10000) {
    preferences.putUInt("elapsed_time", elapsed_time);
    Serial.printf("prefs write: elapsed_time=%d\n", elapsed_time);
    prev_elapsed_time = elapsed_time;
  }

  double e = target - temp;
  e_sum += e;
  DT = Kp * e + Kd * (e - pre_e) + Ki*e_sum;
  pre_e = e;

  Serial.printf("elapsed_time:%d\tKp:%f\tKd%f\tKi:%f\tDT:%f\n", elapsed_time,Kp,Kd,Ki,DT);

  if (DT < 0) DT = 0;
  if (DT > 1) DT = 1;

  char buf[64];
  lcd.setCursor(0,0);
  snprintf(buf, sizeof(buf), "%d|%07d", state, elapsed_time);
  lcd.print(buf);

  lcd.setCursor(0,1);
  snprintf(buf, sizeof(buf), "%.2f|%.2f|%.2f", temp, target, DT);
  lcd.print(buf);

  digitalWrite(SSRPin, HIGH);
  digitalWrite(2, HIGH);
  delay((int)(500 * DT));
  digitalWrite(SSRPin, LOW);
  digitalWrite(2, LOW);
  delay((int)(500 * (1 - DT)));
}