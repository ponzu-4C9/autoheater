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

double DT = 0;
double target = 0;
double k = 0;

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
    
  } else {
    Serial.printf("unknown serial input: '%s'\n", lineBuf);
  }
  lineLen = 0;
}

double pm(double DT){//性能測定
  if (DT < 0) DT = 0;
  if (DT > 1) DT = 1;
  const int len = 10;

  double tempbuf[len] = {0};

  while (1){
    for(int i = len-1;i >= 0;i--){
      tempbuf[i+1] = tempbuf[i];
    }
    thermoCouple.read();
    tempbuf[0] = thermoCouple.getCelsius();

    double sum = 0;
    for(int i = 0;i < len-1;i++){
      sum += abs(tempbuf[i] - tempbuf[i+1]);
    }

    if(sum < 2){
      return DT/(tempbuf[0] - 30.0);//30は室温
    }

    for ( int i = 0; i < len;i++){
      Serial.printf("[%f]", tempbuf[i]);
    }
    Serial.printf("  [%f]\n", sum);


    char buf[64];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "measuring...");
    lcd.print(buf);

    lcd.setCursor(0,1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f|%.2f", tempbuf[0], DT,sum);
    lcd.print(buf);

    digitalWrite(SSRPin, HIGH);
    digitalWrite(2, HIGH);
    delay((int)(500 * DT));
    digitalWrite(SSRPin, LOW);
    digitalWrite(2, LOW);
    delay((int)(500 * (1 - DT)));
  }
  
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
    k = pm(0.01);
    Serial.printf("K = %d\n", k);
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

  //出力の決定
  if(target > temp){
    DT = k*(target - 30) + 0.05;
  }else{
    DT = k*(target - 30) - 0.05;
  }

  Serial.printf("[%d]\n", k*(target - 30),DT);

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