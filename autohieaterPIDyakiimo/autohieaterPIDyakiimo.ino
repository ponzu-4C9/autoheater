#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "TimeProportionalPWM.h"

//================ WiFi =================
#define ssid "SRAS2G"
#define EAP_USERNAME "al25138"
#define EAP_PASSWORD "QFgGQfgGQfiH"

//================ Hardware =================
const int SSRPin = 2;
const int dataPin = 8;
const int clockPin = 10;
const int selectPin = 9;

//================ 制御条件 =================
#define TEMP_STATE0 75.0
#define STATE0_HOLD_TIME (60.0 * 60.0)   // 60分（秒）

//================ PID =================
double Kp = 0.15;
double Kd = 0.5;
double Ki = 0.01;
const double Iterm_limit = 0.4;

double DT = 0.0;
double integral = 0.0;
double pre_e = 0.0;

//================ 状態管理 =================
int state = 0;
double state0start = 0.0;
double predtime = 0.0;

//================ センサ =================
float temp = 0.0;
float target = 0.0;

//================ 通信 =================
wl_status_t preWstatus = WL_CONNECT_FAILED;
WiFiServer server(80);

//================ デバイス =================
MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);
TimeProportionalPWM heater(SSRPin, 1000, true);

//================ ログ =================
typedef struct {
  double time;
  double temp;
} Point;

const int MAX_POINTS = 720;
Point dataPoints[MAX_POINTS];
int dataCount = 0;
double startTime = 0.0;

//================ Utility =================
double gettime() {
  return (double)millis() / 1000.0;
}

//================ 残り時間計算 =================
double calculateStateEnd() {
  switch (state) {
    case 0:
      return STATE0_HOLD_TIME - (gettime() - state0start);
    case 1:
    case 2:
      return 0;
  }
  return 0;
}

double calculateEstimatedEnd() {
  return calculateStateEnd();
}

//================ SETUP =================
void setup() {
  Serial.begin(115200);

  heater.begin();
  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);

  lcd.init();
  lcd.backlight();

  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);

  predtime = gettime();
}

//================ LOOP =================
String IP = "";

void loop() {

  //---- WiFi ----
  wl_status_t Wstatus = WiFi.status();
  if (preWstatus != WL_CONNECTED && Wstatus == WL_CONNECTED) {
    IP = WiFi.localIP().toString();
    server.begin();
    Serial.println("IP: " + IP);
  }
  preWstatus = Wstatus;

  //---- Web ----
  if (Wstatus == WL_CONNECTED) {
    WiFiClient client = server.available();
    if (client) {
      client.setTimeout(3000);
      String request = client.readStringUntil('\n');
      while (client.connected() && client.available()) client.readStringUntil('\n');

      // JSON
      if (request.indexOf("GET /data") >= 0) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println();

        client.print("{\"temp\":");
        client.print(temp, 2);
        client.print(",\"state\":");
        client.print(state);
        client.print(",\"target\":");
        client.print(target, 2);
        client.print(",\"time\":");
        client.print((gettime() - startTime) / 60.0, 1);
        client.print(",\"stateEnd\":");
        client.print(calculateStateEnd(), 0);
        client.print(",\"totalEnd\":");
        client.print(calculateEstimatedEnd(), 0);
        client.print(",\"history\":[");

        for (int i = 0; i < dataCount; i++) {
          if (i > 0) client.print(",");
          client.print("{\"t\":");
          client.print(dataPoints[i].time, 1);
          client.print(",\"v\":");
          client.print(dataPoints[i].temp, 1);
          client.print("}");
        }
        client.println("]}");
      }
      // HTML
      else {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println();
        client.println("<!DOCTYPE html><html><head><meta charset='utf-8'>");
        client.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
        client.println("<title>65℃保持制御</title>");
        client.println("<script src='https://cdn.jsdelivr.net/npm/chart.js'></script></head>");
        client.println("<body style='background:#111;color:#eee;font-family:sans-serif'>");
        client.println("<h1>65℃保持制御</h1>");
        client.println("<p>温度: <span id='temp'>--</span> ℃</p>");
        client.println("<p>状態: <span id='state'>--</span></p>");
        client.println("<p>残り時間: <span id='end'>--</span></p>");
        client.println("<canvas id='c' height='300'></canvas>");
        client.println("<script>");
        client.println("const s=['65℃保持','ヒーターOFF','完了'];");
        client.println("const ctx=document.getElementById('c').getContext('2d');");
        client.println("const ch=new Chart(ctx,{type:'line',data:{datasets:[{label:'Temp',data:[],borderColor:'#f60',pointRadius:0}]},options:{scales:{x:{type:'linear'}}}});");
        client.println("async function u(){const r=await fetch('/data');const d=await r.json();");
        client.println("temp.textContent=d.temp.toFixed(1);");
        client.println("state.textContent=s[d.state];");
        client.println("end.textContent=(d.stateEnd/60).toFixed(1)+' 分';");
        client.println("ch.data.datasets[0].data=d.history.map(p=>({x:p.t,y:p.v}));ch.update('none');}");
        client.println("u();setInterval(u,2000);</script></body></html>");
      }
      client.stop();
    }
  }

  //================ 温度制御 =================
  double dt = gettime() - predtime;
  if (dt > 1.0) {
    predtime = gettime();

    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    double e = target - temp;

    switch (state) {
      case 0: {
        target = TEMP_STATE0;

        integral += e * dt;
        double Iterm = Ki * integral;
        if (Iterm > Iterm_limit) { Iterm = Iterm_limit; integral = Iterm_limit / Ki; }
        if (Iterm < -Iterm_limit) { Iterm = -Iterm_limit; integral = -Iterm_limit / Ki; }

        double de_dt = (e - pre_e) / dt;
        DT = Kp * e + Kd * de_dt + Iterm;
        pre_e = e;

        if (state0start == 0.0) {
          state0start = gettime();
          startTime = state0start;
        }

        if (gettime() - state0start >= STATE0_HOLD_TIME) {
          state = 1;
          DT = 0;
        }
        break;
      }

      case 1:
        DT = 0;
        state = 2;
        break;

      case 2:
        DT = 0;
        break;
    }

    heater.setDuty(DT);

    // ログ（30秒）
    static double lastSave = 0;
    if (state == 0 && gettime() - lastSave > 30) {
      if (dataCount < MAX_POINTS) {
        dataPoints[dataCount++] = {(gettime() - startTime) / 60.0, temp};
      }
      lastSave = gettime();
    }

    // LCD
    lcd.setCursor(0, 0);
    lcd.print(temp); lcd.print(" / "); lcd.print(target);
    lcd.setCursor(0, 1);
    lcd.print("S:"); lcd.print(state); lcd.print(" D:"); lcd.print(DT, 2);
  }

  heater.update();
}
