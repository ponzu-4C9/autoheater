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

//================ PID Parameters =================
double Kp = 0.15;
double Kd = 0.5;
double Ki = 0.0; // 必要に応じて調整
const double Iterm_limit = 0.4; // 積分項のリミッター

double DT = 0.0;      // 出力Duty比 (0.0 - 1.0)
double integral = 0.0;
double pre_e = 0.0;

//================ 制御変数 =================
float temp = 0.0;
float target = 0.0;   // 初期値は0（OFF）
double predtime = 0.0;

//================ 通信 =================
wl_status_t preWstatus = WL_CONNECT_FAILED;
WiFiServer server(80);

//================ デバイス =================
MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);
TimeProportionalPWM heater(SSRPin, 1000, true);

//================ ログ (リングバッファ的運用) =================
typedef struct {
  double time;
  double temp;
} Point;

const int MAX_POINTS = 720; // グラフの点数
Point dataPoints[MAX_POINTS];
int dataCount = 0;

//================ Utility =================
double gettime() {
  return (double)millis() / 1000.0;
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
  
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi...");

  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);

  predtime = gettime();
}

//================ LOOP =================
String IP = "";

void loop() {
  //---- WiFi Status Check ----
  wl_status_t Wstatus = WiFi.status();
  if (preWstatus != WL_CONNECTED && Wstatus == WL_CONNECTED) {
    IP = WiFi.localIP().toString();
    server.begin();
    Serial.println("IP: " + IP);
    lcd.clear();
  }
  preWstatus = Wstatus;

  //---- Web Server Logic ----
  if (Wstatus == WL_CONNECTED) {
    WiFiClient client = server.available();
    if (client) {
      client.setTimeout(3000);
      String request = client.readStringUntil('\n');
      while (client.connected() && client.available()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") break;
      }

      // --- 1. 温度設定 (GET /set?t=XX) ---
      if (request.indexOf("GET /set?t=") >= 0) {
        int idx = request.indexOf("t=");
        String tStr = request.substring(idx + 2);
        int endIdx = tStr.indexOf(" ");
        if (endIdx > 0) tStr = tStr.substring(0, endIdx);
        
        float newTarget = tStr.toFloat();
        if (newTarget >= 0 && newTarget <= 400) {
            target = newTarget;
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/plain");
            client.println();
            client.print("OK: Target set to ");
            client.print(target);
        } else {
            client.println("HTTP/1.1 400 Bad Request");
        }
      }
      // --- 2. データ取得 (JSON) ---
      else if (request.indexOf("GET /data") >= 0) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println();

        client.print("{\"temp\":");
        client.print(temp, 2);
        client.print(",\"target\":");
        client.print(target, 2);
        client.print(",\"duty\":");
        client.print(DT, 2);
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
      // --- 3. メイン画面 (HTML) ---
      else {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println();
        client.println("<!DOCTYPE html><html><head><meta charset='utf-8'>");
        client.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
        client.println("<title>恒温制御</title>");
        client.println("<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>");
        client.println("<style>body{background:#111;color:#eee;font-family:sans-serif;padding:20px;}");
        client.println("input{font-size:1.2rem;width:100px;padding:5px;} button{font-size:1.2rem;padding:5px 15px;cursor:pointer;}");
        client.println(".val{font-size:1.5em;font-weight:bold;color:#f60}</style></head>");
        
        client.println("<body>");
        client.println("<h1>恒温制御パネル</h1>");
        
        client.println("<div style='margin-bottom:20px;border:1px solid #444;padding:15px;border-radius:8px;'>");
        client.println("<label>目標温度: </label>");
        client.println("<input type='number' id='setVal' step='1' value='" + String((int)target) + "'> ℃ ");
        client.println("<button onclick='sendT()'>設定</button>");
        client.println("</div>");

        client.println("<p>現在温度: <span id='temp' class='val'>--</span> ℃</p>");
        client.println("<p>目標設定: <span id='tgt'>--</span> ℃</p>");
        client.println("<p>出力Duty: <span id='duty'>--</span></p>");
        
        client.println("<canvas id='c' height='300'></canvas>");
        
        client.println("<script>");
        client.println("const ctx=document.getElementById('c').getContext('2d');");
        client.println("const ch=new Chart(ctx,{type:'line',data:{datasets:[{label:'Temp',data:[],borderColor:'#f60',pointRadius:0,borderWidth:2}]},options:{scales:{x:{type:'linear',title:{display:true,text:'Time (sec)'}}}}});");
        
        client.println("async function sendT(){");
        client.println("  const v = document.getElementById('setVal').value;");
        client.println("  if(!v) return;");
        client.println("  try { await fetch('/set?t=' + v); alert('設定しました'); u(); } catch(e){ alert('通信エラー'); }");
        client.println("}");

        client.println("async function u(){");
        client.println("  try {");
        client.println("    const r=await fetch('/data');const d=await r.json();");
        client.println("    document.getElementById('temp').textContent=d.temp.toFixed(1);");
        client.println("    document.getElementById('tgt').textContent=d.target.toFixed(1);");
        client.println("    document.getElementById('duty').textContent=(d.duty*100).toFixed(0)+'%';");
        client.println("    ch.data.datasets[0].data=d.history.map(p=>({x:p.t,y:p.v}));ch.update('none');");
        client.println("  } catch(e){ console.log(e); }");
        client.println("}");
        
        client.println("u();setInterval(u,2000);</script></body></html>");
      }
      client.stop();
    }
  }

  //================ PID制御 =================
  double curTime = gettime();
  double dt = curTime - predtime;
  
  if (dt > 0.5) { // 0.5秒周期で制御更新
    predtime = curTime;

    thermoCouple.read();
    temp = thermoCouple.getCelsius();
    if(isnan(temp)) temp = 0.0; // エラー処理

    double e = target - temp;

    // --- PID計算 ---
    // 目標が0(OFF)なら積分値をリセットして出力も0にする
    if (target <= 0.1) {
        DT = 0;
        integral = 0;
        pre_e = 0;
    } else {
        integral += e * dt;
        
        // Anti-Windup
        double Iterm = Ki * integral;
        if (Iterm > Iterm_limit) { Iterm = Iterm_limit; integral = Iterm_limit / Ki; }
        if (Iterm < -Iterm_limit) { Iterm = -Iterm_limit; integral = -Iterm_limit / Ki; }

        double de_dt = (e - pre_e) / dt;
        DT = Kp * e + Kd * de_dt + Iterm;
        pre_e = e;

        // Duty比クリップ (0.0 - 1.0)
        if (DT > 1.0) DT = 1.0;
        if (DT < 0.0) DT = 0.0;
    }

    heater.setDuty(DT);

    // --- ログ記録 (配列がいっぱいならずらす) ---
    static double lastSave = 0;
    if (curTime - lastSave > 10.0) { // 10秒ごとにログ
      if (dataCount < MAX_POINTS) {
        // 配列に空きがある場合
        dataPoints[dataCount++] = {curTime, (double)temp};
      } else {
        // 配列がいっぱいの場合、全体を左にシフト（古いデータを捨てる）
        for (int i = 0; i < MAX_POINTS - 1; i++) {
          dataPoints[i] = dataPoints[i+1];
        }
        dataPoints[MAX_POINTS - 1] = {curTime, (double)temp};
      }
      lastSave = curTime;
    }

    // --- LCD表示 ---
    lcd.setCursor(0, 0);
    lcd.print("Cur: "); lcd.print(temp, 1); lcd.print(" C   ");
    lcd.setCursor(0, 1);
    lcd.print("Tgt: "); lcd.print((int)target); lcd.print(" D:"); lcd.print((int)(DT*100)); lcd.print("% ");
  }

  heater.update();
}