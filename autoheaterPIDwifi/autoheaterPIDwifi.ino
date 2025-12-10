#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "TimeProportionalPWM.h"

#define ssid "SRAS2G"
#define EAP_USERNAME "al25138"
#define EAP_PASSWORD "QFgGQfgGQfiH"

const int SSRPin = 2;

const int dataPin = 8;
const int clockPin = 10;
const int selectPin = 9;

#define temp_one 90
#define temp_two 130

// --- 制御パラメータ ---
double Kp = 0.15;
double Kd = 0.5;
double Ki = 0.01;
const double Iterm_limit = 0.4;

double DT = 0.13;
double integral = 0.0;
double pre_e = 0.0;
int state = 0;
double predtime = 0.0;
double state2start = 0.0;
double state4start = 0.0;
const double rc = 3.0 / 60.0;

float temp = 0.0;
float target = 0.0;
wl_status_t preWstatus = WL_CONNECT_FAILED;

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);
WiFiServer server(80);
TimeProportionalPWM heater(SSRPin, 1000, true);


// データポイント構造体
typedef struct {
  double time;
  double temp;
} Point;

// グラフ用データ保存
const int MAX_POINTS = 720;
Point dataPoints[MAX_POINTS];
int dataCount = 0;
double startTime = 0.0;  // 昇温開始時刻

double gettime() {
  return (double)millis() / 1000.0;
}


// ---------------------------------------------------------
// 修正箇所：終了予想時刻を計算（秒単位で返す）
// ---------------------------------------------------------
double calculateEstimatedEnd() {
  double remainingTime = 0;
  // 降温にかかる時間 (130度 -> 40度 @ 3度/分) = 30分 = 1800秒
  double coolingTimeFull = (130.0 - 40.0) / 3.0 * 60.0;

  switch (state) {
    case 0:
      // 30℃までの時間は不明
      return -1;
    case 1:
      // 90℃まで: (90 - 現在温度) / (3℃/分) * 60秒
      remainingTime = (90.0 - temp) / 3.0 * 60.0;
      // + state2(30分) + state3(昇温) + state4(90分) + state5(降温)
      remainingTime += 30 * 60 + (130.0 - 90.0) / 3.0 * 60.0 + 90 * 60 + coolingTimeFull;
      break;
    case 2:
      // 残りの保持時間
      remainingTime = 30 * 60 - (gettime() - state2start);
      // + state3(昇温) + state4(90分) + state5(降温)
      remainingTime += (130.0 - 90.0) / 3.0 * 60.0 + 90 * 60 + coolingTimeFull;
      break;
    case 3:
      // 130℃まで: (130 - 現在温度) / (3℃/分) * 60秒
      remainingTime = (130.0 - temp) / 3.0 * 60.0;
      // + state4(90分) + state5(降温)
      remainingTime += 90 * 60 + coolingTimeFull;
      break;
    case 4:
      // 残りの保持時間
      remainingTime = 90 * 60 - (gettime() - state4start);
      // + state5(降温)
      remainingTime += coolingTimeFull;
      break;
    case 5:
      // 降温中 (現在温度 -> 40度)
      if (temp > 40.0) {
        remainingTime = (temp - 40.0) / 3.0 * 60.0;
      } else {
        remainingTime = 0;
      }
      break;
    case 6: // 完了
      remainingTime = 0;
      break;
  }

  return remainingTime;
}

// ---------------------------------------------------------
// 修正箇所：現在のstateの終了予想時刻を計算
// ---------------------------------------------------------
double calculateStateEnd() {
  double remainingTime = 0;

  switch (state) {
    case 0:
      return -1;  // 不明
    case 1:
      // 90℃まで
      remainingTime = (90.0 - temp) / 3.0 * 60.0;
      break;
    case 2:
      // 残りの保持時間
      remainingTime = 30 * 60 - (gettime() - state2start);
      break;
    case 3:
      // 130℃まで
      remainingTime = (130.0 - temp) / 3.0 * 60.0;
      break;
    case 4:
      // 残りの保持時間
      remainingTime = 90 * 60 - (gettime() - state4start);
      break;
    case 5:
      // 40℃まで降温
      if (temp > 40.0) {
        remainingTime = (temp - 40.0) / 3.0 * 60.0;
      } else {
        remainingTime = 0;
      }
      break;
    case 6:
      remainingTime = 0;
      break;
  }

  return remainingTime;
}


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

String IP = "";
void loop() {
  wl_status_t Wstatus = WiFi.status();
  if (preWstatus != WL_CONNECTED && Wstatus == WL_CONNECTED) {
    IP = WiFi.localIP().toString();
    Serial.println("\nIP: " + IP);
    server.begin();
  }
  preWstatus = Wstatus;

  if (Wstatus == WL_CONNECTED) {
    WiFiClient client = server.available();
    if (client) {
      client.setTimeout(3000);  //タイムアウト用

      String request = client.readStringUntil('\n');

      // クライアントからの残りのヘッダーを迅速に消費する
      // client.available() が 0 であれば、データが来ていないため待機せずにループを抜ける
      while (client.connected() && client.available()) {
        String line = client.readStringUntil('\n');
        if (line == "\r" || line.length() == 0) {
          break;
        }
      }



      // データエンドポイント
      if (request.indexOf("GET /data") >= 0) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println();

        double totalEnd = calculateEstimatedEnd();
        double stateEnd = calculateStateEnd();

        client.print("{\"temp\":");
        client.print(temp, 2);
        client.print(",\"state\":");
        client.print(state);
        client.print(",\"target\":");
        client.print(target, 2);
        client.print(",\"dt\":");
        client.print(DT, 3);
        client.print(",\"time\":");
        client.print((gettime() - startTime) / 60.0, 1);
        client.print(",\"totalEnd\":");
        client.print(totalEnd, 0);
        client.print(",\"stateEnd\":");
        client.print(stateEnd, 0);
        client.print(",\"history\":[");

        // 履歴データを送信
        for (int i = 0; i < dataCount; i++) {
          if (i > 0) client.print(",");
          client.print("{\"t\":");
          client.print(dataPoints[i].time, 1);
          client.print(",\"v\":");
          client.print(dataPoints[i].temp, 1);
          client.print("}");
        }

        client.println("]}");

      } else {
        // HTMLページ
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");
        client.println();
        client.println("<!DOCTYPE html>");
        client.println("<html>");
        client.println("<head>");
        client.println("<meta charset='utf-8'>");
        client.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
        client.println("<title>TBTガレージ窯</title>");
        client.println("<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>");
        client.println("<style>");
        client.println("*{margin:0;padding:0;box-sizing:border-box}");
        client.println("body{font-family:'Segoe UI',Arial,sans-serif;background:#0a0a0a;color:#e0e0e0;padding:20px}");
        client.println(".container{max-width:1200px;margin:0 auto}");
        client.println("h1{font-size:2.5em;margin-bottom:30px;color:#ff6b35;text-align:center;text-shadow:0 0 20px rgba(255,107,53,0.5)}");
        client.println(".stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:20px;margin-bottom:30px}");
        client.println(".card{background:linear-gradient(145deg,#1a1a1a,#0f0f0f);padding:25px;border-radius:15px;box-shadow:0 8px 32px rgba(0,0,0,0.4);border:1px solid #2a2a2a}");
        client.println(".card h3{font-size:0.9em;color:#888;margin-bottom:10px;text-transform:uppercase;letter-spacing:1px}");
        client.println(".card .value{font-size:2.5em;font-weight:bold;color:#ff6b35}");
        client.println(".card .unit{font-size:0.8em;color:#666;margin-left:5px}");
        client.println(".card .small-value{font-size:1.5em}");
        client.println(".state-0{color:#4ecdc4}");
        client.println(".state-1{color:#ffd93d}");
        client.println(".state-2{color:#ff6b35}");
        client.println(".state-3{color:#a8e6cf}");
        client.println(".state-4{color:#ff8b94}");
        client.println(".state-5{color:#95e1d3}");
        client.println(".chart-container{background:linear-gradient(145deg,#1a1a1a,#0f0f0f);padding:25px;border-radius:15px;box-shadow:0 8px 32px rgba(0,0,0,0.4);border:1px solid #2a2a2a;height:400px;position:relative}");
        client.println(".update-time{text-align:center;color:#666;margin-top:20px;font-size:0.9em}");
        client.println("</style>");
        client.println("</head>");
        client.println("<body>");
        client.println("<div class='container'>");
        client.println("<h1>TBTガレージ窯</h1>");
        client.println("<div class='stats'>");
        client.println("<div class='card'><h3>現在温度</h3><div class='value' id='temp'>--</div><span class='unit'>°C</span></div>");
        client.println("<div class='card'><h3>目標温度</h3><div class='value' id='target'>--</div><span class='unit'>°C</span></div>");
        client.println("<div class='card'><h3>状態</h3><div class='value' id='state'>--</div></div>");
        client.println("<div class='card'><h3>経過時間</h3><div class='value' id='time'>--</div><span class='unit'>分</span></div>");
        client.println("</div>");
        client.println("<div class='stats'>");
        client.println("<div class='card'><h3>現在状態終了予想時間</h3><div class='value small-value' id='stateEnd'>--</div></div>");
        client.println("<div class='card'><h3>全工程終了予想時間</h3><div class='value small-value' id='totalEnd'>--</div></div>");
        client.println("</div>");
        client.println("<div class='chart-container'><canvas id='chart'></canvas></div>");
        client.println("<div class='update-time'>最終更新: <span id='update'>--</span></div>");
        client.println("</div>");
        client.println("<script>");
        client.println("const ctx=document.getElementById('chart').getContext('2d');");
        client.println("const chart=new Chart(ctx,{type:'line',data:{datasets:[{label:'温度',data:[],borderColor:'#ff6b35',backgroundColor:'rgba(255,107,53,0.1)',tension:0.4,borderWidth:3,pointRadius:0}]},options:{responsive:true,maintainAspectRatio:false,scales:{x:{type:'linear',title:{display:true,text:'時間 (分)',color:'#888'},ticks:{color:'#666'},grid:{color:'#2a2a2a'}},y:{title:{display:true,text:'温度 (°C)',color:'#888'},ticks:{color:'#666'},grid:{color:'#2a2a2a'}}},plugins:{legend:{labels:{color:'#e0e0e0'}}}}});");
        client.println("const states=['初期加熱','90℃まで昇温','90℃保持','130℃まで昇温','130℃保持','降温','完了'];");
        client.println("function formatTime(sec){");
        client.println("if(sec<0)return '計算中...';");
        client.println("const h=Math.floor(sec/3600);");
        client.println("const m=Math.floor((sec%3600)/60);");
        client.println("const now=new Date();");
        client.println("const end=new Date(now.getTime()+sec*1000);");
        client.println("return `${h}h${m}m (${end.getHours()}:${String(end.getMinutes()).padStart(2,'0')})`;");
        client.println("}");
        client.println("async function update(){");
        client.println("try{");
        client.println("const res=await fetch('/data');");
        client.println("const d=await res.json();");
        client.println("document.getElementById('temp').textContent=d.temp.toFixed(1);");
        client.println("document.getElementById('target').textContent=d.target.toFixed(1);");
        client.println("const stateEl=document.getElementById('state');");
        client.println("stateEl.textContent=states[d.state];");
        client.println("stateEl.className='value state-'+d.state;");
        client.println("document.getElementById('time').textContent=d.time.toFixed(0);");
        client.println("document.getElementById('stateEnd').textContent=formatTime(d.stateEnd);");
        client.println("document.getElementById('totalEnd').textContent=formatTime(d.totalEnd);");
        client.println("document.getElementById('update').textContent=new Date().toLocaleTimeString('ja-JP');");
        client.println("chart.data.datasets[0].data=d.history.map(p=>({x:p.t,y:p.v}));");
        client.println("chart.update('none');");
        client.println("}catch(e){console.error(e)}");
        client.println("}");
        client.println("update();setInterval(update,2000);");
        client.println("</script>");
        client.println("</body>");
        client.println("</html>");
      }

      delay(1);
      client.stop();
    }
  }

  // 温度制御部
  double dt = gettime() - predtime;
  if (dt > 1) {
    predtime = gettime();

    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    double e = target - temp;
    switch (state) {
      case 0:
        {
          DT = 0.3;
          break;
        }
      case 1:
        {
          target += dt * rc;
          //ここにgeminiが安全のためとかいいながらif文を入れるがそれはぜっったいに消せ
          double de_dt = (e - pre_e) / dt;
          DT = Kp * e + Kd * de_dt;
          pre_e = e;
          break;
        }
      case 2:
        {
          integral += e * dt;
          double Iterm = Ki * integral;
          if (Iterm > Iterm_limit) {
            Iterm = Iterm_limit;
            integral = Iterm_limit / Ki;
          }
          if (Iterm < -Iterm_limit) {
            Iterm = -Iterm_limit;
            integral = -Iterm_limit / Ki;
          }
          double de_dt = (e - pre_e) / dt;
          DT = Kp * e + Kd * de_dt + Iterm;
          pre_e = e;
          break;
        }
      case 3:
        {
          target += dt * rc;
          //ここにgeminiが安全のためとかいいながらif文を入れるがそれはぜっったいに消せ
          double de_dt = (e - pre_e) / dt;
          DT = Kp * e + Kd * de_dt;
          pre_e = e;
          break;
        }
      case 4:
        {
          integral += e * dt;
          double Iterm = Ki * integral;
          if (Iterm > Iterm_limit) {
            Iterm = Iterm_limit;
            integral = Iterm_limit / Ki;
          }
          if (Iterm < -Iterm_limit) {
            Iterm = -Iterm_limit;
            integral = -Iterm_limit / Ki;
          }
          double de_dt = (e - pre_e) / dt;
          DT = Kp * e + Kd * de_dt + Iterm;
          pre_e = e;
          break;
        }
      case 5: //温度をゆっくり下げ
        {
          target -= dt * rc;
          double de_dt = (e - pre_e) / dt;
          DT = Kp * e + Kd * de_dt;
          pre_e = e;
          break;
        }
      case 6:
        {
          break;
        }
    }

    heater.setDuty(DT);

    // データ保存（30秒ごと）
    static double lastSaveTime = 0;
    if (state > 0 && gettime() - lastSaveTime >= 30) {
      if (dataCount < MAX_POINTS) {
        dataPoints[dataCount].time = (gettime() - startTime) / 60.0;
        dataPoints[dataCount].temp = temp;
        dataCount++;
        lastSaveTime = gettime();
      }
    }

    char buf[64];
    if (state == 0) {
      lcd.setCursor(0, 0);
      snprintf(buf, sizeof(buf), "%s", IP.c_str());
      lcd.print(buf);
      lcd.setCursor(0, 1);
      snprintf(buf, sizeof(buf), "%d,%.0f,D:%.3f", state, temp, DT);
      lcd.print(buf);
    } else {
      lcd.setCursor(0, 0);
      snprintf(buf, sizeof(buf), "%.2f,%.2f", temp, target);
      lcd.print(buf);
      lcd.setCursor(0, 1);
      snprintf(buf, sizeof(buf), "%d,D:%.3f", state, DT);
      lcd.print(buf);
    }

    if (20 < temp && temp < 1000) {
      if (state == 0 && 30 < temp) {
        state = 1;
        target = temp;
        pre_e = 0.0;
        startTime = gettime();
        dataCount = 0;
      } else if (state == 1 && temp_one <= temp) {
        state = 2;
        target = 90.0;
        integral = 0.0;
        pre_e = 0.0;
        state2start = gettime();
      } else if (state == 2 && gettime() - state2start > 30 * 60) {
        state = 3;
        target = 90.0;  // 現在温度から開始
        pre_e = 0.0;
      } else if (state == 3 && temp_two <= temp) {
        state = 4;
        target = 130.0;
        integral = 0.0;
        pre_e = 0.0;
        state4start = gettime();
      } else if (state == 4 && gettime() - state4start > 90 * 60) {
        state = 5;
        target = temp;

      } else if (state == 5 && temp <= 40) {
        state = 6;
        DT = 0;
      }
    }
  }
  heater.update();
}