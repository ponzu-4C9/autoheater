#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "TimeProportionalPWM.h"

#define ssid "SRAS2G"
#define EAP_USERNAME "al25138"
#define EAP_PASSWORD "QFgGQfgGQfiH"
const char CONTROL_PASSWORD[] = "mthuji";

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
  double elapsedSec;
  double temp;
  double target;
  int state;
  double duty;
} Point;

// グラフ用データ保存
const int MAX_POINTS = 720;
Point dataPoints[MAX_POINTS];
int dataCount = 0;
double startTime = 0.0;  // 昇温開始時刻
double lastSaveTime = 0.0;

enum ProcessMode {
  MODE_FULL = 0,
  MODE_SKIP_90_HOLD = 1,
};

ProcessMode processMode = MODE_FULL;

double gettime() {
  return (double)millis() / 1000.0;
}

String getRequestPath(const String& requestLine) {
  int firstSpace = requestLine.indexOf(' ');
  if (firstSpace < 0) return "/";
  int secondSpace = requestLine.indexOf(' ', firstSpace + 1);
  if (secondSpace < 0) return "/";
  return requestLine.substring(firstSpace + 1, secondSpace);
}

String stripQuery(const String& path) {
  int queryIndex = path.indexOf('?');
  if (queryIndex < 0) return path;
  return path.substring(0, queryIndex);
}

String getQueryParam(const String& path, const String& key) {
  int queryIndex = path.indexOf('?');
  if (queryIndex < 0) return "";

  String query = path.substring(queryIndex + 1);
  int pos = 0;
  while (pos <= query.length()) {
    int amp = query.indexOf('&', pos);
    if (amp < 0) amp = query.length();
    String pair = query.substring(pos, amp);
    int eq = pair.indexOf('=');
    if (eq >= 0) {
      String name = pair.substring(0, eq);
      String value = pair.substring(eq + 1);
      value.replace("+", " ");
      if (name == key) return value;
    }
    pos = amp + 1;
  }
  return "";
}

bool canChangeMode() {
  return state == 1 && temp <= temp_one;
}

const char* getModeKey() {
  return processMode == MODE_FULL ? "full" : "short130";
}

const char* getModeLabel() {
  return processMode == MODE_FULL ? "標準工程" : "130℃工程";
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
      if (processMode == MODE_FULL) {
        remainingTime += 30 * 60;
      }
      remainingTime += (130.0 - 90.0) / 3.0 * 60.0 + 90 * 60 + coolingTimeFull;
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
      request.trim();
      String requestPath = getRequestPath(request);
      String route = stripQuery(requestPath);

      // クライアントからの残りのヘッダーを迅速に消費する
      // client.available() が 0 であれば、データが来ていないため待機せずにループを抜ける
      while (client.connected() && client.available()) {
        String line = client.readStringUntil('\n');
        if (line == "\r" || line.length() == 0) {
          break;
        }
      }



      // データエンドポイント
      if (route == "/data") {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Cache-Control: no-store");
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
        client.print(",\"mode\":\"");
        client.print(getModeKey());
        client.print("\"");
        client.print(",\"modeLabel\":\"");
        client.print(getModeLabel());
        client.print("\"");
        client.print(",\"canChangeMode\":");
        client.print(canChangeMode() ? "true" : "false");
        client.print(",\"history\":[");

        // 履歴データを送信
        for (int i = 0; i < dataCount; i++) {
          if (i > 0) client.print(",");
          client.print("{\"t\":");
          client.print(dataPoints[i].elapsedSec / 60.0, 1);
          client.print(",\"v\":");
          client.print(dataPoints[i].temp, 1);
          client.print("}");
        }

        client.println("]}");

      } else if (route == "/set-mode") {
        String password = getQueryParam(requestPath, "password");
        String mode = getQueryParam(requestPath, "mode");

        if (password != CONTROL_PASSWORD) {
          client.println("HTTP/1.1 403 Forbidden");
          client.println("Content-Type: application/json");
          client.println("Cache-Control: no-store");
          client.println("Connection: close");
          client.println();
          client.println("{\"ok\":false,\"message\":\"パスワードが違います\"}");
        } else if (!canChangeMode()) {
          client.println("HTTP/1.1 409 Conflict");
          client.println("Content-Type: application/json");
          client.println("Cache-Control: no-store");
          client.println("Connection: close");
          client.println();
          client.println("{\"ok\":false,\"message\":\"このタイミングでは変更できません\"}");
        } else {
          if (mode == "full") {
            processMode = MODE_FULL;
          } else if (mode == "short130") {
            processMode = MODE_SKIP_90_HOLD;
          } else {
            client.println("HTTP/1.1 400 Bad Request");
            client.println("Content-Type: application/json");
            client.println("Cache-Control: no-store");
            client.println("Connection: close");
            client.println();
            client.println("{\"ok\":false,\"message\":\"不正なモードです\"}");
          }

          if (mode == "full" || mode == "short130") {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            client.println("Cache-Control: no-store");
            client.println("Connection: close");
            client.println();
            client.print("{\"ok\":true,\"mode\":\"");
            client.print(getModeKey());
            client.print("\",\"modeLabel\":\"");
            client.print(getModeLabel());
            client.println("\"}");
          }
        }

      } else if (route == "/history.csv") {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/csv; charset=utf-8");
        client.println("Content-Disposition: attachment; filename=\"history.csv\"");
        client.println("Cache-Control: no-store");
        client.println("Connection: close");
        client.println();
        client.println("elapsed_sec,temp_c,target_c,state,duty");
        for (int i = 0; i < dataCount; i++) {
          client.print(dataPoints[i].elapsedSec, 1);
          client.print(",");
          client.print(dataPoints[i].temp, 1);
          client.print(",");
          client.print(dataPoints[i].target, 1);
          client.print(",");
          client.print(dataPoints[i].state);
          client.print(",");
          client.println(dataPoints[i].duty, 3);
        }

      } else {
        // HTMLページ
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Cache-Control: no-store");
        client.println("Connection: close");
        client.println();
        client.println("<!DOCTYPE html>");
        client.println("<html>");
        client.println("<head>");
        client.println("<meta charset='utf-8'>");
        client.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
        client.println("<title>TBTガレージ窯</title>");
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
        client.println(".mode-controls{display:flex;flex-wrap:wrap;gap:12px;margin-top:15px}");
        client.println(".mode-button{border:none;border-radius:999px;padding:12px 18px;background:#2b2b2b;color:#f0f0f0;font-size:1em;cursor:pointer}");
        client.println(".mode-button.active{background:#ff6b35;color:#111;font-weight:bold}");
        client.println(".mode-button:disabled{cursor:not-allowed;opacity:0.45}");
        client.println(".hint{margin-top:12px;color:#999;font-size:0.95em;line-height:1.5}");
        client.println(".toolbar{display:flex;flex-wrap:wrap;gap:12px;margin-bottom:20px}");
        client.println(".download-link{display:inline-flex;align-items:center;justify-content:center;padding:12px 18px;border-radius:999px;background:#1f6feb;color:#fff;text-decoration:none;font-weight:bold}");
        client.println(".chart-container{background:linear-gradient(145deg,#1a1a1a,#0f0f0f);padding:25px;border-radius:15px;box-shadow:0 8px 32px rgba(0,0,0,0.4);border:1px solid #2a2a2a;height:400px;position:relative}");
        client.println("#chart{width:100%;height:100%;display:block}");
        client.println(".update-time{text-align:center;color:#666;margin-top:20px;font-size:0.9em}");
        client.println("</style>");
        client.println("</head>");
        client.println("<body>");
        client.println("<div class='container'>");
        client.println("<h1>TBTガレージ窯</h1>");
        client.println("<div class='toolbar'><a class='download-link' href='/history.csv'>CSVダウンロード</a></div>");
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
        client.println("<div class='card'>");
        client.println("<h3>工程モード</h3>");
        client.println("<div class='value small-value' id='modeLabel'>--</div>");
        client.println("<div class='mode-controls'>");
        client.println("<button class='mode-button' id='modeFull' onclick=\"changeMode('full')\">標準工程</button>");
        client.println("<button class='mode-button' id='modeShort' onclick=\"changeMode('short130')\">130℃工程</button>");
        client.println("</div>");
        client.println("<div class='hint' id='modeHint'>90℃まで昇温中のみ切り替えできます。操作時にパスワードを確認します。</div>");
        client.println("</div>");
        client.println("<div class='chart-container'><canvas id='chart'></canvas></div>");
        client.println("<div class='update-time'>最終更新: <span id='update'>--</span></div>");
        client.println("</div>");
        client.println("<script>");
        client.println("const states=['初期加熱','90℃まで昇温','90℃保持','130℃まで昇温','130℃保持','降温','完了'];");
        client.println("let lastHistory=[];");
        client.println("function drawChart(history){");
        client.println("const canvas=document.getElementById('chart');");
        client.println("const rect=canvas.getBoundingClientRect();");
        client.println("const width=Math.max(320,Math.floor(rect.width));");
        client.println("const height=Math.max(220,Math.floor(rect.height));");
        client.println("const dpr=window.devicePixelRatio||1;");
        client.println("if(canvas.width!==width*dpr||canvas.height!==height*dpr){canvas.width=width*dpr;canvas.height=height*dpr;}");
        client.println("const ctx=canvas.getContext('2d');");
        client.println("ctx.setTransform(dpr,0,0,dpr,0,0);");
        client.println("ctx.clearRect(0,0,width,height);");
        client.println("ctx.fillStyle='#121212';");
        client.println("ctx.fillRect(0,0,width,height);");
        client.println("const pad={left:52,right:18,top:18,bottom:34};");
        client.println("const plotW=width-pad.left-pad.right;");
        client.println("const plotH=height-pad.top-pad.bottom;");
        client.println("ctx.strokeStyle='#2a2a2a';");
        client.println("ctx.lineWidth=1;");
        client.println("for(let i=0;i<=4;i++){");
        client.println("const y=pad.top+(plotH*i/4);");
        client.println("ctx.beginPath();ctx.moveTo(pad.left,y);ctx.lineTo(width-pad.right,y);ctx.stroke();");
        client.println("}");
        client.println("ctx.beginPath();ctx.moveTo(pad.left,pad.top);ctx.lineTo(pad.left,height-pad.bottom);ctx.lineTo(width-pad.right,height-pad.bottom);ctx.stroke();");
        client.println("ctx.fillStyle='#888';");
        client.println("ctx.font='12px sans-serif';");
        client.println("ctx.fillText('時間 (分)',width/2-20,height-8);");
        client.println("ctx.save();ctx.translate(14,height/2+20);ctx.rotate(-Math.PI/2);ctx.fillText('温度 (°C)',0,0);ctx.restore();");
        client.println("if(!history.length){ctx.fillStyle='#666';ctx.fillText('データ待機中',width/2-32,height/2);return;}");
        client.println("let minX=history[0].x;let maxX=history[history.length-1].x;");
        client.println("let minY=history[0].y;let maxY=history[0].y;");
        client.println("for(const p of history){if(p.y<minY)minY=p.y;if(p.y>maxY)maxY=p.y;}");
        client.println("if(maxX-minX<1)maxX=minX+1;");
        client.println("if(maxY-minY<10){const mid=(maxY+minY)/2;minY=mid-5;maxY=mid+5;}");
        client.println("const yPad=Math.max(3,(maxY-minY)*0.08);minY-=yPad;maxY+=yPad;");
        client.println("ctx.fillStyle='#777';");
        client.println("for(let i=0;i<=4;i++){");
        client.println("const yValue=maxY-((maxY-minY)*i/4);");
        client.println("const y=pad.top+(plotH*i/4);");
        client.println("ctx.fillText(yValue.toFixed(0),8,y+4);");
        client.println("}");
        client.println("for(let i=0;i<=4;i++){");
        client.println("const xValue=minX+((maxX-minX)*i/4);");
        client.println("const x=pad.left+(plotW*i/4);");
        client.println("ctx.fillText(xValue.toFixed(0),x-8,height-16);");
        client.println("}");
        client.println("ctx.strokeStyle='#ff6b35';");
        client.println("ctx.lineWidth=3;");
        client.println("ctx.beginPath();");
        client.println("history.forEach((p,index)=>{");
        client.println("const x=pad.left+((p.x-minX)/(maxX-minX))*plotW;");
        client.println("const y=pad.top+((maxY-p.y)/(maxY-minY))*plotH;");
        client.println("if(index===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);");
        client.println("});");
        client.println("ctx.stroke();");
        client.println("}");
        client.println("function updateModeButtons(mode,canChange){");
        client.println("const full=document.getElementById('modeFull');");
        client.println("const shortMode=document.getElementById('modeShort');");
        client.println("full.classList.toggle('active',mode==='full');");
        client.println("shortMode.classList.toggle('active',mode==='short130');");
        client.println("full.disabled=!canChange;");
        client.println("shortMode.disabled=!canChange;");
        client.println("document.getElementById('modeHint').textContent=canChange?'90℃到達前なら標準工程と130℃工程を切り替えできます。':'この工程ではモードは固定されています。';");
        client.println("}");
        client.println("async function changeMode(mode){");
        client.println("const password=window.prompt('操作パスワードを入力してください');");
        client.println("if(password===null)return;");
        client.println("const res=await fetch(`/set-mode?mode=${encodeURIComponent(mode)}&password=${encodeURIComponent(password)}`,{cache:'no-store'});");
        client.println("const data=await res.json();");
        client.println("if(!res.ok){window.alert(data.message||'変更に失敗しました');return;}");
        client.println("await update();");
        client.println("}");
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
        client.println("const res=await fetch('/data',{cache:'no-store'});");
        client.println("const d=await res.json();");
        client.println("document.getElementById('temp').textContent=d.temp.toFixed(1);");
        client.println("document.getElementById('target').textContent=d.target.toFixed(1);");
        client.println("const stateEl=document.getElementById('state');");
        client.println("stateEl.textContent=states[d.state];");
        client.println("stateEl.className='value state-'+d.state;");
        client.println("document.getElementById('time').textContent=d.time.toFixed(0);");
        client.println("document.getElementById('stateEnd').textContent=formatTime(d.stateEnd);");
        client.println("document.getElementById('totalEnd').textContent=formatTime(d.totalEnd);");
        client.println("document.getElementById('modeLabel').textContent=d.modeLabel;");
        client.println("updateModeButtons(d.mode,d.canChangeMode);");
        client.println("document.getElementById('update').textContent=new Date().toLocaleTimeString('ja-JP');");
        client.println("lastHistory=d.history.map(p=>({x:p.t,y:p.v}));");
        client.println("drawChart(lastHistory);");
        client.println("}catch(e){console.error(e)}");
        client.println("}");
        client.println("window.addEventListener('resize',()=>drawChart(lastHistory));");
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
    if (state > 0 && gettime() - lastSaveTime >= 30) {
      if (dataCount < MAX_POINTS) {
        dataPoints[dataCount].elapsedSec = gettime() - startTime;
        dataPoints[dataCount].temp = temp;
        dataPoints[dataCount].target = target;
        dataPoints[dataCount].state = state;
        dataPoints[dataCount].duty = DT;
        dataCount++;
        lastSaveTime = gettime();
      }
    }

    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%s", IP.c_str());
    lcd.print(buf);
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%d,%.0f,%.0f,%d%", state, temp,target, (int)(DT*100));
    lcd.print(buf);

    if (0 < temp && temp < 1000) {
      if (state == 0) {
        state = 1;
        processMode = MODE_FULL;
        target = temp;
        pre_e = 0.0;
        startTime = gettime();
        dataCount = 0;
        lastSaveTime = 0.0;
      } else if (state == 1 && temp_one <= temp) {
        if (processMode == MODE_FULL) {
          state = 2;
          target = 90.0;
          integral = 0.0;
          pre_e = 0.0;
          state2start = gettime();
        } else {
          state = 3;
          target = 90.0;
          pre_e = 0.0;
        }
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

      } else if (state == 5 && target <= 20) {
        state = 6;
        DT = 0;
      }
    }
  }
  heater.update();
}
