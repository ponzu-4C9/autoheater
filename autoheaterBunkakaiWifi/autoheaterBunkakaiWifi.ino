#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include "linearRegressionSlope.h"
#include <freertos/FreeRTOS.h>

// --- Wi-Fi 設定 ---
#define ssid "SRAS2G"
#define EAP_USERNAME "al25138"
#define EAP_PASSWORD "QFgGQfgGQfiH"

const int SSRPin = 2;
const int dataPin = 8;    // so
const int clockPin = 10;  // sck
const int selectPin = 9;  // co

// --- 定数定義 ---
const int target =90;       // 90℃
const int t2_to_t1 =30 * 60;  // 30分
const int target2 =130;       // 130℃
const int t4_to_t3 =90 * 60;  // 一時間半
const int roomtemp =20;       // 室温

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);
WebServer server(80);

volatile double DT;
volatile double temp;
volatile int state = 0;
volatile double k;
volatile double l;
volatile double t;


const int HISTORY_MAX = 1000;
Point webHistory[HISTORY_MAX];
int historyCount = 0;

volatile double predictedStateEnd = 0;
volatile double predictedTotalEnd = 0;
volatile unsigned long stateStartTime = 0;
volatile int lastState = -1;

void DTcont(double duty) {
  if (isnan(duty)) duty = 0;
  if (duty < 0) duty = 0;
  if (duty > 1) duty = 1;
  digitalWrite(SSRPin, HIGH);
  vTaskDelay((int)(1000 * duty));
  digitalWrite(SSRPin, LOW);
  vTaskDelay((int)(1000 * (1 - duty)));
}

double pret = 0;

void ControlTask(void *pvParameters) {
  const int N = 2000;
  static Point ps[N] = { 0 };

  DT = 1;
  double start = (double)millis() / 1000;
  while ((double)millis() / 1000 - start < 60) {//初期上昇
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%.2f", (double)millis() / 1000 - start);
    lcd.print(buf);

    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    t = (double)millis() / 1000;
    if (t != pret) {
      //Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);
    }
    pret = t;

    DTcont(DT);
  }
  state++;

  start = (double)millis() / 1000;
  int i = 0;

  while ((double)millis() / 1000 - start < 70) {//k測定
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    ps[i].temp = (double)temp;
    ps[i].timestamp = (double)millis() / 1000 - start;

    k = linearRegressionSlope(ps, N) / DT;

    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%.2f|%f", (double)millis() / 1000 - start, k);
    lcd.print(buf);

    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.4f", temp, DT);
    lcd.print(buf);

    t = (double)millis() / 1000;
    if (t != pret) {
      //Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);
    }
    pret = t;

    DTcont(DT);
    i++;
  }
  state++;

  DT = (3.0 / 60) / k;

  double timestamp0 = (double)millis() / 1000;
  start = (double)millis() / 1000;
  psclear(ps, N);
  i = 0;

  while (1) {//昇温
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if (10 < temp && temp < 500) {
      if (temp > target) {
        break;
      }
    }

    if (i < N) {
      ps[i].temp = (double)temp;
      ps[i].timestamp = (double)millis() / 1000 - start;
      i++;
    }

    if ((double)millis() / 1000 - timestamp0 > 60) {
      if (linearRegressionSlope(ps, N) > (3.0 / 60)) {
        DT -= 0.01;
      } else {
        DT += 0.005;
      }
      psclear(ps, N);
      start = (double)millis() / 1000;
      i = 0;
      timestamp0 = (double)millis() / 1000;
    }

    double t = (double)millis() / 1000;
    if (t != pret) {
      //Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);
    }
    pret = t;

    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%f", k);
    lcd.print(buf);

    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  DT = 0;
  psclear(ps, N);

  double max_temp = 0;
  int max_i = 0;
  start = (double)millis() / 1000;
  i = 0;
  while (1) {//l測定
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if (temp < target - 2 || i >= N) {
      break;
    }

    if (temp > max_temp) {
      max_temp = temp;
      max_i = i;
    }

    ps[i].timestamp = (double)millis() / 1000 - start;
    ps[i].temp = temp;
    i++;

    t = (double)millis() / 1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);

    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "....");
    lcd.print(buf);

    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  for (i = 0; i < max_i; i++) {
    ps[i].timestamp = 0.0;
    ps[i].temp = 0.0;
  }

  double dk = linearRegressionSlope(ps, N);
  l = dk / (target - roomtemp);
  DT = -dk / k;

  start = (double)millis() / 1000;
  timestamp0 = start;
  lcd.clear();
  while ((double)millis() / 1000 - start < t2_to_t1) {//第一維持
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if ((double)millis() / 1000 - timestamp0 > 70) {
      if (temp > target) {
        DT -= 0.01;
      } else if (temp < target) {
        DT += 0.01;
      }
      timestamp0 = (double)millis() / 1000;
    }

    t = (double)millis() / 1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);

    char buf[64];
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  DT = ((3.0 / 60) - l * (temp - roomtemp)) / k;

  psclear(ps, N);
  start = (double)millis() / 1000;
  i = 0;

  while (1) {//第二昇温
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if (10 < temp && temp < 500) {
      if (temp > target2) {
        break;
      }
    }
    if (i < N) {
      ps[i].temp = (double)temp;
      ps[i].timestamp = (double)millis() / 1000 - start;
      i++;
    }

    if ((double)millis() / 1000 - timestamp0 > 60) {
      if (linearRegressionSlope(ps, N) > (3.0 / 60)) {
        DT -= 0.01;
      } else {
        DT += 0.01;
      }
      psclear(ps, N);
      start = (double)millis() / 1000;
      i = 0;
      timestamp0 = (double)millis() / 1000;
    }

    t = (double)millis() / 1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);

    char buf[64];
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  DT = (-l / k) * (target2 - roomtemp);

  start = (double)millis() / 1000;
  timestamp0 = start;
  lcd.clear();
  while ((double)millis() / 1000 - start < t4_to_t3) {//第二維持
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if ((double)millis() / 1000 - timestamp0 > 70) {
      if (temp > target2) {
        DT -= 0.01;
      } else if (temp < target2) {
        DT += 0.01;
      }
      timestamp0 = (double)millis() / 1000;
    }

    t = (double)millis() / 1000;
    Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);

    char buf[64];
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%.2f|%.2f", temp, DT);
    lcd.print(buf);

    DTcont(DT);
  }
  state++;

  char buf2[64];
  lcd.setCursor(0, 0);
  snprintf(buf2, sizeof(buf2), "Finished!");
  lcd.print(buf2);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void handleRoot() {
  const char *html = R"raw(
HTTP/1.1 200 OK
Content-Type: text/html
Connection: close

<!DOCTYPE html>
<html>
<head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>TBT文化会窯</title>
<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>
<style>
/* ... (CSSは元のまま) ... */
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',Arial,sans-serif;background:#0a0a0a;color:#e0e0e0;padding:20px}
.container{max-width:1200px;margin:0 auto}
h1{font-size:2.5em;margin-bottom:30px;color:#ff6b35;text-align:center;text-shadow:0 0 20px rgba(255,107,53,0.5)}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:20px;margin-bottom:30px}
.card{background:linear-gradient(145deg,#1a1a1a,#0f0f0f);padding:25px;border-radius:15px;box-shadow:0 8px 32px rgba(0,0,0,0.4);border:1px solid #2a2a2a}
.card h3{font-size:0.9em;color:#888;margin-bottom:10px;text-transform:uppercase;letter-spacing:1px}
.card .value{font-size:2.5em;font-weight:bold;color:#ff6b35}
.card .unit{font-size:0.8em;color:#666;margin-left:5px}
.card .small-value{font-size:1.5em}
.state-0{color:#4ecdc4}
.state-1{color:#ffd93d}
.state-2{color:#ff6b35}
.state-3{color:#a8e6cf}
.state-4{color:#ff8b94}
.state-5{color:#95e1d3}
.chart-container{background:linear-gradient(145deg,#1a1a1a,#0f0f0f);padding:25px;border-radius:15px;box-shadow:0 8px 32px rgba(0,0,0,0.4);border:1px solid #2a2a2a;height:400px;position:relative}
.update-time{text-align:center;color:#666;margin-top:20px;font-size:0.9em}
</style>
</head>
<body>
<div class='container'>
<h1>TBT文化会窯</h1>
<div class='stats'>
<div class='card'><h3>現在温度</h3><div class='value' id='temp'>--</div><span class='unit'>°C</span></div>
<div class='card'><h3>目標温度</h3><div class='value' id='target'>--</div><span class='unit'>°C</span></div>
<div class='card'><h3>状態</h3><div class='value' id='state'>--</div></div>
<div class='card'><h3>経過時間</h3><div class='value' id='time'>--</div><span class='unit'>分</span></div>
</div>
<div class='stats'>
<div class='card'><h3>熱効率 K</h3><div class='value small-value' id='k_val'>--</div><span class='unit'>°C/sec / DT</span></div>
<div class='card'><h3>放熱係数 L</h3><div class='value small-value' id='l_val'>--</div><span class='unit'>/sec/°C</span></div>
<div class='card'><h3>デューティ比 DT</h3><div class='value small-value' id='dt_val'>--</div></div>
<div class='card'><h3>状態終了予想 (秒)</h3><div class='value small-value' id='stateEnd'>--</div></div>
</div>
<div class='stats'>
<div class='card'><h3>全工程終了予想 (秒)</h3><div class='value small-value' id='totalEnd'>--</div></div>
</div>
<div class='chart-container'><canvas id='chart'></canvas></div>
<div class='update-time'>最終更新: <span id='update'>--</span></div>
</div>
<script>
const ctx=document.getElementById('chart').getContext('2d');
const chart=new Chart(ctx,{type:'line',data:{datasets:[{label:'温度',data:[],borderColor:'#ff6b35',backgroundColor:'rgba(255,107,53,0.1)',tension:0.4,borderWidth:3,pointRadius:0}]},options:{responsive:true,maintainAspectRatio:false,scales:{x:{type:'linear',title:{display:true,text:'時間 (分)',color:'#888'},ticks:{color:'#666'},grid:{color:'#2a2a2a'}},y:{title:{display:true,text:'温度 (°C)',color:'#888'},ticks:{color:'#666'},grid:{color:'#2a2a2a'}}},plugins:{legend:{labels:{color:'#e0e0e0'}}}}});
const states=['初期加熱','k測定','昇温(T1)','降下測定(L)','維持(T1)','昇温(T2)','維持(T2)','完了'];
function formatTime(sec){
if(sec<0)return '計算中...';
const h=Math.floor(sec/3600);
const m=Math.floor((sec%3600)/60);
const now=new Date();
const end=new Date(now.getTime()+sec*1000);
return `${h}h${m}m (${end.getHours()}:${String(end.getMinutes()).padStart(2,'0')})`;
}
async function update(){
try{
const res=await fetch('/data');
const d=await res.json();
document.getElementById('temp').textContent=d.temp.toFixed(1);
document.getElementById('target').textContent=d.target.toFixed(1);
const stateEl=document.getElementById('state');
stateEl.textContent=states[d.state] || '不明';
stateEl.className='value state-'+d.state;
document.getElementById('time').textContent=d.time.toFixed(0);

document.getElementById('k_val').textContent=d.k_val.toFixed(4);
document.getElementById('l_val').textContent=d.l_val.toFixed(6);
document.getElementById('dt_val').textContent=d.DT_val.toFixed(3);

document.getElementById('stateEnd').textContent=d.stateEnd;
document.getElementById('totalEnd').textContent=d.totalEnd;

document.getElementById('update').textContent=new Date().toLocaleTimeString('ja-JP');
chart.data.datasets[0].data=d.history.map(p=>({x:p.t,y:p.v}));
chart.update('none');
}catch(e){console.error(e)}
}
update();setInterval(update,2000);
</script>
</body>
</html>
)raw";
  server.sendContent(html);
}

void handleData() {
  
  double currentTemp = temp;
  double currentDT = DT;
  double currentK = k;
  double currentL = l;
  int currentState = state;
  double currentPredictedStateEnd = predictedStateEnd;
  double currentPredictedTotalEnd = predictedTotalEnd;
  double elapsedMin = 0;
  
  double currentTarget;

  String json = "{";

  if (isnan(currentTemp)) json += "\"temp\":0.00,";
  else json += "\"temp\":" + String(currentTemp, 2) + ",";

  if (currentState <= 4) currentTarget = target;
  else currentTarget = target2;

  json += "\"target\":" + String(currentTarget) + ",";
  json += "\"state\":" + String(currentState) + ",";

  elapsedMin = (millis() / 1000.0) / 60.0;
  json += "\"time\":" + String(elapsedMin, 1) + ",";

  json += "\"k_val\":" + String(currentK, 6) + ",";
  json += "\"l_val\":" + String(currentL, 6) + ",";
  json += "\"DT_val\":" + String(currentDT, 3) + ",";

  json += "\"stateEnd\":" + String(currentPredictedStateEnd, 0) + ",";
  json += "\"totalEnd\":" + String(currentPredictedTotalEnd, 0) + ",";

  json += "\"history\":[";
  for (int i = 0; i < historyCount; i++) {
    json += "{\"t\":" + String(webHistory[i].timestamp / 60.0, 2) + ",\"v\":" + String(webHistory[i].temp, 1) + "}";
    if (i < historyCount - 1) json += ",";
  }
  json += "]";

  json += "}";

  Serial.println("--- Sending JSON ---");
  Serial.println(json);
  Serial.println("--------------------");

  server.send(200, "application/json", json);
}

void webTask(void *pvParameters) {
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  unsigned long lastRecordTime = 0;
  
  
  lastState = state;
  
  stateStartTime = millis() / 1000;

  while (1) {
    server.handleClient();

    unsigned long currentSec = millis() / 1000;

    
    double currentTemp = temp;
    int currentState = state;

    if (currentState != lastState) {
      stateStartTime = currentSec;
      lastState = currentState;
    }

    if (currentSec - lastRecordTime >= 30) {
      lastRecordTime = currentSec;

      if (historyCount < HISTORY_MAX) {
        webHistory[historyCount].temp = currentTemp;
        webHistory[historyCount].timestamp = (double)currentSec;
        historyCount++;
      }

      double elapsedInState = currentSec - stateStartTime;
      double slope = 0;

      int sampleCount = 0;
      const int SAMPLE_SIZE = 10;
      Point recentPoints[SAMPLE_SIZE];

      for (int i = 0; i < SAMPLE_SIZE; i++) {
        int idx = historyCount - 1 - i;
        if (idx >= 0) {
          recentPoints[i] = webHistory[idx];
          recentPoints[i].timestamp -= (double)stateStartTime;
          sampleCount++;
        } else {
          recentPoints[i].timestamp = 0;
          recentPoints[i].temp = 0;
        }
      }

      slope = linearRegressionSlope(recentPoints, sampleCount);

      double newPredictedStateEnd = -1;
      double newPredictedTotalEnd = -1;
      double timeRemState = 0;

      if (currentState == 2) {
        if (slope > 0.001) {
          timeRemState = (target - currentTemp) / slope;
          newPredictedStateEnd = timeRemState;
        }
      } else if (currentState == 4) {
        double duration = t2_to_t1;
        if (elapsedInState < duration) {
          timeRemState = duration - elapsedInState;
          newPredictedStateEnd = timeRemState;
        } else {
          newPredictedStateEnd = 0;
        }
      } else if (currentState == 5) {
        if (slope > 0.001) {
          timeRemState = (target2 - currentTemp) / slope;
          newPredictedStateEnd = timeRemState;
        }
      } else if (currentState == 6) {
        double duration = t4_to_t3;
        if (elapsedInState < duration) {
          timeRemState = duration - elapsedInState;
          newPredictedStateEnd = timeRemState;
        } else {
          newPredictedStateEnd = 0;
        }
      }

      if (newPredictedStateEnd != -1) {
        double futureTime = 0;
        if (currentState <= 3) futureTime += t2_to_t1;
        if (currentState <= 4) futureTime += (target2 - target) / 0.05;
        if (currentState <= 5) futureTime += t4_to_t3;
        newPredictedTotalEnd = newPredictedStateEnd + futureTime;
      }

      
      predictedStateEnd = newPredictedStateEnd;
      predictedTotalEnd = newPredictedTotalEnd;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);

  
  DT = 0;
  temp = roomtemp;
  k = 0;
  l = 0;
  state = 0;

  pinMode(SSRPin, OUTPUT);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();

  xTaskCreatePinnedToCore(
    ControlTask,
    "Control",
    10000,
    NULL,
    2,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    webTask,
    "Communication",
    10000,
    NULL,
    1,
    NULL,
    0
  );
}

void loop() {
}