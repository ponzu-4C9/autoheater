#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include "linearRegressionSlope.h"
#include <freertos/FreeRTOS.h>

// --- Wi-Fi 設定 (ここを書き換えてください) ---
#define ssid "SRAS2G"
#define EAP_USERNAME "al25138"
#define EAP_PASSWORD "QFgGQfgGQfiH"

// --- ピン定義 ---
const int SSRPin = 2;

const int dataPin = 8; //so
const int clockPin = 10; // sck
const int selectPin = 9; //co

// --- 定数定義 ---
#define target 90         // 90℃
#define t2_to_t1 30 * 60  // 30分
#define target2 130       // 130℃
#define t4_to_t3 90 * 60  // 一時間半

#define roomtemp 20  // 室温

// --- グローバルオブジェクト ---
MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);
WebServer server(80);

// --- 制御用グローバル変数 ---
double DT;
double temp;
int state = 0;
double k;
double t;

// --- Webタスク用グローバル変数 ---
// グラフ表示用の履歴データ
const int HISTORY_MAX = 1000;  // 30秒間隔なら約8時間分
Point webHistory[HISTORY_MAX];
int historyCount = 0;

// 予測計算用の変数
double predictedStateEnd = 0;
double predictedTotalEnd = 0;
unsigned long stateStartTime = 0;
int lastState = -1;


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

  //窯の温度上昇度合を測定
  DT = 1;
  double start = (double)millis() / 1000;
  while ((double)millis() / 1000 - start < 60) {  //60秒まつ
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
      Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);
    }
    pret = t;

    DTcont(DT);
  }
  state++;

  start = (double)millis() / 1000;

  int i = 0;

  while ((double)millis() / 1000 - start < 70) {  //測定
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
      Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);
    }
    pret = t;

    DTcont(DT);

    i++;
  }
  state++;

  DT = (3.0 / 60) / k;


  double timestamp0 = (double)millis() / 1000;  //一分に一回出力見直し用タイムスタンプ
  start = (double)millis() / 1000;

  psclear(ps, N);
  i = 0;

  while (1) {  //target度まで温度上昇
    //温度計読み込み
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
      Serial.printf("%f\ttemp:%f\tDT:%f\n", t, temp, DT);
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
  while (1) {  //降下測定
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if (temp < target - 2 || i >= N) {  //目標温度より2度下がったらbreak
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

  //最大値を迎えた時間より以前をすべて無効点にする
  for (i = 0; i < max_i; i++) {
    ps[i].timestamp = 0.0;
    ps[i].temp = 0.0;
  }

  //dT/dtを調べる
  double dk = linearRegressionSlope(ps, N);

  //ニュートンの冷却法則の式の比例定数を出す
  double l = dk / (target - roomtemp);  // T-T（室温）

  //最適な出力を
  DT = -dk / k;  // T-(T（室温）)

  start = (double)millis() / 1000;
  lcd.clear();
  while ((double)millis() / 1000 - start < t2_to_t1) {  //target度で30分維持
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if ((double)millis() / 1000 - timestamp0 > 180) {
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

  while (1) {  //target2度まで温度上昇
    //温度計読み込み
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
  while ((double)millis() / 1000 - start < t4_to_t3) {  //target2度で90分維持
    //温度計読み込み
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    if ((double)millis() / 1000 - timestamp0 > 80) {
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

  char buf[64];
  lcd.setCursor(0, 0);
  snprintf(buf, sizeof(buf), "Finished!");
  lcd.print(buf);

  // 修正：タスク終了時に無限ループで停止させる (クラッシュ防止)
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
// HTMLを配信するハンドラ
void handleRoot() {
  // 生文字列リテラルでHTMLを定義
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
<div class='card'><h3>現在状態終了予想時間</h3><div class='value small-value' id='stateEnd'>--</div></div>
<div class='card'><h3>全工程終了予想時間</h3><div class='value small-value' id='totalEnd'>--</div></div>
</div>
<div class='chart-container'><canvas id='chart'></canvas></div>
<div class='update-time'>最終更新: <span id='update'>--</span></div>
</div>
<script>
const ctx=document.getElementById('chart').getContext('2d');
const chart=new Chart(ctx,{type:'line',data:{datasets:[{label:'温度',data:[],borderColor:'#ff6b35',backgroundColor:'rgba(255,107,53,0.1)',tension:0.4,borderWidth:3,pointRadius:0}]},options:{responsive:true,maintainAspectRatio:false,scales:{x:{type:'linear',title:{display:true,text:'時間 (分)',color:'#888'},ticks:{color:'#666'},grid:{color:'#2a2a2a'}},y:{title:{display:true,text:'温度 (°C)',color:'#888'},ticks:{color:'#666'},grid:{color:'#2a2a2a'}}},plugins:{legend:{labels:{color:'#e0e0e0'}}}}});
const states=['初期加熱','k測定','昇温','l測定','維持','昇温',''維持','完了'];
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
document.getElementById('stateEnd').textContent=formatTime(d.stateEnd);
document.getElementById('totalEnd').textContent=formatTime(d.totalEnd);
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

// JSONデータを配信するハンドラ
void handleData() {
  String json = "{";
  json += "\"temp\":" + String(temp, 2) + ",";

  double currentTarget = 0;
  if (state <= 4) currentTarget = target;
  else currentTarget = target2;

  json += "\"target\":" + String(currentTarget) + ",";
  json += "\"state\":" + String(state) + ",";

  double elapsedMin = (millis() / 1000.0) / 60.0;
  json += "\"time\":" + String(elapsedMin, 1) + ",";

  json += "\"stateEnd\":" + String(predictedStateEnd, 0) + ",";
  json += "\"totalEnd\":" + String(predictedTotalEnd, 0) + ",";

  // グラフ用履歴データ (メモリ節約のため間引きや制限が必要だが、ここではそのまま送る)
  json += "\"history\":[";
  for (int i = 0; i < historyCount; i++) {
    json += "{\"t\":" + String(webHistory[i].timestamp / 60.0, 2) + ",\"v\":" + String(webHistory[i].temp, 1) + "}";
    if (i < historyCount - 1) json += ",";
  }
  json += "]";

  json += "}";

  server.send(200, "application/json", json);
}

void webTask(void *pvParameters) {
  // Wi-Fi 接続
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Webサーバー設定
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  unsigned long lastRecordTime = 0;

  // 状態監視用
  lastState = state;
  stateStartTime = millis() / 1000;

  while (1) {
    server.handleClient();

    unsigned long currentSec = millis() / 1000;

    // 状態変化検知 (開始時間をリセット)
    if (state != lastState) {
      stateStartTime = currentSec;
      lastState = state;
    }

    // 30秒に1回 履歴保存 & 予測計算
    if (currentSec - lastRecordTime >= 30) {
      lastRecordTime = currentSec;

      // Point構造体に保存
      if (historyCount < HISTORY_MAX) {
        webHistory[historyCount].temp = temp;
        webHistory[historyCount].timestamp = (double)currentSec;  // 絶対時間
        historyCount++;
      }

      // --- 予測ロジック ---
      // linearRegressionSlopeを使って、直近の履歴から傾きを算出
      // ControlTaskの変数には触れないので、WebTask独自のPoint配列を使う

      double elapsedInState = currentSec - stateStartTime;
      double slope = 0;

      // 直近データの抽出 (例えば過去5点)
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

      // 傾き算出 (deg/sec)
      slope = linearRegressionSlope(recentPoints, sampleCount);

      // 残り時間計算
      predictedStateEnd = -1;  // 計算不能時
      predictedTotalEnd = -1;

      // 状態ごとの予測
      double timeRemState = 0;

      if (state == 2) {  // 90度まで昇温
        if (slope > 0.001) {
          timeRemState = (target - temp) / slope;
          predictedStateEnd = timeRemState;
        }
      } else if (state == 3) {  // 90度保持
        double duration = t2_to_t1;
        if (elapsedInState < duration) {
          timeRemState = duration - elapsedInState;
          predictedStateEnd = timeRemState;
        } else {
          predictedStateEnd = 0;
        }
      } else if (state == 5) {  // 130度まで昇温
        if (slope > 0.001) {
          timeRemState = (target2 - temp) / slope;
          predictedStateEnd = timeRemState;
        }
      } else if (state == 6) {  // 130度保持
        double duration = t4_to_t3;
        if (elapsedInState < duration) {
          timeRemState = duration - elapsedInState;
          predictedStateEnd = timeRemState;
        } else {
          predictedStateEnd = 0;
        }
      }

      // 全工程終了予想 (簡易計算: 現在の残り + 今後の工程の定義時間)
      // ※ kなどが不明なため、昇温時間は理想値(3度/分 = 0.05度/秒)で仮定するしかない
      if (predictedStateEnd != -1) {
        double futureTime = 0;
        if (state < 3) futureTime += t2_to_t1;                   // 90度保持
        if (state < 5) futureTime += (target2 - target) / 0.05;  // 90->130昇温(仮)
        if (state < 6) futureTime += t4_to_t3;                   // 130度保持

        predictedTotalEnd = predictedStateEnd + futureTime;
      }
    }

    // ControlTaskが1秒ブロックするので、WebTaskは隙間で動く必要がある。
    // 頻繁にYieldしてチャンスを伺う。
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(SSRPin, OUTPUT);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();

  xTaskCreatePinnedToCore(
    ControlTask,  // 実行する関数
    "Control",    // タスク名
    10000,        // スタックサイズ (必要に応じて調整)
    NULL,         // パラメータ
    2,            // 優先度
    NULL,         // タスクハンドル
    0             // コア (ESP32C3は実質0)
  );

  xTaskCreatePinnedToCore(
    webTask,
    "Communication",
    10000,
    NULL,
    1,
    NULL,
    0);
}

void loop() {
}