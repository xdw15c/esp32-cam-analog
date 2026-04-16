#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <SD_MMC.h>
#include <ctype.h>
#include <math.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "secrets.h"

// AI Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

static const char *kPhotoPath = "/latest.jpg";
static const char *kConfigPath = "/config/config.json";
static const uint8_t kGaugeCount = 2;

struct GaugeConfig {
  int id = 0;
  String name;
  int cx = 0;
  int cy = 0;
  int radius = 0;
  float angleMin = 0.0f;
  float angleMax = 0.0f;
  float valueMin = 0.0f;
  float valueMax = 0.0f;
  String unit;
  bool valid = false;
};

struct DeviceConfig {
  String deviceId = "esp32cam-01";
  int intervalS = 2;
  GaugeConfig gauges[kGaugeCount];
  bool loaded = false;
};

struct GaugeReading {
  bool detected = false;
  float angleDeg = 0.0f;
  float value = 0.0f;
  float confidence = 0.0f;
  float darkness = 255.0f;
};

WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>ESP32-CAM Manometry</title>
<style>
*{box-sizing:border-box}
body{font-family:sans-serif;margin:0;background:#f1f5f9;color:#1e293b}
.top{background:#0f172a;color:#f8fafc;padding:.75rem 1rem;display:flex;align-items:center;gap:1rem;flex-wrap:wrap}
.top h1{font-size:1.05rem;margin:0;flex:1}
nav button{background:transparent;border:0;color:#94a3b8;padding:.5rem 1rem;cursor:pointer;font-size:.9rem;border-bottom:2px solid transparent}
nav button.active{color:#f8fafc;border-bottom-color:#3b82f6}
.page{display:none;padding:1rem;max-width:1200px;margin:0 auto}
.page.active{display:block}
.card{background:#fff;border-radius:10px;padding:1rem;box-shadow:0 2px 12px #0001;margin-bottom:1rem}
.row{display:flex;gap:.5rem;flex-wrap:wrap;margin-bottom:.75rem}
button{border:0;border-radius:7px;padding:.5rem .85rem;cursor:pointer;background:#3b82f6;color:#fff;font-size:.875rem}
button.sec{background:#64748b}
button.grn{background:#16a34a}
button.mode-btn{background:#f1f5f9;color:#1e293b;border:1px solid #cbd5e1;width:100%;margin-top:.5rem;text-align:left}
button.mode-btn.active-mode{background:#fef9c3;border-color:#eab308;color:#713f12}
canvas{max-width:100%;border:1px solid #e2e8f0;border-radius:8px;display:block}
.hint{color:#64748b;font-size:.85rem;margin:.2rem 0 .6rem}
.gg{display:grid;grid-template-columns:1fr 1fr;gap:1rem}
@media(max-width:640px){.gg{grid-template-columns:1fr}}
.gc{border:2px solid #e2e8f0;border-radius:10px;padding:.75rem}
.gc.g1{border-color:#e11d48}.gc.g2{border-color:#2563eb}
.gc h3{margin:0 0 .5rem;font-size:.95rem}
label{display:block;font-size:.8rem;color:#475569;margin:.45rem 0 .1rem}
input[type=number],input[type=text]{width:100%;border:1px solid #cbd5e1;border-radius:6px;padding:.3rem .45rem;font-size:.875rem}
input:focus{outline:2px solid #3b82f6;border-color:transparent}
pre{background:#0f172a;color:#e2e8f0;padding:.75rem;border-radius:8px;overflow:auto;font-size:.78rem}
.imginfo{color:#64748b;font-size:.8rem;margin:.3rem 0 0}
.analysis{margin-top:1rem}
</style>
</head><body>

<div class="top">
  <h1>&#128247; ESP32-CAM Manometry</h1>
  <nav>
    <button class="active" onclick="tab('cam',this)">Kamera</button>
    <button onclick="tab('roi',this)">Kalibracja ROI</button>
  </nav>
</div>

<div id="tab-cam" class="page active">
  <div class="card">
    <div class="row">
      <button onclick="capture()">&#128247; Zrob zdjecie</button>
      <button class="grn" onclick="analyzeCapture()">&#129504; Analizuj obraz</button>
      <button class="sec" onclick="reloadCam()">&#8635; Odswiez</button>
    </div>
    <canvas id="canvas"></canvas>
    <p id="img-info" class="imginfo"></p>
    <div class="analysis">
      <h3 style="margin:0 0 .5rem">Wynik analizy</h3>
      <pre id="analysis-pre">Brak analizy</pre>
    </div>
  </div>
</div>

<div id="tab-roi" class="page">
  <div class="card">
    <p class="hint">Kliknij przycisk trybu przy manometrze, a nastepnie kliknij na obrazie, by ustawic srodek wskazowki.</p>
    <div class="row">
      <button onclick="captureAndSwitch()">&#128247; Zrob nowe zdjecie</button>
      <button class="grn" onclick="saveConfig()">&#128190; Zapisz config na SD</button>
      <button class="sec" onclick="loadConfig()">&#128194; Wczytaj z SD</button>
    </div>
    <canvas id="canvas-roi"></canvas>
    <p id="roi-info" class="imginfo"></p>
  </div>
  <div class="gg">
    <div class="gc g1">
      <h3 style="color:#e11d48">&#9899; Manometr 1</h3>
      <button class="mode-btn" id="mb1" onclick="setMode(1)">&#127919; Ustaw srodek M1 (kliknij na obrazie)</button>
      <label>Nazwa</label><input type="text" id="g1-name" value="Manometr 1">
      <label>Srodek X (px)</label><input type="number" id="g1-cx" value="160" oninput="redraw()">
      <label>Srodek Y (px)</label><input type="number" id="g1-cy" value="120" oninput="redraw()">
      <label>Promien wskazowki (px)</label><input type="number" id="g1-r" value="80" oninput="redraw()">
      <label>Kat MIN (deg, od gory, CW) np -135 = godz 7</label><input type="number" id="g1-amin" value="-135" oninput="redraw()">
      <label>Kat MAX (deg, od gory, CW) np 135 = godz 5</label><input type="number" id="g1-amax" value="135" oninput="redraw()">
      <label>Wartosc MIN</label><input type="number" id="g1-vmin" value="0">
      <label>Wartosc MAX</label><input type="number" id="g1-vmax" value="10">
      <label>Jednostka</label><input type="text" id="g1-unit" value="bar">
    </div>
    <div class="gc g2">
      <h3 style="color:#2563eb">&#9899; Manometr 2</h3>
      <button class="mode-btn" id="mb2" onclick="setMode(2)">&#127919; Ustaw srodek M2 (kliknij na obrazie)</button>
      <label>Nazwa</label><input type="text" id="g2-name" value="Manometr 2">
      <label>Srodek X (px)</label><input type="number" id="g2-cx" value="480" oninput="redraw()">
      <label>Srodek Y (px)</label><input type="number" id="g2-cy" value="120" oninput="redraw()">
      <label>Promien wskazowki (px)</label><input type="number" id="g2-r" value="80" oninput="redraw()">
      <label>Kat MIN (deg, od gory, CW)</label><input type="number" id="g2-amin" value="-135" oninput="redraw()">
      <label>Kat MAX (deg, od gory, CW)</label><input type="number" id="g2-amax" value="135" oninput="redraw()">
      <label>Wartosc MIN</label><input type="number" id="g2-vmin" value="0">
      <label>Wartosc MAX</label><input type="number" id="g2-vmax" value="10">
      <label>Jednostka</label><input type="text" id="g2-unit" value="bar">
    </div>
  </div>
  <div class="card" style="margin-top:1rem">
    <h3 style="margin:0 0 .5rem">config.json (podglad)</h3>
    <pre id="cfg-pre">{}</pre>
  </div>
</div>

<script>
const COLORS=['','#e11d48','#2563eb'];
let mode=0;
const canvas=document.getElementById('canvas');
const ctx=canvas.getContext('2d');
const cRoi=document.getElementById('canvas-roi');
const rctx=cRoi.getContext('2d');
let roiImg=null;

function tab(id,btn){
  document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('nav button').forEach(b=>b.classList.remove('active'));
  document.getElementById('tab-'+id).classList.add('active');
  btn.classList.add('active');
  if(id==='roi'){loadRoiImg();updatePreview();}
}

function loadCamImg(){
  const info=document.getElementById('img-info');
  info.textContent='Ladowanie...';
  const i=new Image();
  i.onload=()=>{canvas.width=i.width;canvas.height=i.height;ctx.drawImage(i,0,0);info.textContent=i.width+'x'+i.height+' px';};
  i.src='/photo.jpg?t='+Date.now();
}
function reloadCam(){loadCamImg();}

function loadRoiImg(){
  const info=document.getElementById('roi-info');
  info.textContent='Ladowanie...';
  const i=new Image();
  i.onload=()=>{cRoi.width=i.width;cRoi.height=i.height;roiImg=i;redraw();info.textContent=i.width+'x'+i.height+' px';};
  i.src='/photo.jpg?t='+Date.now();
}

async function capture(){
  const r=await fetch('/capture',{method:'POST'});
  if(!r.ok){alert('Blad capture');return;}
  loadCamImg();
}
async function captureAndSwitch(){
  const r=await fetch('/capture',{method:'POST'});
  if(!r.ok){alert('Blad capture');return;}
  loadRoiImg();
}

async function analyzeCapture(){
  const out=document.getElementById('analysis-pre');
  out.textContent='Analiza w toku...';
  const r=await fetch('/analyze',{method:'POST'});
  const text=await r.text();
  if(!r.ok){out.textContent=text||'Blad analizy';return;}
  try{out.textContent=JSON.stringify(JSON.parse(text),null,2);}catch{out.textContent=text;}
  loadCamImg();
}

function gv(id){return +document.getElementById(id).value||0;}
function d2r(d){return(d-90)*Math.PI/180;}
function getG(n){
  const p='g'+n;
  return{cx:gv(p+'-cx'),cy:gv(p+'-cy'),r:gv(p+'-r'),amin:gv(p+'-amin'),amax:gv(p+'-amax')};
}

function redraw(){
  if(!roiImg)return;
  rctx.clearRect(0,0,cRoi.width,cRoi.height);
  rctx.drawImage(roiImg,0,0,cRoi.width,cRoi.height);
  [1,2].forEach(n=>{
    const g=getG(n),col=COLORS[n];
    const aMin=d2r(g.amin),aMax=d2r(g.amax);
    rctx.strokeStyle=col;rctx.lineWidth=3;
    rctx.beginPath();rctx.arc(g.cx,g.cy,g.r,aMin,aMax);rctx.stroke();
    rctx.lineWidth=2;
    [[aMin,'MIN'],[aMax,'MAX']].forEach(([a,lbl])=>{
      const ex=g.cx+g.r*Math.cos(a),ey=g.cy+g.r*Math.sin(a);
      rctx.strokeStyle=col;rctx.beginPath();rctx.moveTo(g.cx,g.cy);rctx.lineTo(ex,ey);rctx.stroke();
      rctx.fillStyle=col;rctx.font='bold 11px sans-serif';rctx.fillText(lbl,ex+4,ey+4);
    });
    rctx.strokeStyle=col;rctx.lineWidth=2;
    [[-12,0,12,0],[0,-12,0,12]].forEach(([x1,y1,x2,y2])=>{
      rctx.beginPath();rctx.moveTo(g.cx+x1,g.cy+y1);rctx.lineTo(g.cx+x2,g.cy+y2);rctx.stroke();
    });
    rctx.fillStyle=col;rctx.font='bold 13px sans-serif';
    rctx.fillText('M'+n,g.cx+6,g.cy-g.r-7);
  });
  updatePreview();
}

function setMode(n){
  mode=(mode===n)?0:n;
  [1,2].forEach(i=>document.getElementById('mb'+i).classList.toggle('active-mode',mode===i));
  cRoi.style.cursor=mode?'crosshair':'default';
}

cRoi.addEventListener('click',ev=>{
  if(!mode)return;
  const rect=cRoi.getBoundingClientRect();
  const x=Math.round((ev.clientX-rect.left)*cRoi.width/rect.width);
  const y=Math.round((ev.clientY-rect.top)*cRoi.height/rect.height);
  document.getElementById('g'+mode+'-cx').value=x;
  document.getElementById('g'+mode+'-cy').value=y;
  setMode(0);redraw();
});

function buildConfig(){
  return{
    device_id:'esp32cam-01',interval_s:2,
    gauges:[1,2].map(n=>{
      const p='g'+n;
      return{id:n,name:document.getElementById(p+'-name').value,
        cx:gv(p+'-cx'),cy:gv(p+'-cy'),radius:gv(p+'-r'),
        angle_min:gv(p+'-amin'),angle_max:gv(p+'-amax'),
        value_min:gv(p+'-vmin'),value_max:gv(p+'-vmax'),
        unit:document.getElementById(p+'-unit').value};
    })
  };
}

function applyConfig(cfg){
  if(!cfg||!Array.isArray(cfg.gauges))return;
  cfg.gauges.forEach(g=>{
    const n=g.id,p='g'+n;
    const s=(id,v)=>{const el=document.getElementById(id);if(el&&v!==undefined)el.value=v;};
    s(p+'-name',g.name);s(p+'-cx',g.cx);s(p+'-cy',g.cy);s(p+'-r',g.radius);
    s(p+'-amin',g.angle_min);s(p+'-amax',g.angle_max);
    s(p+'-vmin',g.value_min);s(p+'-vmax',g.value_max);s(p+'-unit',g.unit);
  });
  redraw();
}

function updatePreview(){
  document.getElementById('cfg-pre').textContent=JSON.stringify(buildConfig(),null,2);
}

async function saveConfig(){
  const r=await fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(buildConfig())});
  if(!r.ok){alert('Blad zapisu konfiguracji');return;}
  alert('Konfiguracja zapisana na SD (/config/config.json)');
}

async function loadConfig(){
  const r=await fetch('/config');
  if(!r.ok)return;
  applyConfig(await r.json());
}

loadCamImg();
loadConfig();
</script>
</body></html>
)HTML";

bool checkAuth() {
  if (server.authenticate(WEB_USERNAME, WEB_PASSWORD)) return true;
  server.requestAuthentication();
  return false;
}

String sdReadText(const char *path) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return String();

  String text;
  text.reserve(f.size() + 1);
  while (f.available()) {
    text += static_cast<char>(f.read());
  }
  f.close();
  return text;
}

bool sdWriteBytes(const char *path, const uint8_t *data, size_t len) {
  if (SD_MMC.exists(path) && !SD_MMC.remove(path)) return false;
  File f = SD_MMC.open(path, FILE_WRITE);
  if (!f) return false;
  const size_t written = f.write(data, len);
  f.close();
  return written == len;
}

bool sdWriteText(const char *path, const String &text) {
  if (SD_MMC.exists(path) && !SD_MMC.remove(path)) return false;
  File f = SD_MMC.open(path, FILE_WRITE);
  if (!f) return false;
  const size_t written = f.print(text);
  f.close();
  return written == text.length();
}

bool sdEnsureDir(const char *dir) {
  if (SD_MMC.exists(dir)) return true;
  return SD_MMC.mkdir(dir);
}

String jsonEscape(const String &value) {
  String escaped;
  escaped.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (c == '\\' || c == '"') {
      escaped += '\\';
      escaped += c;
      continue;
    }
    if (c == '\n') {
      escaped += "\\n";
      continue;
    }
    if (c == '\r') continue;
    escaped += c;
  }
  return escaped;
}

bool jsonExtractNumber(const String &source, const char *key, float &value) {
  const String pattern = String("\"") + key + '"';
  int start = source.indexOf(pattern);
  if (start < 0) return false;

  start = source.indexOf(':', start);
  if (start < 0) return false;
  ++start;

  while (start < static_cast<int>(source.length()) && isspace(static_cast<unsigned char>(source[start]))) {
    ++start;
  }

  int end = start;
  while (end < static_cast<int>(source.length())) {
    const char c = source[end];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') {
      ++end;
      continue;
    }
    break;
  }

  if (end <= start) return false;
  value = source.substring(start, end).toFloat();
  return true;
}

bool jsonExtractInt(const String &source, const char *key, int &value) {
  float number = 0.0f;
  if (!jsonExtractNumber(source, key, number)) return false;
  value = static_cast<int>(lroundf(number));
  return true;
}

bool jsonExtractString(const String &source, const char *key, String &value) {
  const String pattern = String("\"") + key + '"';
  int start = source.indexOf(pattern);
  if (start < 0) return false;

  start = source.indexOf(':', start);
  if (start < 0) return false;
  start = source.indexOf('"', start);
  if (start < 0) return false;
  ++start;

  int end = start;
  bool escaped = false;
  while (end < static_cast<int>(source.length())) {
    const char c = source[end];
    if (c == '\\' && !escaped) {
      escaped = true;
      ++end;
      continue;
    }
    if (c == '"' && !escaped) break;
    escaped = false;
    ++end;
  }

  if (end >= static_cast<int>(source.length())) return false;
  value = source.substring(start, end);
  value.replace("\\\"", "\"");
  value.replace("\\n", "\n");
  value.replace("\\\\", "\\");
  return true;
}

bool parseGaugeConfig(const String &objectText, GaugeConfig &gauge) {
  gauge = GaugeConfig();
  if (!jsonExtractInt(objectText, "id", gauge.id)) return false;
  jsonExtractString(objectText, "name", gauge.name);
  jsonExtractInt(objectText, "cx", gauge.cx);
  jsonExtractInt(objectText, "cy", gauge.cy);
  jsonExtractInt(objectText, "radius", gauge.radius);
  jsonExtractNumber(objectText, "angle_min", gauge.angleMin);
  jsonExtractNumber(objectText, "angle_max", gauge.angleMax);
  jsonExtractNumber(objectText, "value_min", gauge.valueMin);
  jsonExtractNumber(objectText, "value_max", gauge.valueMax);
  jsonExtractString(objectText, "unit", gauge.unit);

  gauge.valid = gauge.id >= 1 && gauge.id <= kGaugeCount && gauge.radius > 0;
  return gauge.valid;
}

bool loadDeviceConfig(DeviceConfig &config) {
  config = DeviceConfig();
  if (!SD_MMC.exists(kConfigPath)) return false;

  const String json = sdReadText(kConfigPath);
  if (json.isEmpty()) return false;

  jsonExtractString(json, "device_id", config.deviceId);
  jsonExtractInt(json, "interval_s", config.intervalS);

  const int gaugesPos = json.indexOf("\"gauges\"");
  if (gaugesPos < 0) return false;

  const int arrayStart = json.indexOf('[', gaugesPos);
  const int arrayEnd = json.indexOf(']', arrayStart);
  if (arrayStart < 0 || arrayEnd < 0) return false;

  int cursor = arrayStart + 1;
  while (cursor < arrayEnd) {
    const int objectStart = json.indexOf('{', cursor);
    if (objectStart < 0 || objectStart > arrayEnd) break;

    int depth = 0;
    int objectEnd = -1;
    for (int i = objectStart; i <= arrayEnd; ++i) {
      if (json[i] == '{') ++depth;
      if (json[i] == '}') {
        --depth;
        if (depth == 0) {
          objectEnd = i;
          break;
        }
      }
    }

    if (objectEnd < 0) break;
    GaugeConfig gauge;
    if (parseGaugeConfig(json.substring(objectStart, objectEnd + 1), gauge)) {
      config.gauges[gauge.id - 1] = gauge;
    }
    cursor = objectEnd + 1;
  }

  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    if (!config.gauges[i].valid) return false;
  }

  config.loaded = true;
  return true;
}

bool saveFrameAsJpeg(camera_fb_t *fb, const char *path) {
  if (!fb) return false;
  if (fb->format == PIXFORMAT_JPEG) {
    return sdWriteBytes(path, fb->buf, fb->len);
  }

  uint8_t *jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (!frame2jpg(fb, 85, &jpgBuf, &jpgLen)) {
    Serial.println("[CAM] frame2jpg failed");
    return false;
  }

  const bool ok = sdWriteBytes(path, jpgBuf, jpgLen);
  free(jpgBuf);
  return ok;
}

uint8_t rgb565ToGray(uint16_t pixel) {
  const uint8_t r = ((pixel >> 11) & 0x1F) * 255 / 31;
  const uint8_t g = ((pixel >> 5) & 0x3F) * 255 / 63;
  const uint8_t b = (pixel & 0x1F) * 255 / 31;
  return static_cast<uint8_t>((r * 30 + g * 59 + b * 11) / 100);
}

bool sampleGray(const camera_fb_t *fb, int x, int y, uint8_t &gray) {
  if (!fb || fb->format != PIXFORMAT_RGB565) return false;
  if (x < 0 || y < 0 || x >= fb->width || y >= fb->height) return false;

  const uint16_t *pixels = reinterpret_cast<const uint16_t *>(fb->buf);
  gray = rgb565ToGray(pixels[(y * fb->width) + x]);
  return true;
}

float angleToRadians(float angleDeg) {
  return (angleDeg - 90.0f) * PI / 180.0f;
}

float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float angleToValue(float angleDeg, const GaugeConfig &gauge) {
  const float span = gauge.angleMax - gauge.angleMin;
  if (fabsf(span) < 0.001f) return gauge.valueMin;
  const float ratio = clamp01((angleDeg - gauge.angleMin) / span);
  return gauge.valueMin + ratio * (gauge.valueMax - gauge.valueMin);
}

float scoreNeedleAngle(const camera_fb_t *fb, const GaugeConfig &gauge, float angleDeg) {
  const float angleRad = angleToRadians(angleDeg);
  const float dirX = cosf(angleRad);
  const float dirY = sinf(angleRad);
  const float perpX = -dirY;
  const float perpY = dirX;
  const float startRadius = gauge.radius * 0.22f;
  const float endRadius = gauge.radius * 0.95f;
  const float step = max(1.0f, gauge.radius / 28.0f);

  uint32_t sum = 0;
  uint16_t samples = 0;
  uint16_t iter = 0;
  for (float radius = startRadius; radius <= endRadius; radius += step) {
    for (int offset = -2; offset <= 2; ++offset) {
      const int x = lroundf(gauge.cx + dirX * radius + perpX * offset);
      const int y = lroundf(gauge.cy + dirY * radius + perpY * offset);
      uint8_t gray = 0;
      if (!sampleGray(fb, x, y, gray)) continue;
      sum += gray;
      ++samples;
      ++iter;
      if ((iter % 200) == 0) {
        delay(0);
      }
    }
  }

  if (samples == 0) return 255.0f;
  return static_cast<float>(sum) / samples;
}

GaugeReading analyzeGauge(const camera_fb_t *fb, const GaugeConfig &gauge) {
  GaugeReading reading;
  if (!gauge.valid || !fb || fb->format != PIXFORMAT_RGB565) return reading;

  constexpr float kAngleStep = 2.0f;
  constexpr int kMaxSamples = 181;
  float scores[kMaxSamples];
  float angles[kMaxSamples];
  int count = 0;

  const float start = min(gauge.angleMin, gauge.angleMax);
  const float end = max(gauge.angleMin, gauge.angleMax);
  for (float angle = start; angle <= end && count < kMaxSamples; angle += kAngleStep) {
    angles[count] = angle;
    scores[count] = scoreNeedleAngle(fb, gauge, angle);
    ++count;
    if ((count % 12) == 0) {
      delay(0);
    }
  }

  if (count == 0) return reading;

  int bestIndex = 0;
  for (int i = 1; i < count; ++i) {
    if (scores[i] < scores[bestIndex]) bestIndex = i;
  }

  int secondIndex = -1;
  for (int i = 0; i < count; ++i) {
    if (abs(i - bestIndex) <= 2) continue;
    if (secondIndex < 0 || scores[i] < scores[secondIndex]) secondIndex = i;
  }

  const float bestScore = scores[bestIndex];
  const float secondScore = secondIndex >= 0 ? scores[secondIndex] : 255.0f;
  const float contrast = max(0.0f, secondScore - bestScore);
  const float confidence = clamp01(contrast / 45.0f);

  if (bestScore > 220.0f || confidence < 0.08f) {
    return reading;
  }

  reading.detected = true;
  reading.angleDeg = angles[bestIndex];
  reading.value = angleToValue(reading.angleDeg, gauge);
  reading.confidence = confidence;
  reading.darkness = bestScore;
  return reading;
}

String buildAnalysisJson(const DeviceConfig &config, const GaugeReading readings[], int frameWidth, int frameHeight, uint32_t elapsedMs) {
  const bool g1 = readings[0].detected;
  const bool g2 = readings[1].detected;
  String status = "ok";
  if (!g1 && !g2) status = "both_not_detected";
  else if (!g1) status = "gauge_1_not_detected";
  else if (!g2) status = "gauge_2_not_detected";

  String json = "{";
  json += "\"device_id\":\"" + jsonEscape(config.deviceId) + "\",";
  json += "\"status\":\"" + status + "\",";
  json += "\"frame\":{\"width\":" + String(frameWidth) + ",\"height\":" + String(frameHeight) + "},";
  json += "\"processing_ms\":" + String(elapsedMs) + ',';
  json += "\"gauges\":[";

  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    if (i > 0) json += ',';
    json += '{';
    json += "\"id\":" + String(config.gauges[i].id) + ',';
    json += "\"name\":\"" + jsonEscape(config.gauges[i].name) + "\",";
    json += "\"unit\":\"" + jsonEscape(config.gauges[i].unit) + "\",";
    json += "\"detected\":";
    json += readings[i].detected ? "true" : "false";
    if (readings[i].detected) {
      json += ",\"angle_deg\":" + String(readings[i].angleDeg, 2);
      json += ",\"value\":" + String(readings[i].value, 3);
      json += ",\"confidence\":" + String(readings[i].confidence, 3);
      json += ",\"darkness\":" + String(readings[i].darkness, 1);
    }
    json += '}';
  }

  json += "]}";
  return json;
}

bool captureAndSave() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAM] Frame capture failed");
    return false;
  }

  const bool ok = saveFrameAsJpeg(fb, kPhotoPath);
  const size_t len = fb->len;
  esp_camera_fb_return(fb);
  if (!ok) {
    Serial.println("[SD]  Photo write failed");
    return false;
  }

  Serial.printf("[CAM] Photo saved (%u bytes)\n", len);
  return true;
}

void handleRoot() {
  if (!checkAuth()) return;
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleCapture() {
  if (!checkAuth()) return;
  if (!captureAndSave()) {
    server.send(500, "text/plain", "capture_failed");
    return;
  }
  server.send(200, "text/plain", "ok");
}

void handleAnalyze() {
  if (!checkAuth()) return;

  DeviceConfig config;
  if (!loadDeviceConfig(config)) {
    server.send(409, "text/plain", "calibration_missing_or_invalid");
    return;
  }

  const uint32_t start = millis();
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "capture_failed");
    return;
  }

  if (fb->format != PIXFORMAT_RGB565) {
    esp_camera_fb_return(fb);
    server.send(500, "text/plain", "unexpected_pixel_format");
    return;
  }

  GaugeReading readings[kGaugeCount];
  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    readings[i] = analyzeGauge(fb, config.gauges[i]);
  }

  const int frameWidth = fb->width;
  const int frameHeight = fb->height;
  const bool photoOk = saveFrameAsJpeg(fb, kPhotoPath);
  const String response = buildAnalysisJson(config, readings, frameWidth, frameHeight, millis() - start);
  esp_camera_fb_return(fb);

  if (!photoOk) {
    server.send(500, "text/plain", "photo_save_failed");
    return;
  }

  Serial.printf("[CV] Analysis done in %u ms  frame=%dx%d\n",
    static_cast<unsigned>(millis() - start), frameWidth, frameHeight);
  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    const GaugeReading &r = readings[i];
    const GaugeConfig  &g = config.gauges[i];
    if (r.detected) {
      Serial.printf("[CV] M%d %-16s  angle=%7.2f deg  value=%6.3f %s  conf=%.3f  dark=%.1f\n",
        g.id, g.name.c_str(), r.angleDeg, r.value, g.unit.c_str(), r.confidence, r.darkness);
    } else {
      Serial.printf("[CV] M%d %-16s  NOT DETECTED\n", g.id, g.name.c_str());
    }
  }
  server.send(200, "application/json", response);
}

void handlePhoto() {
  if (!checkAuth()) return;
  if (!SD_MMC.exists(kPhotoPath)) {
    server.send(404, "text/plain", "photo_not_found");
    return;
  }
  File f = SD_MMC.open(kPhotoPath, FILE_READ);
  if (!f) {
    server.send(500, "text/plain", "open_failed");
    return;
  }
  server.streamFile(f, "image/jpeg");
  f.close();
}

void handleGetConfig() {
  if (!checkAuth()) return;
  if (!SD_MMC.exists(kConfigPath)) {
    server.send(404, "text/plain", "config_not_found");
    return;
  }
  File f = SD_MMC.open(kConfigPath, FILE_READ);
  if (!f) {
    server.send(500, "text/plain", "open_failed");
    return;
  }
  server.streamFile(f, "application/json");
  f.close();
}

void handleSaveConfig() {
  if (!checkAuth()) return;
  const String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "text/plain", "empty_body");
    return;
  }
  if (!sdEnsureDir("/config")) {
    server.send(500, "text/plain", "mkdir_failed");
    return;
  }
  if (!sdWriteText(kConfigPath, body)) {
    server.send(500, "text/plain", "write_failed");
    return;
  }
  Serial.println("[CFG] config.json saved");
  server.send(200, "text/plain", "ok");
}

bool setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;

  // RGB565 is much heavier than JPEG; keep startup frame size conservative.
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 16;
  config.fb_count = 1;

  esp_err_t camErr = esp_camera_init(&config);
  if (camErr != ESP_OK) {
    Serial.printf("[CAM] Camera init failed at QVGA (0x%x), retry QQVGA\n", static_cast<unsigned>(camErr));
    esp_camera_deinit();
    config.frame_size = FRAMESIZE_QQVGA;
    camErr = esp_camera_init(&config);
  }

  if (camErr != ESP_OK) {
    Serial.printf("[CAM] Camera init failed (0x%x)\n", static_cast<unsigned>(camErr));
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
  }

  Serial.println("[CAM] Camera ready");
  return true;
}

bool setupStorage() {
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[SD] SD_MMC mount failed");
    return false;
  }

  const uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[SD] No SD card detected");
    return false;
  }

  Serial.println("[SD] Storage ready");
  return true;
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 30000) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected. IP: ");
    Serial.println(WiFi.localIP());
    Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("[WiFi] Connection timeout");
  }
}

static const uint32_t kWiFiReconnectIntervalMs = 15000;
static uint32_t lastWiFiReconnectMs = 0;

void maintainWiFi(uint32_t nowMs) {
  if (WiFi.status() == WL_CONNECTED) return;
  if (nowMs - lastWiFiReconnectMs < kWiFiReconnectIntervalMs) return;

  lastWiFiReconnectMs = nowMs;
  Serial.printf("[WiFi] Reconnect attempt (status=%d)\n", static_cast<int>(WiFi.status()));
  if (!WiFi.reconnect()) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/capture", HTTP_POST, handleCapture);
  server.on("/analyze", HTTP_POST, handleAnalyze);
  server.on("/photo.jpg", HTTP_GET, handlePhoto);
  server.on("/config", HTTP_GET, handleGetConfig);
  server.on("/config", HTTP_POST, handleSaveConfig);
  server.begin();
  Serial.println("[WEB]  Server started on port 80");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("[BOOT] ESP32-CAM Manometry v0.3");

  if (!setupStorage()) {
    Serial.println("[BOOT] Fatal: storage setup failed");
    while (true) {
      Serial.println("[BOOT] Waiting in fatal loop: storage");
      delay(1000);
    }
  }

  if (!setupCamera()) {
    Serial.println("[BOOT] Fatal: camera setup failed");
    while (true) {
      Serial.println("[BOOT] Waiting in fatal loop: camera");
      delay(1000);
    }
  }

  setupWiFi();
  setupWebServer();

  if (!captureAndSave()) Serial.println("[BOOT] Initial capture failed (continuing)");
}

static void runPeriodicAnalysis() {
  DeviceConfig config;
  if (!loadDeviceConfig(config)) {
    Serial.println("[CV] Periodic: calibration missing, skipping");
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CV] Periodic: capture failed");
    return;
  }

  if (fb->format != PIXFORMAT_RGB565) {
    esp_camera_fb_return(fb);
    Serial.println("[CV] Periodic: unexpected pixel format");
    return;
  }

  const uint32_t start = millis();
  GaugeReading readings[kGaugeCount];
  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    readings[i] = analyzeGauge(fb, config.gauges[i]);
  }

  const int frameWidth = fb->width;
  const int frameHeight = fb->height;
  saveFrameAsJpeg(fb, kPhotoPath);
  const uint32_t elapsed = millis() - start;
  esp_camera_fb_return(fb);

  Serial.printf("[CV] Periodic done in %u ms  frame=%dx%d\n", elapsed, frameWidth, frameHeight);
  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    const GaugeReading &r = readings[i];
    const GaugeConfig  &g = config.gauges[i];
    if (r.detected) {
      Serial.printf("[CV] M%d %-16s  angle=%7.2f deg  value=%6.3f %s  conf=%.3f  dark=%.1f\n",
        g.id, g.name.c_str(), r.angleDeg, r.value, g.unit.c_str(), r.confidence, r.darkness);
    } else {
      Serial.printf("[CV] M%d %-16s  NOT DETECTED\n", g.id, g.name.c_str());
    }
  }
}

static const uint32_t kAnalysisIntervalMs = 30000;
static uint32_t lastAnalysisMs = 0;
static volatile bool analysisInProgress = false;
static const bool kEnablePeriodicAnalysis = false;
static const uint32_t kHeartbeatIntervalMs = 10000;
static uint32_t lastHeartbeatMs = 0;

void loop() {
  server.handleClient();

  const uint32_t now = millis();
  maintainWiFi(now);

  if (now - lastHeartbeatMs >= kHeartbeatIntervalMs) {
    lastHeartbeatMs = now;
    const bool wifiOk = (WiFi.status() == WL_CONNECTED);
    Serial.printf("[SYS] alive uptime=%lus wifi=%s ip=%s\n",
      static_cast<unsigned long>(now / 1000),
      wifiOk ? "ok" : "down",
      wifiOk ? WiFi.localIP().toString().c_str() : "-");
  }

  if (kEnablePeriodicAnalysis && !analysisInProgress && (now - lastAnalysisMs >= kAnalysisIntervalMs)) {
    lastAnalysisMs = now;
    analysisInProgress = true;
    runPeriodicAnalysis();
    analysisInProgress = false;
  }
}
