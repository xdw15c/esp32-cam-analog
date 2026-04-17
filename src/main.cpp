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
#define FLASH_LED_GPIO_NUM 4

static const uint16_t kFlashWarmupMs = 70;
static bool gFlashEnabled = true;
static uint32_t gAnalysisIntervalMs = 60000;
static const uint32_t kMinAnalysisIntervalMs = 10000;
static const uint32_t kMaxAnalysisIntervalMs = 3600000;
static bool gStorageReady = false;
static bool gCameraReady = false;
static bool gEmergencyMode = false;

static const uint32_t kEmergencyBlinkIntervalMs = 10000;
static const uint16_t kEmergencyBlinkPulseMs = 120;
static const uint16_t kEmergencyBlinkIntraPauseMs = 220;
static const uint16_t kEmergencyBlinkStepPauseMs = 900;

enum class EmergencyFault : uint8_t {
  None = 0,
  Storage = 1,
  Camera = 2,
  StorageAndCamera = 3
};

static EmergencyFault gEmergencyFault = EmergencyFault::None;
static uint8_t gEmergencyBlinkTargetCount = 2;
static uint8_t gEmergencyBlinkDoneCount = 0;

enum class EmergencyBlinkState : uint8_t {
  Idle = 0,
  On1,
  Off1,
  On2,
  Off2,
  BetweenSteps
};

static EmergencyBlinkState gEmergencyBlinkState = EmergencyBlinkState::Idle;
static uint32_t gEmergencyBlinkStateMs = 0;
static uint32_t gEmergencyNextBlinkMs = 0;

static const char *kPhotoPath = "/latest.jpg";
static const char *kConfigPath = "/config/config.json";
static const char *kLogDir = "/logs";
static const char *kLogPath = "/logs/system.log";
static const size_t kWebLogTailBytes = 8192;
static uint8_t gJpegWorkBuffer[3100];
static const uint8_t kGaugeCount = 2;
static File gUploadFile;
static const float kAngleSearchMarginDeg = 8.0f;

enum class AnalysisInputMode : uint8_t {
  Auto = 0,
  Camera = 1,
  SdPhoto = 2
};

enum class AnalysisSourceCode : uint16_t {
  Unavailable = 0,
  CameraLive = 1,
  SdPhoto = 2
};

static AnalysisInputMode gAnalysisInputMode = AnalysisInputMode::Auto;
static AnalysisSourceCode gLastAnalysisSource = AnalysisSourceCode::Unavailable;

const char *analysisInputModeName(AnalysisInputMode mode) {
  switch (mode) {
    case AnalysisInputMode::Camera:
      return "camera";
    case AnalysisInputMode::SdPhoto:
      return "sd_photo";
    case AnalysisInputMode::Auto:
    default:
      return "auto";
  }
}

bool parseAnalysisInputMode(const String &value, AnalysisInputMode &mode) {
  if (value == "camera") {
    mode = AnalysisInputMode::Camera;
    return true;
  }
  if (value == "sd_photo" || value == "sd") {
    mode = AnalysisInputMode::SdPhoto;
    return true;
  }
  if (value == "auto") {
    mode = AnalysisInputMode::Auto;
    return true;
  }
  return false;
}

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
  String analysisMode = "color_target";
  String needleColor = "#D00000";
  String backgroundColor = "#7F7F7F";
  String textColor = "#101010";
  bool valid = false;
};

struct DeviceConfig {
  String deviceId = "esp32cam-01";
  int intervalS = 60;
  bool flashEnabled = true;
  String analysisInputSource = "auto";
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

struct GaugeAnalysisDebug {
  bool enabled = false;
  float searchStartDeg = 0.0f;
  float searchEndDeg = 0.0f;
  int scannedAngles = 0;
  int pointerCandidates = 0;
  float selectedAngleDeg = 0.0f;
  float selectedScore = 0.0f;
  float selectedTipReach = 0.0f;
  float oppositeAngleDeg = 0.0f;
  float oppositeScore = 0.0f;
  float oppositeTipReach = 0.0f;
  bool oppositeDominates = false;
  String selectedReason;
};

WebServer server(80);
WiFiServer modbusServer(502);
WiFiClient modbusClient;
static const uint8_t kModbusUnitId = 1;

static const uint16_t kModbusRegCount = 16;
uint16_t modbusHolding[kModbusRegCount] = {0};
GaugeAnalysisDebug gLastGaugeDebug[kGaugeCount];

GaugeAnalysisDebug *debugForGauge(const GaugeConfig &gauge) {
  if (gauge.id < 1 || gauge.id > static_cast<int>(kGaugeCount)) return nullptr;
  return &gLastGaugeDebug[gauge.id - 1];
}

void resetGaugeDebug(const GaugeConfig &gauge, float rangeStartDeg, float rangeEndDeg, int scannedAngles) {
  GaugeAnalysisDebug *dbg = debugForGauge(gauge);
  if (!dbg) return;
  *dbg = GaugeAnalysisDebug();
  dbg->enabled = true;
  dbg->searchStartDeg = rangeStartDeg;
  dbg->searchEndDeg = rangeEndDeg;
  dbg->scannedAngles = scannedAngles;
}

camera_fb_t *captureFrameWithFlash();

struct AnalysisFrame {
  camera_fb_t *fb = nullptr;
  camera_fb_t ownedFb = {};
  uint8_t *ownedPixels = nullptr;
  uint8_t *ownedJpeg = nullptr;
};

enum ModbusRegister : uint16_t {
  REG_STATUS = 0,
  REG_G1_X100 = 1,
  REG_G2_X100 = 2,
  REG_DIFF_X100 = 3,
  REG_CONF1_X1000 = 4,
  REG_CONF2_X1000 = 5,
  REG_UPTIME_LO = 6,
  REG_UPTIME_HI = 7,
  REG_WIFI_RSSI = 8,
  REG_HEARTBEAT = 9,
  REG_FAULT_CODE = 10,
  REG_ANALYSIS_SOURCE = 11,
  REG_RESERVED12 = 12,
  REG_RESERVED13 = 13,
  REG_RESERVED14 = 14,
  REG_RESERVED15 = 15
};

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
.cfg-ana{margin:.6rem 0 0;padding:.55rem;border:1px dashed #cbd5e1;border-radius:8px;background:#f8fafc}
.cfg-ana-grid{display:grid;grid-template-columns:repeat(2,minmax(140px,1fr));gap:.5rem}
@media(max-width:640px){.cfg-ana-grid{grid-template-columns:1fr}}
select,input[type=color]{width:100%;border:1px solid #cbd5e1;border-radius:6px;padding:.3rem .45rem;font-size:.875rem;background:#fff;min-height:34px}
</style>
</head><body>

<div class="top">
  <h1>&#128247; ESP32-CAM Manometry</h1>
  <nav>
    <button class="active" onclick="tab('cam',this)">Kamera</button>
    <button onclick="tab('roi',this)">Kalibracja ROI</button>
    <button onclick="tab('modbus',this)">Modbus</button>
    <button onclick="tab('logs',this)">Logi</button>
  </nav>
</div>

<div id="tab-cam" class="page active">
  <div class="card">
    <div class="row">
      <button onclick="capture()">&#128247; Zrob zdjecie / odswiez SD</button>
      <button class="grn" onclick="analyzeCapture()">&#129504; Analizuj obraz</button>
      <button class="sec" onclick="reloadCam()">&#8635; Odswiez</button>
      <input type="file" id="upload-input" accept=".jpg,.jpeg,image/jpeg" style="display:none" onchange="uploadPhoto(this)">
      <button class="sec" onclick="document.getElementById('upload-input').click()">&#128228; Upload zdjecia</button>
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
      <button onclick="captureAndSwitch()">&#128247; Wczytaj zdjecie z SD</button>
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
      <div class="cfg-ana">
        <label>Tryb analizy M1</label>
        <select id="g1-analysis-mode" onchange="onGaugeAnalysisModeChanged(1);updatePreview();">
          <option value="classic_darkness">Klasyczna (najciemniejsza wskazowka)</option>
          <option value="color_target">Kolorowa (wskazany kolor wskazowki)</option>
          <option value="hybrid_pca_hough">Hybrydowa (PCA blob + mini-Hough)</option>
        </select>
        <div id="g1-color-settings" class="cfg-ana-grid">
          <div>
            <label>Kolor wskazowki</label>
            <input type="color" id="g1-needle-color" value="#d00000" oninput="updatePreview()">
          </div>
          <div>
            <label>Kolor tla tarczy</label>
            <input type="color" id="g1-background-color" value="#7f7f7f" oninput="updatePreview()">
          </div>
          <div>
            <label>Kolor cyfr/napisow</label>
            <input type="color" id="g1-text-color" value="#101010" oninput="updatePreview()">
          </div>
        </div>
      </div>
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
      <div class="cfg-ana">
        <label>Tryb analizy M2</label>
        <select id="g2-analysis-mode" onchange="onGaugeAnalysisModeChanged(2);updatePreview();">
          <option value="classic_darkness">Klasyczna (najciemniejsza wskazowka)</option>
          <option value="color_target">Kolorowa (wskazany kolor wskazowki)</option>
          <option value="hybrid_pca_hough">Hybrydowa (PCA blob + mini-Hough)</option>
        </select>
        <div id="g2-color-settings" class="cfg-ana-grid">
          <div>
            <label>Kolor wskazowki</label>
            <input type="color" id="g2-needle-color" value="#d00000" oninput="updatePreview()">
          </div>
          <div>
            <label>Kolor tla tarczy</label>
            <input type="color" id="g2-background-color" value="#7f7f7f" oninput="updatePreview()">
          </div>
          <div>
            <label>Kolor cyfr/napisow</label>
            <input type="color" id="g2-text-color" value="#101010" oninput="updatePreview()">
          </div>
        </div>
      </div>
    </div>
  </div>
  <div class="card" style="margin-top:1rem">
    <div class="cfg-ana">
      <h3 style="margin:0 0 .5rem">Ustawienia urzadzenia</h3>
      <label style="display:flex;align-items:center;gap:.5rem;color:#1e293b;margin:.2rem 0 .6rem">
        <input type="checkbox" id="flash-enabled" checked onchange="updatePreview()">
        Uzyj diody flash podczas robienia zdjecia
      </label>
      <label>Zrodlo analizy obrazu</label>
      <select id="analysis-input-source" onchange="updatePreview()">
        <option value="auto">Auto (kamera, a przy bledzie SD)</option>
        <option value="camera">Tylko kamera live</option>
        <option value="sd_photo">Tylko plik SD (/latest.jpg)</option>
      </select>
      <label>Czas pomiedzy automatycznym zdjeciem i analiza (sekundy)</label>
      <input type="number" id="auto-interval-s" min="10" max="3600" value="60" oninput="updatePreview()">
      <p class="hint" style="margin-top:.4rem">Zakres: 10..3600 s</p>
    </div>
    <h3 style="margin:0 0 .5rem">config.json (podglad)</h3>
    <pre id="cfg-pre">{}</pre>
  </div>
</div>

<div id="tab-modbus" class="page">
  <div class="card">
    <div class="row">
      <button class="sec" onclick="loadModbusStatus()">&#8635; Odswiez status Modbus</button>
    </div>
    <h3 style="margin:0 0 .5rem">Parametry polaczenia</h3>
    <pre id="modbus-conn">Ladowanie...</pre>
    <h3 style="margin:1rem 0 .5rem">Stan urzadzenia</h3>
    <pre id="modbus-state">Ladowanie...</pre>
    <h3 style="margin:1rem 0 .5rem">Rejestry holding (0..15)</h3>
    <pre id="modbus-regs">Ladowanie...</pre>
    <h3 style="margin:1rem 0 .5rem">Opis rejestrow i kody</h3>
    <pre id="modbus-help">Ladowanie...</pre>
  </div>
</div>

<div id="tab-logs" class="page">
  <div class="card">
    <div class="row">
      <button class="sec" onclick="loadLogs()">&#8635; Odswiez log</button>
      <button onclick="clearLogs()">Wyczysc log</button>
    </div>
    <p class="hint">Pokazywane sa ostatnie wpisy z pliku /logs/system.log.</p>
    <pre id="logs-pre">Ladowanie...</pre>
  </div>
</div>

<script>
const COLORS=['','#e11d48','#2563eb'];
let mode=0;
let modbusTimer=null;
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
  if(modbusTimer){clearInterval(modbusTimer);modbusTimer=null;}
  if(id==='roi'){loadRoiImg();updatePreview();}
  if(id==='modbus'){loadModbusStatus();modbusTimer=setInterval(loadModbusStatus,2000);}
  if(id==='logs'){loadLogs();}
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

async function uploadPhoto(input){
  if(!input.files||!input.files[0])return;
  const info=document.getElementById('img-info');
  info.textContent='Wysylanie...';
  const fd=new FormData();
  fd.append('photo',input.files[0],'photo.jpg');
  const r=await fetch('/upload',{method:'POST',body:fd});
  input.value='';
  if(!r.ok){info.textContent='Blad uploadu: '+(await r.text());return;}
  loadCamImg();
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
  const autoInterval=Math.max(10,Math.min(3600,gv('auto-interval-s')||60));
  const sourceEl=document.getElementById('analysis-input-source');
  return{
    device_id:'esp32cam-01',
    interval_s:autoInterval,
    flash_enabled:!!document.getElementById('flash-enabled').checked,
    analysis_input_source:sourceEl?sourceEl.value:'auto',
    gauges:[1,2].map(n=>{
      const p='g'+n;
      return{id:n,name:document.getElementById(p+'-name').value,
        cx:gv(p+'-cx'),cy:gv(p+'-cy'),radius:gv(p+'-r'),
        angle_min:gv(p+'-amin'),angle_max:gv(p+'-amax'),
        value_min:gv(p+'-vmin'),value_max:gv(p+'-vmax'),
        unit:document.getElementById(p+'-unit').value,
        analysis_mode:document.getElementById(p+'-analysis-mode').value,
        needle_color:document.getElementById(p+'-needle-color').value,
        background_color:document.getElementById(p+'-background-color').value,
        text_color:document.getElementById(p+'-text-color').value};
    })
  };
}

function applyConfig(cfg){
  if(!cfg||!Array.isArray(cfg.gauges))return;
  const flashEl=document.getElementById('flash-enabled');
  const intervalEl=document.getElementById('auto-interval-s');
  const sourceEl=document.getElementById('analysis-input-source');
  if(flashEl)flashEl.checked=(cfg.flash_enabled!==false);
  if(intervalEl)intervalEl.value=Math.max(10,Math.min(3600,(+cfg.interval_s)||60));
  if(sourceEl)sourceEl.value=(cfg.analysis_input_source||'auto');
  cfg.gauges.forEach(g=>{
    const n=g.id,p='g'+n;
    const s=(id,v)=>{const el=document.getElementById(id);if(el&&v!==undefined)el.value=v;};
    s(p+'-name',g.name);s(p+'-cx',g.cx);s(p+'-cy',g.cy);s(p+'-r',g.radius);
    s(p+'-amin',g.angle_min);s(p+'-amax',g.angle_max);
    s(p+'-vmin',g.value_min);s(p+'-vmax',g.value_max);s(p+'-unit',g.unit);
    s(p+'-analysis-mode',g.analysis_mode||cfg.analysis_mode||'color_target');
    s(p+'-needle-color',g.needle_color||cfg.needle_color||'#d00000');
    s(p+'-background-color',g.background_color||cfg.background_color||'#7f7f7f');
    s(p+'-text-color',g.text_color||cfg.text_color||'#101010');
    onGaugeAnalysisModeChanged(n);
  });
  redraw();
}

function onGaugeAnalysisModeChanged(n){
  const mode=document.getElementById('g'+n+'-analysis-mode').value;
  const box=document.getElementById('g'+n+'-color-settings');
  box.style.display=(mode==='color_target')?'block':'none';
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

function renderModbusRegisters(registers){
  if(!Array.isArray(registers)||!registers.length)return 'Brak danych';
  const descByAddr={
    0:'status',
    1:'g1_x100',
    2:'g2_x100',
    3:'diff_x100',
    4:'conf1_x1000',
    5:'conf2_x1000',
    6:'uptime_lo',
    7:'uptime_hi',
    8:'wifi_rssi',
    9:'heartbeat',
    10:'fault_code',
    11:'analysis_source',
    12:'reserved12',
    13:'reserved13',
    14:'reserved14',
    15:'reserved15'
  };
  return registers.map(r=>{
    const name=descByAddr[r.addr]||'unknown';
    return 'addr '+r.addr+' (4'+String(1+r.addr).padStart(4,'0')+')  '+name+'  u16='+r.u16+'  i16='+r.i16+'  hex='+r.hex;
  }).join('\n');
}

function decodeStatusCode(code){
  if(code===0)return 'ok';
  if(code===11)return 'gauge_1_not_detected';
  if(code===12)return 'gauge_2_not_detected';
  if(code===13)return 'both_not_detected';
  return 'unknown';
}

function decodeFaultCode(code){
  if(code===0)return '0: brak bledow';
  const parts=[];
  if(code&0x01)parts.push('bit0=storage_not_ready');
  if(code&0x02)parts.push('bit1=camera_not_ready');
  return code+': '+(parts.length?parts.join(', '):'nieznany kod');
}

function decodeAnalysisSource(code){
  if(code===0)return '0: unavailable';
  if(code===1)return '1: camera_live';
  if(code===2)return '2: sd_photo';
  return code+': unknown';
}

function renderModbusHelp(data){
  const regs=Array.isArray(data&&data.registers)?data.registers:[];
  const byAddr={};
  regs.forEach(r=>{byAddr[r.addr]=r;});
  const u16=addr=>byAddr[addr]?byAddr[addr].u16:0;
  const i16=addr=>byAddr[addr]?byAddr[addr].i16:0;

  const lines=[];
  lines.push('Mapa rejestrow (holding 4xxxx):');
  lines.push('40001 addr0  status        : '+u16(0)+' -> '+decodeStatusCode(u16(0)));
  lines.push('40002 addr1  g1_x100       : '+i16(1)+' (wartosc M1 * 100)');
  lines.push('40003 addr2  g2_x100       : '+i16(2)+' (wartosc M2 * 100)');
  lines.push('40004 addr3  diff_x100     : '+i16(3)+' (M1-M2)*100');
  lines.push('40005 addr4  conf1_x1000   : '+u16(4)+' (pewnosc M1 0..1000)');
  lines.push('40006 addr5  conf2_x1000   : '+u16(5)+' (pewnosc M2 0..1000)');
  lines.push('40007 addr6  uptime_lo     : '+u16(6)+' (sekundy low word)');
  lines.push('40008 addr7  uptime_hi     : '+u16(7)+' (sekundy high word)');
  lines.push('40009 addr8  wifi_rssi     : '+i16(8)+' dBm');
  lines.push('40010 addr9  heartbeat     : '+u16(9)+' (licznik zycia)');
  lines.push('40011 addr10 fault_code    : '+decodeFaultCode(u16(10)));
  lines.push('40012 addr11 analysis_src  : '+decodeAnalysisSource(u16(11)));
  lines.push('40013 addr12 reserved12    : '+u16(12));
  lines.push('40014 addr13 reserved13    : '+u16(13));
  lines.push('40015 addr14 reserved14    : '+u16(14));
  lines.push('40016 addr15 reserved15    : '+u16(15));
  lines.push('');
  lines.push('Kody specjalne:');
  lines.push('status: 0=ok, 11=brak M1, 12=brak M2, 13=brak obu');
  lines.push('fault_code bitmask: bit0=storage, bit1=camera');
  lines.push('analysis_source: 0=unavailable, 1=camera_live, 2=sd_photo');
  lines.push('Brak pomiaru w rejestrach i16: -32768');
  return lines.join('\n');
}

async function loadModbusStatus(){
  const conn=document.getElementById('modbus-conn');
  const state=document.getElementById('modbus-state');
  const regs=document.getElementById('modbus-regs');
  const help=document.getElementById('modbus-help');
  try{
    const r=await fetch('/modbus/status');
    if(!r.ok){
      const t=await r.text();
      conn.textContent='Blad odczytu: '+(t||r.status);
      if(help)help.textContent='Brak danych';
      return;
    }
    const data=await r.json();
    conn.textContent='IP: '+data.ip+'\nPort: '+data.port+'\nUnit ID: '+data.unit_id+'\nFunkcja: '+data.function;
    state.textContent='WiFi: '+data.wifi_status+'\nRSSI: '+data.rssi_dbm+' dBm\nHeartbeat: '+data.heartbeat+'\nStatus: '+decodeStatusCode((data.registers&&data.registers[0])?data.registers[0].u16:0)+'\nFault: '+decodeFaultCode(data.fault_code||0)+'\nSource: '+decodeAnalysisSource((data.registers&&data.registers[11])?data.registers[11].u16:0)+'\nInput mode: '+(data.analysis_input_mode||'auto')+'\nCamera ready: '+(data.camera_ready?'yes':'no')+'\nStorage ready: '+(data.storage_ready?'yes':'no');
    regs.textContent=renderModbusRegisters(data.registers);
    if(help)help.textContent=renderModbusHelp(data);
  }catch(e){
    conn.textContent='Blad: '+e;
    if(help)help.textContent='Blad: '+e;
  }
}

async function loadLogs(){
  const out=document.getElementById('logs-pre');
  out.textContent='Ladowanie...';
  try{
    const r=await fetch('/logs?t='+Date.now());
    const text=await r.text();
    if(!r.ok){out.textContent='Blad odczytu logu: '+(text||r.status);return;}
    out.textContent=text&&text.length?text:'Brak wpisow logu';
  }catch(e){
    out.textContent='Blad: '+e;
  }
}

async function clearLogs(){
  if(!confirm('Na pewno wyczyscic log?'))return;
  const r=await fetch('/logs/clear',{method:'POST'});
  const text=await r.text();
  if(!r.ok){alert('Blad kasowania logu: '+(text||r.status));return;}
  loadLogs();
}

loadCamImg();
loadConfig();
onGaugeAnalysisModeChanged(1);
onGaugeAnalysisModeChanged(2);
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

String sdReadTextTail(const char *path, size_t maxBytes) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return String();

  const size_t fileSize = f.size();
  size_t startOffset = 0;
  if (maxBytes > 0 && fileSize > maxBytes) {
    startOffset = fileSize - maxBytes;
  }

  if (startOffset > 0 && !f.seek(startOffset)) {
    f.close();
    return String();
  }

  // Start at a whole line when serving the tail.
  if (startOffset > 0) {
    while (f.available()) {
      if (f.read() == '\n') break;
    }
  }

  String text;
  text.reserve(maxBytes > 0 ? maxBytes + 32 : fileSize + 1);
  while (f.available()) {
    text += static_cast<char>(f.read());
  }
  f.close();
  return text;
}

bool appendLogLine(const String &line) {
  if (!gStorageReady) return false;
  if (!sdEnsureDir(kLogDir)) return false;

  File f = SD_MMC.open(kLogPath, FILE_APPEND);
  if (!f) return false;
  const size_t written = f.println(line);
  f.close();
  return written > 0;
}

void logEvent(const char *level, const String &message) {
  String line;
  line.reserve(48 + message.length());
  line += String(millis());
  line += " [";
  line += (level ? level : "INFO");
  line += "] ";
  line += message;
  Serial.println(line);
  appendLogLine(line);
}

void logEvent(const char *level, const char *message) {
  logEvent(level, String(message ? message : ""));
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

String hex4(uint16_t value) {
  char buf[5];
  snprintf(buf, sizeof(buf), "%04X", value);
  return String(buf);
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

bool jsonExtractBool(const String &source, const char *key, bool &value) {
  const String pattern = String("\"") + key + '"';
  int start = source.indexOf(pattern);
  if (start < 0) return false;

  start = source.indexOf(':', start);
  if (start < 0) return false;
  ++start;

  while (start < static_cast<int>(source.length()) && isspace(static_cast<unsigned char>(source[start]))) {
    ++start;
  }

  if (source.startsWith("true", start)) {
    value = true;
    return true;
  }
  if (source.startsWith("false", start)) {
    value = false;
    return true;
  }
  return false;
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
  jsonExtractString(objectText, "analysis_mode", gauge.analysisMode);
  jsonExtractString(objectText, "needle_color", gauge.needleColor);
  jsonExtractString(objectText, "background_color", gauge.backgroundColor);
  jsonExtractString(objectText, "text_color", gauge.textColor);

  gauge.valid = gauge.id >= 1 && gauge.id <= kGaugeCount && gauge.radius > 0;
  return gauge.valid;
}

bool parseHexColor(const String &value, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (value.length() != 7 || value[0] != '#') return false;

  auto hexToNibble = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
  };

  const int r1 = hexToNibble(value[1]);
  const int r2 = hexToNibble(value[2]);
  const int g1 = hexToNibble(value[3]);
  const int g2 = hexToNibble(value[4]);
  const int b1 = hexToNibble(value[5]);
  const int b2 = hexToNibble(value[6]);
  if (r1 < 0 || r2 < 0 || g1 < 0 || g2 < 0 || b1 < 0 || b2 < 0) return false;

  r = static_cast<uint8_t>((r1 << 4) | r2);
  g = static_cast<uint8_t>((g1 << 4) | g2);
  b = static_cast<uint8_t>((b1 << 4) | b2);
  return true;
}

String formatHexColor(uint8_t r, uint8_t g, uint8_t b) {
  char out[8];
  snprintf(out, sizeof(out), "#%02X%02X%02X", r, g, b);
  return String(out);
}

bool loadDeviceConfig(DeviceConfig &config) {
  config = DeviceConfig();
  if (!SD_MMC.exists(kConfigPath)) return false;

  const String json = sdReadText(kConfigPath);
  if (json.isEmpty()) return false;

  jsonExtractString(json, "device_id", config.deviceId);
  jsonExtractInt(json, "interval_s", config.intervalS);
  jsonExtractBool(json, "flash_enabled", config.flashEnabled);
  jsonExtractString(json, "analysis_input_source", config.analysisInputSource);

  String legacyMode = "color_target";
  String legacyNeedle = "#D00000";
  String legacyBackground = "#7F7F7F";
  String legacyText = "#101010";
  jsonExtractString(json, "analysis_mode", legacyMode);
  jsonExtractString(json, "needle_color", legacyNeedle);
  jsonExtractString(json, "background_color", legacyBackground);
  jsonExtractString(json, "text_color", legacyText);

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
      if (gauge.analysisMode != "classic_darkness" && gauge.analysisMode != "color_target" && gauge.analysisMode != "hybrid_pca_hough") {
        gauge.analysisMode = legacyMode;
      }
      if (gauge.analysisMode != "classic_darkness" && gauge.analysisMode != "color_target" && gauge.analysisMode != "hybrid_pca_hough") {
        gauge.analysisMode = "color_target";
      }

      uint8_t cr = 0;
      uint8_t cg = 0;
      uint8_t cb = 0;
      if (!parseHexColor(gauge.needleColor, cr, cg, cb)) gauge.needleColor = legacyNeedle;
      if (!parseHexColor(gauge.backgroundColor, cr, cg, cb)) gauge.backgroundColor = legacyBackground;
      if (!parseHexColor(gauge.textColor, cr, cg, cb)) gauge.textColor = legacyText;

      if (!parseHexColor(gauge.needleColor, cr, cg, cb)) gauge.needleColor = "#D00000";
      if (!parseHexColor(gauge.backgroundColor, cr, cg, cb)) gauge.backgroundColor = "#7F7F7F";
      if (!parseHexColor(gauge.textColor, cr, cg, cb)) gauge.textColor = "#101010";

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

void applyRuntimeSettings(const DeviceConfig &config) {
  gFlashEnabled = config.flashEnabled;

  AnalysisInputMode parsedMode = AnalysisInputMode::Auto;
  if (parseAnalysisInputMode(config.analysisInputSource, parsedMode)) {
    gAnalysisInputMode = parsedMode;
  } else {
    gAnalysisInputMode = AnalysisInputMode::Auto;
  }

  int intervalS = config.intervalS;
  if (intervalS < static_cast<int>(kMinAnalysisIntervalMs / 1000)) intervalS = static_cast<int>(kMinAnalysisIntervalMs / 1000);
  if (intervalS > static_cast<int>(kMaxAnalysisIntervalMs / 1000)) intervalS = static_cast<int>(kMaxAnalysisIntervalMs / 1000);
  gAnalysisIntervalMs = static_cast<uint32_t>(intervalS) * 1000UL;
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

void rgb565ToRgb(uint16_t pixel, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = ((pixel >> 11) & 0x1F) * 255 / 31;
  g = ((pixel >> 5) & 0x3F) * 255 / 63;
  b = (pixel & 0x1F) * 255 / 31;
}

bool sampleGray(const camera_fb_t *fb, int x, int y, uint8_t &gray) {
  if (!fb || fb->format != PIXFORMAT_RGB565) return false;
  if (x < 0 || y < 0 || x >= fb->width || y >= fb->height) return false;

  const uint16_t *pixels = reinterpret_cast<const uint16_t *>(fb->buf);
  gray = rgb565ToGray(pixels[(y * fb->width) + x]);
  return true;
}

bool sampleRgb(const camera_fb_t *fb, int x, int y, uint8_t &r, uint8_t &g, uint8_t &b, uint8_t &gray) {
  if (!fb || fb->format != PIXFORMAT_RGB565) return false;
  if (x < 0 || y < 0 || x >= fb->width || y >= fb->height) return false;

  const uint16_t *pixels = reinterpret_cast<const uint16_t *>(fb->buf);
  const uint16_t pixel = pixels[(y * fb->width) + x];
  rgb565ToRgb(pixel, r, g, b);
  gray = static_cast<uint8_t>((r * 30 + g * 59 + b * 11) / 100);
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

int16_t scaledToI16(float value, float scale) {
  const float scaled = value * scale;
  if (scaled > 32767.0f) return 32767;
  if (scaled < -32768.0f) return -32768;
  return static_cast<int16_t>(lroundf(scaled));
}

void setRegisterS16(uint16_t index, int16_t value) {
  if (index >= kModbusRegCount) return;
  modbusHolding[index] = static_cast<uint16_t>(value);
}

void setRegisterU16(uint16_t index, uint16_t value) {
  if (index >= kModbusRegCount) return;
  modbusHolding[index] = value;
}

uint16_t currentFaultCode() {
  uint16_t faultCode = 0;
  if (!gStorageReady) faultCode |= 0x01;
  if (!gCameraReady) faultCode |= 0x02;
  return faultCode;
}

uint16_t currentAnalysisSourceCode() {
  if (gLastAnalysisSource != AnalysisSourceCode::Unavailable) {
    return static_cast<uint16_t>(gLastAnalysisSource);
  }

  switch (gAnalysisInputMode) {
    case AnalysisInputMode::Camera:
      return gCameraReady ? static_cast<uint16_t>(AnalysisSourceCode::CameraLive) : static_cast<uint16_t>(AnalysisSourceCode::Unavailable);
    case AnalysisInputMode::SdPhoto:
      return (gStorageReady && SD_MMC.exists(kPhotoPath)) ? static_cast<uint16_t>(AnalysisSourceCode::SdPhoto) : static_cast<uint16_t>(AnalysisSourceCode::Unavailable);
    case AnalysisInputMode::Auto:
    default:
      if (gCameraReady) return static_cast<uint16_t>(AnalysisSourceCode::CameraLive);
      return (gStorageReady && SD_MMC.exists(kPhotoPath)) ? static_cast<uint16_t>(AnalysisSourceCode::SdPhoto) : static_cast<uint16_t>(AnalysisSourceCode::Unavailable);
  }
}

const char *currentAnalysisSourceName() {
  switch (static_cast<AnalysisSourceCode>(currentAnalysisSourceCode())) {
    case AnalysisSourceCode::CameraLive:
      return "camera_live";
    case AnalysisSourceCode::SdPhoto:
      return "sd_photo";
    default:
      return "unavailable";
  }
}

void updateSystemRegisters(uint32_t nowMs) {
  const uint32_t uptimeSeconds = nowMs / 1000;
  setRegisterU16(REG_UPTIME_LO, static_cast<uint16_t>(uptimeSeconds & 0xFFFF));
  setRegisterU16(REG_UPTIME_HI, static_cast<uint16_t>((uptimeSeconds >> 16) & 0xFFFF));

  if (WiFi.status() == WL_CONNECTED) {
    setRegisterS16(REG_WIFI_RSSI, static_cast<int16_t>(WiFi.RSSI()));
  } else {
    setRegisterS16(REG_WIFI_RSSI, -32768);
  }

  setRegisterU16(REG_FAULT_CODE, currentFaultCode());
  setRegisterU16(REG_ANALYSIS_SOURCE, currentAnalysisSourceCode());
}

void updateAnalysisRegisters(const GaugeReading readings[]) {
  const bool g1Detected = readings[0].detected;
  const bool g2Detected = readings[1].detected;

  uint16_t status = 0;
  if (!g1Detected && !g2Detected) status = 13;
  else if (!g1Detected) status = 11;
  else if (!g2Detected) status = 12;
  setRegisterU16(REG_STATUS, status);

  setRegisterS16(REG_G1_X100, g1Detected ? scaledToI16(readings[0].value, 100.0f) : -32768);
  setRegisterS16(REG_G2_X100, g2Detected ? scaledToI16(readings[1].value, 100.0f) : -32768);
  setRegisterS16(REG_DIFF_X100, (g1Detected && g2Detected) ? scaledToI16(readings[0].value - readings[1].value, 100.0f) : -32768);
  setRegisterU16(REG_CONF1_X1000, g1Detected ? static_cast<uint16_t>(lroundf(clamp01(readings[0].confidence) * 1000.0f)) : 0);
  setRegisterU16(REG_CONF2_X1000, g2Detected ? static_cast<uint16_t>(lroundf(clamp01(readings[1].confidence) * 1000.0f)) : 0);
}

float angleToValue(float angleDeg, const GaugeConfig &gauge) {
  const float span = gauge.angleMax - gauge.angleMin;
  if (fabsf(span) < 0.001f) return gauge.valueMin;
  const float ratio = clamp01((angleDeg - gauge.angleMin) / span);
  return gauge.valueMin + ratio * (gauge.valueMax - gauge.valueMin);
}

struct AngleScore {
  float redStrength = 0.0f;
  float grayLevel = 255.0f;
  float tipReach = 0.0f;
};

struct AnalysisColorProfile {
  uint8_t needleR = 208;
  uint8_t needleG = 0;
  uint8_t needleB = 0;
  uint8_t backgroundR = 127;
  uint8_t backgroundG = 127;
  uint8_t backgroundB = 127;
  uint8_t textR = 16;
  uint8_t textG = 16;
  uint8_t textB = 16;
};

AnalysisColorProfile buildColorProfile(const GaugeConfig &gauge) {
  AnalysisColorProfile profile;
  parseHexColor(gauge.needleColor, profile.needleR, profile.needleG, profile.needleB);
  parseHexColor(gauge.backgroundColor, profile.backgroundR, profile.backgroundG, profile.backgroundB);
  parseHexColor(gauge.textColor, profile.textR, profile.textG, profile.textB);
  return profile;
}

float colorDistanceSq(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2) {
  const float dr = static_cast<float>(r1) - r2;
  const float dg = static_cast<float>(g1) - g2;
  const float db = static_cast<float>(b1) - b2;
  return dr * dr + dg * dg + db * db;
}

float normalizeAngle180(float angleDeg) {
  while (angleDeg <= -180.0f) angleDeg += 360.0f;
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  return angleDeg;
}

float toGaugeAngleDeg(float angleRad) {
  return normalizeAngle180((angleRad * 180.0f / PI) + 90.0f);
}

float angleDistanceDeg(float a, float b) {
  return fabsf(normalizeAngle180(a - b));
}

float radialPriority(float radius, float startRadius, float endRadius) {
  const float span = max(1.0f, endRadius - startRadius);
  const float norm = clamp01((radius - startRadius) / span);
  const float normSq = norm * norm;
  // Strongly reduce influence near axle to avoid selecting short counterweight arm.
  return 0.15f + (0.85f * normSq);
}

bool angleWithinGaugeRange(float angleDeg, const GaugeConfig &gauge) {
  const float start = min(gauge.angleMin, gauge.angleMax) - kAngleSearchMarginDeg;
  const float end = max(gauge.angleMin, gauge.angleMax) + kAngleSearchMarginDeg;
  return angleDeg >= start && angleDeg <= end;
}

void expandedGaugeRange(const GaugeConfig &gauge, float &start, float &end) {
  start = min(gauge.angleMin, gauge.angleMax) - kAngleSearchMarginDeg;
  end = max(gauge.angleMin, gauge.angleMax) + kAngleSearchMarginDeg;
}

struct HybridLineScore {
  float score = 0.0f;
  float grayLevel = 255.0f;
  uint16_t hits = 0;
  uint16_t samples = 0;
  float weightedHitRatio = 0.0f;
  float tipReach = 0.0f;
};

HybridLineScore scoreNeedleLineHybrid(const camera_fb_t *fb, const GaugeConfig &gauge, float angleDeg, uint8_t darkThreshold) {
  HybridLineScore result;
  const float angleRad = angleToRadians(angleDeg);
  const float dirX = cosf(angleRad);
  const float dirY = sinf(angleRad);
  const float perpX = -dirY;
  const float perpY = dirX;
  const float startRadius = gauge.radius * 0.20f;
  const float endRadius = gauge.radius * 0.98f;
  const float step = max(1.0f, gauge.radius / 34.0f);

  uint32_t graySum = 0;
  float weightedHits = 0.0f;
  float weightedSamples = 0.0f;
  float maxHitRadius = 0.0f;
  uint16_t iter = 0;
  for (float radius = startRadius; radius <= endRadius; radius += step) {
    const float radiusWeight = radialPriority(radius, startRadius, endRadius);
    for (int offset = -1; offset <= 1; ++offset) {
      const int x = lroundf(gauge.cx + dirX * radius + perpX * offset);
      const int y = lroundf(gauge.cy + dirY * radius + perpY * offset);
      uint8_t gray = 0;
      if (!sampleGray(fb, x, y, gray)) continue;
      graySum += gray;
      ++result.samples;
      weightedSamples += radiusWeight;
      if (gray <= darkThreshold) {
        ++result.hits;
        weightedHits += radiusWeight;
        if (radius > maxHitRadius) maxHitRadius = radius;
      }
      ++iter;
      if ((iter % 220) == 0) {
        delay(0);
      }
    }
  }

  if (result.samples == 0) return result;

  result.grayLevel = static_cast<float>(graySum) / result.samples;
  const float hitRatio = static_cast<float>(result.hits) / result.samples;
  result.weightedHitRatio = (weightedSamples > 0.0f) ? (weightedHits / weightedSamples) : 0.0f;
  result.tipReach = clamp01((maxHitRadius - startRadius) / max(1.0f, endRadius - startRadius));
  const float darknessBoost = max(0.0f, static_cast<float>(darkThreshold) - result.grayLevel);
  result.score =
    (result.hits * 1.00f) +
    (hitRatio * 40.0f) +
    (result.weightedHitRatio * 95.0f) +
    (result.tipReach * 120.0f) +
    (darknessBoost * 0.14f);
  return result;
}

GaugeReading analyzeGaugeHybrid(const camera_fb_t *fb, const GaugeConfig &gauge) {
  GaugeReading reading;
  if (!gauge.valid || !fb || fb->format != PIXFORMAT_RGB565) return reading;

  float rangeStart = 0.0f;
  float rangeEnd = 0.0f;
  expandedGaugeRange(gauge, rangeStart, rangeEnd);
  resetGaugeDebug(gauge, rangeStart, rangeEnd, 0);
  GaugeAnalysisDebug *dbg = debugForGauge(gauge);

  const float ringInner = gauge.radius * 0.16f;
  const float ringOuter = gauge.radius * 1.00f;
  const float ringInner2 = ringInner * ringInner;
  const float ringOuter2 = ringOuter * ringOuter;

  // Pass 1: estimate adaptive darkness threshold from gauge annulus.
  uint32_t sumGray = 0;
  uint32_t sumSqGray = 0;
  uint16_t statsCount = 0;
  for (int y = gauge.cy - gauge.radius; y <= gauge.cy + gauge.radius; y += 2) {
    for (int x = gauge.cx - gauge.radius; x <= gauge.cx + gauge.radius; x += 2) {
      const float dx = static_cast<float>(x - gauge.cx);
      const float dy = static_cast<float>(y - gauge.cy);
      const float d2 = dx * dx + dy * dy;
      if (d2 < ringInner2 || d2 > ringOuter2) continue;

      uint8_t gray = 0;
      if (!sampleGray(fb, x, y, gray)) continue;
      sumGray += gray;
      sumSqGray += static_cast<uint32_t>(gray) * gray;
      ++statsCount;
    }
    delay(0);
  }

  if (statsCount < 90) return reading;

  const float meanGray = static_cast<float>(sumGray) / statsCount;
  float variance = (static_cast<float>(sumSqGray) / statsCount) - (meanGray * meanGray);
  if (variance < 0.0f) variance = 0.0f;
  const float stdDev = sqrtf(variance);
  float thresholdF = meanGray - (10.0f + stdDev * 0.55f);
  if (thresholdF < 35.0f) thresholdF = 35.0f;
  if (thresholdF > 185.0f) thresholdF = 185.0f;
  const uint8_t darkThreshold = static_cast<uint8_t>(lroundf(thresholdF));

  // Pass 2: PCA over dark pixels to estimate dominant needle axis.
  float sumW = 0.0f;
  float sxx = 0.0f;
  float sxy = 0.0f;
  float syy = 0.0f;
  uint16_t darkCount = 0;
  for (int y = gauge.cy - gauge.radius; y <= gauge.cy + gauge.radius; y += 2) {
    for (int x = gauge.cx - gauge.radius; x <= gauge.cx + gauge.radius; x += 2) {
      const float dx = static_cast<float>(x - gauge.cx);
      const float dy = static_cast<float>(y - gauge.cy);
      const float d2 = dx * dx + dy * dy;
      if (d2 < ringInner2 || d2 > ringOuter2) continue;

      uint8_t gray = 0;
      if (!sampleGray(fb, x, y, gray)) continue;
      if (gray > darkThreshold) continue;

      const float weight = static_cast<float>((darkThreshold - gray) + 1);
      sxx += dx * dx * weight;
      sxy += dx * dy * weight;
      syy += dy * dy * weight;
      sumW += weight;
      ++darkCount;
    }
    delay(0);
  }

  if (darkCount < 40 || sumW < 1.0f) return reading;

  const float covXX = sxx / sumW;
  const float covXY = sxy / sumW;
  const float covYY = syy / sumW;
  const float pcaAngleRad = 0.5f * atan2f(2.0f * covXY, covXX - covYY);
  const float pcaAngleA = toGaugeAngleDeg(pcaAngleRad);
  const float pcaAngleB = normalizeAngle180(pcaAngleA + 180.0f);

  // Anisotropy close to 1 means elongated shape (good pointer-like blob).
  const float trace = covXX + covYY;
  const float detPart = (covXX - covYY) * (covXX - covYY) + 4.0f * covXY * covXY;
  const float rootTerm = detPart > 0.0f ? sqrtf(detPart) : 0.0f;
  const float lambda1 = 0.5f * (trace + rootTerm);
  const float lambda2 = 0.5f * (trace - rootTerm);
  const float anisotropy = clamp01((lambda1 - lambda2) / max(1.0f, lambda1 + lambda2));

  HybridLineScore scoreA = scoreNeedleLineHybrid(fb, gauge, pcaAngleA, darkThreshold);
  HybridLineScore scoreB = scoreNeedleLineHybrid(fb, gauge, pcaAngleB, darkThreshold);
  if (dbg) dbg->pointerCandidates = 2;

  float seedAngle = pcaAngleA;
  HybridLineScore seedScore = scoreA;
  const bool inRangeA = angleWithinGaugeRange(pcaAngleA, gauge);
  const bool inRangeB = angleWithinGaugeRange(pcaAngleB, gauge);
  const float orientA = scoreA.score + (scoreA.tipReach * 70.0f) + (scoreA.weightedHitRatio * 45.0f);
  const float orientB = scoreB.score + (scoreB.tipReach * 70.0f) + (scoreB.weightedHitRatio * 45.0f);
  if (!inRangeA && inRangeB) {
    seedAngle = pcaAngleB;
    seedScore = scoreB;
  } else if (inRangeA == inRangeB && orientB > orientA) {
    seedAngle = pcaAngleB;
    seedScore = scoreB;
  }

  // Pass 3: mini-Hough around PCA estimate for robust angle refinement.
  constexpr float kRefineWindowDeg = 18.0f;
  constexpr float kRefineStepDeg = 1.0f;
  constexpr int kMaxRefine = 41;
  float refineAngles[kMaxRefine];
  HybridLineScore refineScores[kMaxRefine];
  int refineCount = 0;

  for (float delta = -kRefineWindowDeg; delta <= kRefineWindowDeg && refineCount < kMaxRefine; delta += kRefineStepDeg) {
    const float candidate = normalizeAngle180(seedAngle + delta);
    refineAngles[refineCount] = candidate;
    refineScores[refineCount] = scoreNeedleLineHybrid(fb, gauge, candidate, darkThreshold);
    ++refineCount;
  }

  if (refineCount == 0) return reading;
  if (dbg) dbg->scannedAngles = refineCount;

  int bestIndex = 0;
  for (int i = 1; i < refineCount; ++i) {
    const float candidate = refineScores[i].score + (refineScores[i].tipReach * 55.0f) + (refineScores[i].weightedHitRatio * 25.0f);
    const float current = refineScores[bestIndex].score + (refineScores[bestIndex].tipReach * 55.0f) + (refineScores[bestIndex].weightedHitRatio * 25.0f);
    if (candidate > current) bestIndex = i;
  }

  int secondIndex = -1;
  for (int i = 0; i < refineCount; ++i) {
    if (i == bestIndex) continue;
    if (angleDistanceDeg(refineAngles[i], refineAngles[bestIndex]) < 3.1f) continue;
    if (secondIndex < 0 || refineScores[i].score > refineScores[secondIndex].score) secondIndex = i;
  }

  const float bestScore = refineScores[bestIndex].score;
  const float secondScore = secondIndex >= 0 ? refineScores[secondIndex].score : seedScore.score;
  const float contrast = max(0.0f, bestScore - secondScore);
  const float hitRatio = (refineScores[bestIndex].samples > 0)
    ? (static_cast<float>(refineScores[bestIndex].hits) / refineScores[bestIndex].samples)
    : 0.0f;
  const float weightedHitRatio = refineScores[bestIndex].weightedHitRatio;
  const float tipReach = refineScores[bestIndex].tipReach;

  float confidence = 0.0f;
  confidence += 0.30f * anisotropy;
  confidence += 0.28f * clamp01(contrast / 42.0f);
  confidence += 0.20f * clamp01((weightedHitRatio - 0.12f) / 0.58f);
  confidence += 0.22f * tipReach;
  confidence = clamp01(confidence);

  const bool insideRange = angleWithinGaugeRange(refineAngles[bestIndex], gauge);
  const float oppositeAngle = normalizeAngle180(refineAngles[bestIndex] + 180.0f);
  const HybridLineScore oppositeScore = scoreNeedleLineHybrid(fb, gauge, oppositeAngle, darkThreshold);
  const bool oppositeDominates = (oppositeScore.tipReach > (tipReach + 0.12f)) && (oppositeScore.score >= (bestScore * 0.90f));
  if (dbg) {
    dbg->selectedAngleDeg = refineAngles[bestIndex];
    dbg->selectedScore = bestScore;
    dbg->selectedTipReach = tipReach;
    dbg->oppositeAngleDeg = oppositeAngle;
    dbg->oppositeScore = oppositeScore.score;
    dbg->oppositeTipReach = oppositeScore.tipReach;
    dbg->oppositeDominates = oppositeDominates;
  }
  if (!insideRange || refineScores[bestIndex].hits < 10 || weightedHitRatio < 0.15f || tipReach < 0.42f || confidence < 0.10f || oppositeDominates) {
    if (dbg) {
      if (!insideRange) dbg->selectedReason = "rejected_out_of_range";
      else if (oppositeDominates) dbg->selectedReason = "rejected_opposite_dominates";
      else if (tipReach < 0.42f) dbg->selectedReason = "rejected_short_tip_reach";
      else if (weightedHitRatio < 0.15f) dbg->selectedReason = "rejected_low_hit_ratio";
      else dbg->selectedReason = "rejected_low_confidence";
    }
    return reading;
  }

  reading.detected = true;
  reading.angleDeg = refineAngles[bestIndex];
  reading.value = angleToValue(reading.angleDeg, gauge);
  reading.confidence = confidence;
  reading.darkness = refineScores[bestIndex].grayLevel;
  if (dbg) dbg->selectedReason = "selected_best_hybrid_score";
  return reading;
}

float scoreClassicDarkness(const camera_fb_t *fb, const GaugeConfig &gauge, float angleDeg, float &grayLevel, float &tipReachOut) {
  const float angleRad = angleToRadians(angleDeg);
  const float dirX = cosf(angleRad);
  const float dirY = sinf(angleRad);
  const float perpX = -dirY;
  const float perpY = dirX;
  const float startRadius = gauge.radius * 0.22f;
  const float endRadius = gauge.radius * 0.95f;
  const float step = max(1.0f, gauge.radius / 28.0f);

  uint32_t sum = 0;
  float weightedDarknessSum = 0.0f;
  float weightSum = 0.0f;
  float maxDarkRadius = 0.0f;
  uint16_t samples = 0;
  for (float radius = startRadius; radius <= endRadius; radius += step) {
    const float radiusWeight = radialPriority(radius, startRadius, endRadius);
    for (int offset = -2; offset <= 2; ++offset) {
      const int x = lroundf(gauge.cx + dirX * radius + perpX * offset);
      const int y = lroundf(gauge.cy + dirY * radius + perpY * offset);
      uint8_t gray = 0;
      if (!sampleGray(fb, x, y, gray)) continue;
      sum += gray;
      weightedDarknessSum += (255.0f - gray) * radiusWeight;
      weightSum += radiusWeight;
      if (gray <= 170 && radius > maxDarkRadius) maxDarkRadius = radius;
      ++samples;
    }
    delay(0);
  }

  grayLevel = samples > 0 ? (static_cast<float>(sum) / samples) : 255.0f;
  if (weightSum <= 0.0f) return 0.0f;

  const float weightedDarkness = weightedDarknessSum / weightSum;
  const float tipReach = clamp01((maxDarkRadius - startRadius) / max(1.0f, endRadius - startRadius));
  tipReachOut = tipReach;
  // Higher is better for unified selection logic; tipReach helps reject short thick counterweight.
  return weightedDarkness + (tipReach * 36.0f);
}

AngleScore scoreNeedleAngleColor(const camera_fb_t *fb, const GaugeConfig &gauge, const AnalysisColorProfile &profile, float angleDeg) {
  const float angleRad = angleToRadians(angleDeg);
  const float dirX = cosf(angleRad);
  const float dirY = sinf(angleRad);
  const float perpX = -dirY;
  const float perpY = dirX;
  const float startRadius = gauge.radius * 0.28f;
  const float endRadius = gauge.radius * 0.92f;
  const float step = max(1.0f, gauge.radius / 28.0f);

  AngleScore score;
  uint32_t graySum = 0;
  float redSum = 0.0f;
  float weightedHits = 0.0f;
  float totalWeight = 0.0f;
  float maxNeedleRadius = 0.0f;
  uint16_t redHits = 0;
  uint16_t samples = 0;
  uint16_t iter = 0;
  for (float radius = startRadius; radius <= endRadius; radius += step) {
    const float radiusWeight = radialPriority(radius, startRadius, endRadius);
    for (int offset = -1; offset <= 1; ++offset) {
      const int x = lroundf(gauge.cx + dirX * radius + perpX * offset);
      const int y = lroundf(gauge.cy + dirY * radius + perpY * offset);
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 0;
      uint8_t gray = 0;
      if (!sampleRgb(fb, x, y, r, g, b, gray)) continue;

      const float dNeedle = colorDistanceSq(r, g, b, profile.needleR, profile.needleG, profile.needleB);
      const float dBg = colorDistanceSq(r, g, b, profile.backgroundR, profile.backgroundG, profile.backgroundB);
      const float dText = colorDistanceSq(r, g, b, profile.textR, profile.textG, profile.textB);

      // Reward pixels near target needle color and far from background/text colors.
      const float proximityNeedle = max(0.0f, 255.0f - sqrtf(dNeedle));
      const float awayFromBg = min(255.0f, sqrtf(dBg));
      const float awayFromText = min(255.0f, sqrtf(dText));
      const float weighted = (proximityNeedle * 1.6f) + (awayFromBg * 0.5f) + (awayFromText * 0.5f);

      const bool looksNeedle = (proximityNeedle >= 28.0f) && (dNeedle < dBg) && (dNeedle < dText);
      if (looksNeedle) {
        redSum += weighted * radiusWeight;
        weightedHits += radiusWeight;
        if (radius > maxNeedleRadius) maxNeedleRadius = radius;
        ++redHits;
      }

      totalWeight += radiusWeight;
      graySum += gray;
      ++samples;
      ++iter;
      if ((iter % 200) == 0) {
        delay(0);
      }
    }
  }

  if (samples == 0) return score;

  score.grayLevel = static_cast<float>(graySum) / samples;
  if (redHits == 0) {
    score.redStrength = 0.0f;
    return score;
  }

  const float hitRatio = (totalWeight > 0.0f) ? (weightedHits / totalWeight) : 0.0f;
  score.tipReach = clamp01((maxNeedleRadius - startRadius) / max(1.0f, endRadius - startRadius));
  score.redStrength = (redSum / max(0.001f, weightedHits)) + (hitRatio * 90.0f) + (score.tipReach * 44.0f);
  return score;
}

GaugeReading analyzeGaugeClassic(const camera_fb_t *fb, const GaugeConfig &gauge) {
  GaugeReading reading;
  if (!gauge.valid || !fb || fb->format != PIXFORMAT_RGB565) return reading;

  constexpr float kAngleStep = 2.0f;
  constexpr int kMaxSamples = 181;
  float scores[kMaxSamples];
  float grayLevels[kMaxSamples];
  float tipReaches[kMaxSamples];
  float angles[kMaxSamples];
  int count = 0;

  float start = 0.0f;
  float end = 0.0f;
  expandedGaugeRange(gauge, start, end);
  resetGaugeDebug(gauge, start, end, 0);
  GaugeAnalysisDebug *dbg = debugForGauge(gauge);
  for (float angle = start; angle <= end && count < kMaxSamples; angle += kAngleStep) {
    angles[count] = angle;
    float gray = 255.0f;
    float tipReach = 0.0f;
    scores[count] = scoreClassicDarkness(fb, gauge, angle, gray, tipReach);
    grayLevels[count] = gray;
    tipReaches[count] = tipReach;
    ++count;
    if ((count % 12) == 0) {
      delay(0);
    }
  }

  if (count == 0) return reading;
  if (dbg) dbg->scannedAngles = count;

  int bestIndex = 0;
  for (int i = 1; i < count; ++i) {
    if (scores[i] > scores[bestIndex]) bestIndex = i;
  }

  int secondIndex = -1;
  for (int i = 0; i < count; ++i) {
    if (abs(i - bestIndex) <= 2) continue;
    if (secondIndex < 0 || scores[i] > scores[secondIndex]) secondIndex = i;
  }

  const float bestScore = scores[bestIndex];
  const float secondScore = secondIndex >= 0 ? scores[secondIndex] : 0.0f;
  const float contrast = max(0.0f, bestScore - secondScore);
  const float confidence = clamp01((contrast / 45.0f) + (0.28f * tipReaches[bestIndex]));

  const float oppositeAngle = normalizeAngle180(angles[bestIndex] + 180.0f);
  float oppositeGray = 255.0f;
  float oppositeTip = 0.0f;
  const float oppositeScore = scoreClassicDarkness(fb, gauge, oppositeAngle, oppositeGray, oppositeTip);
  const bool oppositeDominates = (oppositeTip > (tipReaches[bestIndex] + 0.12f)) && (oppositeScore >= (bestScore * 0.90f));
  if (dbg) {
    dbg->pointerCandidates = (oppositeScore > 1.0f || oppositeTip > 0.12f) ? 2 : 1;
    dbg->selectedAngleDeg = angles[bestIndex];
    dbg->selectedScore = bestScore;
    dbg->selectedTipReach = tipReaches[bestIndex];
    dbg->oppositeAngleDeg = oppositeAngle;
    dbg->oppositeScore = oppositeScore;
    dbg->oppositeTipReach = oppositeTip;
    dbg->oppositeDominates = oppositeDominates;
  }

  if (grayLevels[bestIndex] > 220.0f || confidence < 0.08f || oppositeDominates) {
    if (dbg) {
      if (oppositeDominates) dbg->selectedReason = "rejected_opposite_dominates";
      else if (grayLevels[bestIndex] > 220.0f) dbg->selectedReason = "rejected_too_bright";
      else dbg->selectedReason = "rejected_low_confidence";
    }
    return reading;
  }

  reading.detected = true;
  reading.angleDeg = angles[bestIndex];
  reading.value = angleToValue(reading.angleDeg, gauge);
  reading.confidence = confidence;
  reading.darkness = grayLevels[bestIndex];
  if (dbg) dbg->selectedReason = "selected_best_darkness_score";
  return reading;
}

GaugeReading analyzeGaugeColor(const camera_fb_t *fb, const GaugeConfig &gauge, const AnalysisColorProfile &profile) {
  GaugeReading reading;
  if (!gauge.valid || !fb || fb->format != PIXFORMAT_RGB565) return reading;

  constexpr float kAngleStep = 2.0f;
  constexpr int kMaxSamples = 181;
  float scores[kMaxSamples];
  float grayLevels[kMaxSamples];
  float tipReaches[kMaxSamples];
  float angles[kMaxSamples];
  int count = 0;

  float start = 0.0f;
  float end = 0.0f;
  expandedGaugeRange(gauge, start, end);
  resetGaugeDebug(gauge, start, end, 0);
  GaugeAnalysisDebug *dbg = debugForGauge(gauge);
  for (float angle = start; angle <= end && count < kMaxSamples; angle += kAngleStep) {
    angles[count] = angle;
    const AngleScore score = scoreNeedleAngleColor(fb, gauge, profile, angle);
    scores[count] = score.redStrength;
    grayLevels[count] = score.grayLevel;
    tipReaches[count] = score.tipReach;
    ++count;
    if ((count % 12) == 0) {
      delay(0);
    }
  }

  if (count == 0) return reading;
  if (dbg) dbg->scannedAngles = count;

  int bestIndex = 0;
  for (int i = 1; i < count; ++i) {
    if (scores[i] > scores[bestIndex]) bestIndex = i;
  }

  int secondIndex = -1;
  for (int i = 0; i < count; ++i) {
    if (abs(i - bestIndex) <= 2) continue;
    if (secondIndex < 0 || scores[i] > scores[secondIndex]) secondIndex = i;
  }

  const float bestScore = scores[bestIndex];
  const float secondScore = secondIndex >= 0 ? scores[secondIndex] : 0.0f;
  const float contrast = max(0.0f, bestScore - secondScore);
  const float confidence = clamp01((contrast / 46.0f) + (0.35f * tipReaches[bestIndex]));

  const float oppositeAngle = normalizeAngle180(angles[bestIndex] + 180.0f);
  const AngleScore opposite = scoreNeedleAngleColor(fb, gauge, profile, oppositeAngle);
  const bool oppositeDominates = (opposite.tipReach > (tipReaches[bestIndex] + 0.12f)) && (opposite.redStrength >= (bestScore * 0.90f));
  if (dbg) {
    dbg->pointerCandidates = (opposite.redStrength > 8.0f || opposite.tipReach > 0.12f) ? 2 : 1;
    dbg->selectedAngleDeg = angles[bestIndex];
    dbg->selectedScore = bestScore;
    dbg->selectedTipReach = tipReaches[bestIndex];
    dbg->oppositeAngleDeg = oppositeAngle;
    dbg->oppositeScore = opposite.redStrength;
    dbg->oppositeTipReach = opposite.tipReach;
    dbg->oppositeDominates = oppositeDominates;
  }

  if (bestScore < 14.0f || confidence < 0.05f || oppositeDominates) {
    if (dbg) {
      if (oppositeDominates) dbg->selectedReason = "rejected_opposite_dominates";
      else if (bestScore < 14.0f) dbg->selectedReason = "rejected_low_color_score";
      else dbg->selectedReason = "rejected_low_confidence";
    }
    return reading;
  }

  reading.detected = true;
  reading.angleDeg = angles[bestIndex];
  reading.value = angleToValue(reading.angleDeg, gauge);
  reading.confidence = confidence;
  reading.darkness = grayLevels[bestIndex];
  if (dbg) dbg->selectedReason = "selected_best_color_score";
  return reading;
}

GaugeReading analyzeGauge(const camera_fb_t *fb, const GaugeConfig &gauge) {
  if (gauge.analysisMode == "classic_darkness") {
    return analyzeGaugeClassic(fb, gauge);
  }
  if (gauge.analysisMode == "hybrid_pca_hough") {
    return analyzeGaugeHybrid(fb, gauge);
  }
  const AnalysisColorProfile profile = buildColorProfile(gauge);
  return analyzeGaugeColor(fb, gauge, profile);
}

String buildAnalysisJson(const DeviceConfig &config, const GaugeReading readings[], int frameWidth, int frameHeight, uint32_t elapsedMs) {
  const bool g1 = readings[0].detected;
  const bool g2 = readings[1].detected;
  String status = "ok";
  if (!g1 && !g2) status = "both_not_detected";
  else if (!g1) status = "gauge_1_not_detected";
  else if (!g2) status = "gauge_2_not_detected";

  int detectedGaugeCount = 0;
  int pointerCandidatesTotal = 0;
  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    if (readings[i].detected) ++detectedGaugeCount;
    pointerCandidatesTotal += gLastGaugeDebug[i].pointerCandidates;
  }

  String json = "{";
  json += "\"device_id\":\"" + jsonEscape(config.deviceId) + "\",";
  json += "\"status\":\"" + status + "\",";
  json += "\"analysis_input_mode\":\"" + String(analysisInputModeName(gAnalysisInputMode)) + "\",";
  json += "\"source\":\"" + String(currentAnalysisSourceName()) + "\",";
  json += "\"frame\":{\"width\":" + String(frameWidth) + ",\"height\":" + String(frameHeight) + "},";
  json += "\"debug\":{\"detected_gauges\":" + String(detectedGaugeCount) + ",\"pointer_candidates_total\":" + String(pointerCandidatesTotal) + "},";
  json += "\"processing_ms\":" + String(elapsedMs) + ',';
  json += "\"gauges\":[";

  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    if (i > 0) json += ',';
    json += '{';
    json += "\"id\":" + String(config.gauges[i].id) + ',';
    json += "\"name\":\"" + jsonEscape(config.gauges[i].name) + "\",";
    json += "\"unit\":\"" + jsonEscape(config.gauges[i].unit) + "\",";
    json += "\"analysis_mode\":\"" + jsonEscape(config.gauges[i].analysisMode) + "\",";
    json += "\"detected\":";
    json += readings[i].detected ? "true" : "false";
    if (readings[i].detected) {
      json += ",\"angle_deg\":" + String(readings[i].angleDeg, 2);
      json += ",\"value\":" + String(readings[i].value, 3);
      json += ",\"confidence\":" + String(readings[i].confidence, 3);
      json += ",\"darkness\":" + String(readings[i].darkness, 1);
    }
    const GaugeAnalysisDebug &dbg = gLastGaugeDebug[i];
    if (dbg.enabled) {
      json += ",\"debug\":{";
      json += "\"search_start_deg\":" + String(dbg.searchStartDeg, 2) + ',';
      json += "\"search_end_deg\":" + String(dbg.searchEndDeg, 2) + ',';
      json += "\"scanned_angles\":" + String(dbg.scannedAngles) + ',';
      json += "\"pointer_candidates\":" + String(dbg.pointerCandidates) + ',';
      json += "\"selected_angle_deg\":" + String(dbg.selectedAngleDeg, 2) + ',';
      json += "\"selected_score\":" + String(dbg.selectedScore, 3) + ',';
      json += "\"selected_tip_reach\":" + String(dbg.selectedTipReach, 3) + ',';
      json += "\"opposite_angle_deg\":" + String(dbg.oppositeAngleDeg, 2) + ',';
      json += "\"opposite_score\":" + String(dbg.oppositeScore, 3) + ',';
      json += "\"opposite_tip_reach\":" + String(dbg.oppositeTipReach, 3) + ',';
      json += "\"opposite_dominates\":" + String(dbg.oppositeDominates ? "true" : "false") + ',';
      json += "\"selected_reason\":\"" + jsonEscape(dbg.selectedReason.length() ? dbg.selectedReason : String("not_set")) + "\"";
      json += '}';
    }
    json += '}';
  }

  json += "]}";
  return json;
}

bool loadAnalysisFrameFromSd(AnalysisFrame &frame, String &error) {
  if (!gStorageReady) {
    error = "storage_not_ready";
    return false;
  }
  if (!SD_MMC.exists(kPhotoPath)) {
    error = "photo_not_found";
    return false;
  }

  File f = SD_MMC.open(kPhotoPath, FILE_READ);
  if (!f) {
    error = "photo_open_failed";
    return false;
  }

  const size_t jpgLen = f.size();
  if (jpgLen == 0) {
    f.close();
    error = "photo_empty";
    return false;
  }

  uint8_t *jpgBuf = static_cast<uint8_t *>(ps_malloc(jpgLen));
  if (!jpgBuf) {
    f.close();
    error = "jpg_alloc_failed";
    return false;
  }

  const size_t bytesRead = f.read(jpgBuf, jpgLen);
  f.close();
  if (bytesRead != jpgLen) {
    free(jpgBuf);
    error = "photo_read_failed";
    return false;
  }

  auto parseJpegDimensions = [](const uint8_t *data, size_t len, uint16_t &width, uint16_t &height) -> bool {
    if (!data || len < 4 || data[0] != 0xFF || data[1] != 0xD8) return false;

    size_t offset = 2;
    while (offset + 1 < len) {
      while (offset < len && data[offset] == 0xFF) {
        ++offset;
      }
      if (offset >= len) return false;

      const uint8_t marker = data[offset++];
      if (marker == 0xD9 || marker == 0xDA) return false;
      if (marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) continue;
      if (offset + 1 >= len) return false;

      const uint16_t segmentLen = static_cast<uint16_t>((data[offset] << 8) | data[offset + 1]);
      offset += 2;
      if (segmentLen < 2 || offset + segmentLen - 2 > len) return false;

      const bool isSofMarker =
        (marker >= 0xC0 && marker <= 0xC3) ||
        (marker >= 0xC5 && marker <= 0xC7) ||
        (marker >= 0xC9 && marker <= 0xCB) ||
        (marker >= 0xCD && marker <= 0xCF);
      if (isSofMarker) {
        if (segmentLen < 7) return false;
        height = static_cast<uint16_t>((data[offset + 1] << 8) | data[offset + 2]);
        width = static_cast<uint16_t>((data[offset + 3] << 8) | data[offset + 4]);
        return width > 0 && height > 0;
      }

      offset += segmentLen - 2;
    }

    return false;
  };

  uint16_t imageWidth = 0;
  uint16_t imageHeight = 0;
  if (!parseJpegDimensions(jpgBuf, jpgLen, imageWidth, imageHeight)) {
    free(jpgBuf);
    error = "photo_info_failed";
    return false;
  }

  const size_t rgb565Len = static_cast<size_t>(imageWidth) * static_cast<size_t>(imageHeight) * 2;
  uint8_t *rgbBuf = static_cast<uint8_t *>(ps_malloc(rgb565Len));
  if (!rgbBuf) {
    free(jpgBuf);
    error = "rgb_alloc_failed";
    return false;
  }

  if (!jpg2rgb565(jpgBuf, jpgLen, rgbBuf, JPG_SCALE_NONE)) {
    free(rgbBuf);
    free(jpgBuf);
    error = "photo_decode_failed";
    return false;
  }

  frame.ownedJpeg = jpgBuf;
  frame.ownedPixels = rgbBuf;
  frame.ownedFb.buf = rgbBuf;
  frame.ownedFb.len = rgb565Len;
  frame.ownedFb.width = imageWidth;
  frame.ownedFb.height = imageHeight;
  frame.ownedFb.format = PIXFORMAT_RGB565;
  frame.fb = &frame.ownedFb;
  return true;
}

bool loadAnalysisFrame(AnalysisFrame &frame, String &error) {
  gLastAnalysisSource = AnalysisSourceCode::Unavailable;

  auto tryCamera = [&]() -> bool {
    if (!gCameraReady) {
      error = "camera_not_ready";
      return false;
    }
    frame.fb = captureFrameWithFlash();
    if (!frame.fb) {
      error = "capture_failed";
      return false;
    }
    gLastAnalysisSource = AnalysisSourceCode::CameraLive;
    return true;
  };

  auto trySd = [&]() -> bool {
    if (!loadAnalysisFrameFromSd(frame, error)) {
      return false;
    }
    gLastAnalysisSource = AnalysisSourceCode::SdPhoto;
    return true;
  };

  switch (gAnalysisInputMode) {
    case AnalysisInputMode::Camera:
      return tryCamera();
    case AnalysisInputMode::SdPhoto:
      return trySd();
    case AnalysisInputMode::Auto:
    default:
      if (gCameraReady) {
        return tryCamera();
      }
      return trySd();
  }
}

void releaseAnalysisFrame(AnalysisFrame &frame) {
  if (frame.fb == &frame.ownedFb) {
    if (frame.ownedPixels) free(frame.ownedPixels);
    if (frame.ownedJpeg) free(frame.ownedJpeg);
  } else if (frame.fb) {
    esp_camera_fb_return(frame.fb);
  }
  frame = AnalysisFrame();
}

bool captureAndSave() {
  if (!gStorageReady) {
    Serial.println("[SD] Storage not ready");
    return false;
  }

  if (!gCameraReady) {
    if (SD_MMC.exists(kPhotoPath)) {
      Serial.println("[SD] Camera unavailable, keeping existing photo");
      return true;
    }
    Serial.println("[CAM] Camera not ready and no fallback photo on SD");
    return false;
  }

  camera_fb_t *fb = captureFrameWithFlash();
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
    server.send(gCameraReady ? 500 : 404, "text/plain", gCameraReady ? "capture_failed" : "photo_not_found");
    return;
  }
  server.send(200, "text/plain", "ok");
}

void handleAnalyze() {
  if (!checkAuth()) return;
  if (!gStorageReady) {
    server.send(503, "text/plain", "storage_not_ready");
    return;
  }

  DeviceConfig config;
  if (!loadDeviceConfig(config)) {
    server.send(409, "text/plain", "calibration_missing_or_invalid");
    return;
  }
  applyRuntimeSettings(config);

  const uint32_t start = millis();
  AnalysisFrame frame;
  String frameError;
  if (!loadAnalysisFrame(frame, frameError)) {
    server.send(frameError == "storage_not_ready" ? 503 : 500, "text/plain", frameError);
    return;
  }

  camera_fb_t *fb = frame.fb;

  if (fb->format != PIXFORMAT_RGB565) {
    releaseAnalysisFrame(frame);
    server.send(500, "text/plain", "unexpected_pixel_format");
    return;
  }

  GaugeReading readings[kGaugeCount];
  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    readings[i] = analyzeGauge(fb, config.gauges[i]);
  }
  updateAnalysisRegisters(readings);

  const int frameWidth = fb->width;
  const int frameHeight = fb->height;
  const bool sourceFromCamera = (frame.fb != &frame.ownedFb);
  const bool photoOk = sourceFromCamera ? saveFrameAsJpeg(fb, kPhotoPath) : true;
  const String response = buildAnalysisJson(config, readings, frameWidth, frameHeight, millis() - start);
  releaseAnalysisFrame(frame);

  if (!photoOk) {
    server.send(500, "text/plain", "photo_save_failed");
    return;
  }

  Serial.printf("[CV] Analysis done in %u ms  source=%s  mode=%s  frame=%dx%d\n",
    static_cast<unsigned>(millis() - start), currentAnalysisSourceName(), analysisInputModeName(gAnalysisInputMode), frameWidth, frameHeight);
  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    const GaugeReading &r = readings[i];
    const GaugeConfig  &g = config.gauges[i];
    if (r.detected) {
      Serial.printf("[CV] M%d %-16s  mode=%s  angle=%7.2f deg  value=%6.3f %s  conf=%.3f  dark=%.1f\n",
        g.id, g.name.c_str(), g.analysisMode.c_str(), r.angleDeg, r.value, g.unit.c_str(), r.confidence, r.darkness);
    } else {
      Serial.printf("[CV] M%d %-16s  mode=%s  NOT DETECTED\n", g.id, g.name.c_str(), g.analysisMode.c_str());
    }
  }
  server.send(200, "application/json", response);
}

void handlePhoto() {
  if (!checkAuth()) return;
  if (!gStorageReady) {
    server.send(503, "text/plain", "storage_not_ready");
    return;
  }
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
  if (!gStorageReady) {
    server.send(503, "text/plain", "storage_not_ready");
    return;
  }
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
  if (!gStorageReady) {
    server.send(503, "text/plain", "storage_not_ready");
    return;
  }
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
  DeviceConfig config;
  if (loadDeviceConfig(config)) {
    applyRuntimeSettings(config);
  }
  logEvent("CFG", "config.json saved");
  server.send(200, "text/plain", "ok");
}

void handleGetLogs() {
  if (!checkAuth()) return;
  if (!gStorageReady) {
    server.send(503, "text/plain", "storage_not_ready");
    return;
  }

  if (!SD_MMC.exists(kLogPath)) {
    server.send(200, "text/plain; charset=utf-8", "Brak wpisow logu\n");
    return;
  }

  const String logTail = sdReadTextTail(kLogPath, kWebLogTailBytes);
  server.send(200, "text/plain; charset=utf-8", logTail.length() ? logTail : String("Brak wpisow logu\n"));
}

void handleClearLogs() {
  if (!checkAuth()) return;
  if (!gStorageReady) {
    server.send(503, "text/plain", "storage_not_ready");
    return;
  }

  if (SD_MMC.exists(kLogPath) && !SD_MMC.remove(kLogPath)) {
    server.send(500, "text/plain", "clear_failed");
    return;
  }

  logEvent("LOG", "Log cleared from web UI");
  server.send(200, "text/plain", "ok");
}

void handleUploadBody() {
  HTTPUpload &upload = server.upload();
  if (!gStorageReady) return;
  if (upload.status == UPLOAD_FILE_START) {
    if (SD_MMC.exists(kPhotoPath)) SD_MMC.remove(kPhotoPath);
    gUploadFile = SD_MMC.open(kPhotoPath, FILE_WRITE);
    if (!gUploadFile) Serial.println("[UPLOAD] Cannot open file for write");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (gUploadFile) gUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (gUploadFile) gUploadFile.close();
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (gUploadFile) { gUploadFile.close(); SD_MMC.remove(kPhotoPath); }
  }
}

void handleUploadDone() {
  if (!checkAuth()) {
    if (gStorageReady) SD_MMC.remove(kPhotoPath);
    return;
  }
  if (!gStorageReady) {
    server.send(503, "text/plain", "storage_not_ready");
    return;
  }
  if (!SD_MMC.exists(kPhotoPath)) {
    server.send(500, "text/plain", "upload_failed");
    return;
  }
  logEvent("UPLOAD", String("JPEG uploaded -> ") + kPhotoPath);
  server.send(200, "text/plain", "ok");
}

String buildModbusStatusJson() {
  const bool wifiOk = (WiFi.status() == WL_CONNECTED);
  String json = "{";
  json += "\"ip\":\"" + String(wifiOk ? WiFi.localIP().toString() : "-") + "\",";
  json += "\"port\":502,";
  json += "\"unit_id\":" + String(kModbusUnitId) + ',';
  json += "\"function\":\"0x03\",";
  json += "\"wifi_status\":\"" + String(wifiOk ? "connected" : "disconnected") + "\",";
  json += "\"rssi_dbm\":" + String(static_cast<int16_t>(modbusHolding[REG_WIFI_RSSI])) + ',';
  json += "\"heartbeat\":" + String(modbusHolding[REG_HEARTBEAT]) + ',';
  json += "\"fault_code\":" + String(modbusHolding[REG_FAULT_CODE]) + ',';
  json += "\"analysis_input_mode\":\"" + String(analysisInputModeName(gAnalysisInputMode)) + "\",";
  json += "\"analysis_source\":\"" + String(currentAnalysisSourceName()) + "\",";
  json += "\"camera_ready\":" + String(gCameraReady ? "true" : "false") + ',';
  json += "\"storage_ready\":" + String(gStorageReady ? "true" : "false") + ',';
  json += "\"registers\":[";
  for (uint16_t i = 0; i < kModbusRegCount; ++i) {
    if (i > 0) json += ',';
    const uint16_t raw = modbusHolding[i];
    json += '{';
    json += "\"addr\":" + String(i) + ',';
    json += "\"reg\":" + String(40001 + i) + ',';
    json += "\"u16\":" + String(raw) + ',';
    json += "\"i16\":" + String(static_cast<int16_t>(raw)) + ',';
    json += "\"hex\":\"0x" + hex4(raw) + "\"";
    json += '}';
  }
  json += "]}";
  return json;
}

void handleGetModbusStatus() {
  if (!checkAuth()) return;
  server.send(200, "application/json", buildModbusStatusJson());
}

void writeModbusException(WiFiClient &client, const uint8_t *mbap, uint8_t unitId, uint8_t functionCode, uint8_t exceptionCode) {
  uint8_t response[9];
  response[0] = mbap[0];
  response[1] = mbap[1];
  response[2] = 0;
  response[3] = 0;
  response[4] = 0;
  response[5] = 3;  // unit + fc + exception
  response[6] = unitId;
  response[7] = static_cast<uint8_t>(functionCode | 0x80);
  response[8] = exceptionCode;
  client.write(response, sizeof(response));
}

void handleModbusTcp() {
  if (!modbusClient || !modbusClient.connected()) {
    if (modbusClient) modbusClient.stop();
    modbusClient = modbusServer.available();
    return;
  }

  if (modbusClient.available() < 8) return;

  uint8_t mbap[7];
  if (modbusClient.readBytes(mbap, sizeof(mbap)) != sizeof(mbap)) return;

  const uint16_t protocolId = static_cast<uint16_t>((mbap[2] << 8) | mbap[3]);
  const uint16_t length = static_cast<uint16_t>((mbap[4] << 8) | mbap[5]);
  const uint8_t unitId = mbap[6];

  if (protocolId != 0 || length < 2 || length > 253) {
    modbusClient.stop();
    return;
  }

  // Respond only to our configured Modbus Unit ID.
  if (unitId != kModbusUnitId) {
    return;
  }

  const uint16_t bodyLen = static_cast<uint16_t>(length - 1);
  const uint32_t waitStart = millis();
  while (modbusClient.available() < bodyLen) {
    if (millis() - waitStart > 100) return;
    delay(0);
  }

  uint8_t body[253];
  if (modbusClient.readBytes(body, bodyLen) != bodyLen) return;

  const uint8_t functionCode = body[0];
  if (functionCode != 0x03) {
    writeModbusException(modbusClient, mbap, unitId, functionCode, 0x01);  // Illegal Function
    return;
  }

  if (bodyLen < 5) {
    writeModbusException(modbusClient, mbap, unitId, functionCode, 0x03);  // Illegal Data Value
    return;
  }

  const uint16_t startAddr = static_cast<uint16_t>((body[1] << 8) | body[2]);
  const uint16_t quantity = static_cast<uint16_t>((body[3] << 8) | body[4]);

  // Accept both common client styles:
  // - raw zero-based holding address: 0..15
  // - 4xxxx-style numbers sent as start address: 40000..40015 or 40001..40016
  uint16_t normalizedStart = startAddr;
  if (startAddr >= 40001 && startAddr < static_cast<uint16_t>(40001 + kModbusRegCount)) {
    normalizedStart = static_cast<uint16_t>(startAddr - 40001);
  } else if (startAddr >= 40000 && startAddr < static_cast<uint16_t>(40000 + kModbusRegCount)) {
    normalizedStart = static_cast<uint16_t>(startAddr - 40000);
  }

  if (quantity == 0 || quantity > 125) {
    writeModbusException(modbusClient, mbap, unitId, functionCode, 0x03);
    return;
  }

  if (normalizedStart + quantity > kModbusRegCount) {
    writeModbusException(modbusClient, mbap, unitId, functionCode, 0x02);  // Illegal Data Address
    return;
  }

  const uint8_t byteCount = static_cast<uint8_t>(quantity * 2);
  const uint16_t respLength = static_cast<uint16_t>(3 + byteCount);  // unit + fc + byteCount + data

  uint8_t response[260];
  response[0] = mbap[0];
  response[1] = mbap[1];
  response[2] = 0;
  response[3] = 0;
  response[4] = static_cast<uint8_t>((respLength >> 8) & 0xFF);
  response[5] = static_cast<uint8_t>(respLength & 0xFF);
  response[6] = unitId;
  response[7] = functionCode;
  response[8] = byteCount;

  uint16_t out = 9;
  for (uint16_t i = 0; i < quantity; ++i) {
    const uint16_t reg = modbusHolding[normalizedStart + i];
    response[out++] = static_cast<uint8_t>((reg >> 8) & 0xFF);
    response[out++] = static_cast<uint8_t>(reg & 0xFF);
  }
  modbusClient.write(response, out);
}

void setupModbusTcp() {
  modbusServer.begin();
  modbusServer.setNoDelay(true);
  Serial.println("[MODBUS] TCP server started on port 502");
}

void setEmergencyMode(bool enabled, const char *reason) {
  gEmergencyMode = enabled;
  if (enabled) {
    gEmergencyBlinkState = EmergencyBlinkState::Idle;
    gEmergencyBlinkStateMs = millis();
    gEmergencyNextBlinkMs = millis();
    gEmergencyBlinkDoneCount = 0;
    logEvent("SAFE", String("Emergency mode enabled: ") + (reason ? reason : "unknown"));
  } else {
    gEmergencyBlinkState = EmergencyBlinkState::Idle;
    gEmergencyBlinkStateMs = millis();
    gEmergencyNextBlinkMs = 0;
    gEmergencyFault = EmergencyFault::None;
    gEmergencyBlinkDoneCount = 0;
    gEmergencyBlinkTargetCount = 2;
    logEvent("SAFE", "Emergency mode disabled");
  }
}

void setEmergencyFault(EmergencyFault fault) {
  gEmergencyFault = fault;
  switch (fault) {
    case EmergencyFault::Storage:
      gEmergencyBlinkTargetCount = 2;
      break;
    case EmergencyFault::Camera:
      gEmergencyBlinkTargetCount = 3;
      break;
    case EmergencyFault::StorageAndCamera:
      gEmergencyBlinkTargetCount = 4;
      break;
    case EmergencyFault::None:
    default:
      gEmergencyBlinkTargetCount = 2;
      break;
  }
}

void writeFlashLedRaw(bool enabled) {
  digitalWrite(FLASH_LED_GPIO_NUM, enabled ? HIGH : LOW);
}

void setFlashLed(bool enabled) {
  if (!gFlashEnabled) {
    writeFlashLedRaw(false);
    return;
  }
  writeFlashLedRaw(enabled);
}

void setupFlashLed() {
  pinMode(FLASH_LED_GPIO_NUM, OUTPUT);
  setFlashLed(false);
  Serial.println("[FLASH] LED ready");
}

void maintainEmergencyFlash(uint32_t nowMs) {
  if (!gEmergencyMode) {
    gEmergencyBlinkState = EmergencyBlinkState::Idle;
    writeFlashLedRaw(false);
    return;
  }

  switch (gEmergencyBlinkState) {
    case EmergencyBlinkState::Idle:
      if ((int32_t)(nowMs - gEmergencyNextBlinkMs) >= 0) {
        gEmergencyBlinkDoneCount = 0;
        writeFlashLedRaw(true);
        gEmergencyBlinkState = EmergencyBlinkState::On1;
        gEmergencyBlinkStateMs = nowMs;
      }
      break;

    case EmergencyBlinkState::On1:
      if (nowMs - gEmergencyBlinkStateMs >= kEmergencyBlinkPulseMs) {
        writeFlashLedRaw(false);
        gEmergencyBlinkState = EmergencyBlinkState::Off1;
        gEmergencyBlinkStateMs = nowMs;
      }
      break;

    case EmergencyBlinkState::Off1:
      if (nowMs - gEmergencyBlinkStateMs >= kEmergencyBlinkIntraPauseMs) {
        writeFlashLedRaw(true);
        gEmergencyBlinkState = EmergencyBlinkState::On2;
        gEmergencyBlinkStateMs = nowMs;
      }
      break;

    case EmergencyBlinkState::On2:
      if (nowMs - gEmergencyBlinkStateMs >= kEmergencyBlinkPulseMs) {
        writeFlashLedRaw(false);
        gEmergencyBlinkState = EmergencyBlinkState::Off2;
        gEmergencyBlinkStateMs = nowMs;
      }
      break;

    case EmergencyBlinkState::Off2:
      if (nowMs - gEmergencyBlinkStateMs >= kEmergencyBlinkIntraPauseMs) {
        ++gEmergencyBlinkDoneCount;
        if (gEmergencyBlinkDoneCount >= gEmergencyBlinkTargetCount) {
          gEmergencyBlinkState = EmergencyBlinkState::Idle;
          gEmergencyBlinkStateMs = nowMs;
          gEmergencyNextBlinkMs = nowMs + kEmergencyBlinkIntervalMs;
        } else {
          gEmergencyBlinkState = EmergencyBlinkState::BetweenSteps;
          gEmergencyBlinkStateMs = nowMs;
        }
      }
      break;

    case EmergencyBlinkState::BetweenSteps:
      if (nowMs - gEmergencyBlinkStateMs >= kEmergencyBlinkStepPauseMs) {
        writeFlashLedRaw(true);
        gEmergencyBlinkState = EmergencyBlinkState::On1;
        gEmergencyBlinkStateMs = nowMs;
      }
      break;
  }
}

camera_fb_t *captureFrameWithFlash() {
  if (!gCameraReady) return nullptr;
  setFlashLed(true);
  delay(kFlashWarmupMs);
  camera_fb_t *fb = esp_camera_fb_get();
  setFlashLed(false);
  return fb;
}

bool setupCamera() {
  camera_config_t config = {};
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

  // Fixed stable resolution requested by user: 1024x768.
  config.frame_size = FRAMESIZE_XGA;
  config.jpeg_quality = 16;
  config.fb_count = 1;

  const esp_err_t camErr = esp_camera_init(&config);

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
  server.on("/modbus/status", HTTP_GET, handleGetModbusStatus);
  server.on("/logs", HTTP_GET, handleGetLogs);
  server.on("/logs/clear", HTTP_POST, handleClearLogs);
  server.on("/upload", HTTP_POST, handleUploadDone, handleUploadBody);
  server.begin();
  logEvent("WEB", "Server started on port 80");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("[BOOT] ESP32-CAM Manometry v0.3");

  setupFlashLed();

  gStorageReady = setupStorage();
  if (!gStorageReady) {
    logEvent("BOOT", "Warning: storage setup failed -> safe mode path active");
  } else {
    logEvent("BOOT", "Storage setup OK");
  }

  gCameraReady = setupCamera();
  if (!gCameraReady) {
    logEvent("BOOT", "Warning: camera setup failed -> safe mode path active");
  } else {
    logEvent("BOOT", "Camera setup OK");
  }

  if (!gStorageReady) {
    setEmergencyFault(
      (!gStorageReady && !gCameraReady)
        ? EmergencyFault::StorageAndCamera
        : (!gStorageReady ? EmergencyFault::Storage : EmergencyFault::Camera)
    );
    setEmergencyMode(
      true,
      (!gStorageReady && !gCameraReady)
        ? "storage+camera init failed"
        : (!gStorageReady ? "storage init failed" : "camera init failed")
    );
  }

  setupWiFi();
  setupWebServer();
  setupModbusTcp();

  DeviceConfig config;
  if (loadDeviceConfig(config)) {
    applyRuntimeSettings(config);
    logEvent("CFG", String("analysis_input_source=") + analysisInputModeName(gAnalysisInputMode));
  } else {
    gAnalysisInputMode = AnalysisInputMode::Auto;
    logEvent("CFG", "No config found, using defaults (analysis_input_source=auto)");
  }

  if (!captureAndSave()) {
    logEvent("BOOT", "Initial capture/fallback photo check failed (continuing)");
  }
  logEvent("BOOT", "Setup completed");
}

static void runPeriodicAnalysis() {
  if (!gStorageReady) {
    Serial.println("[CV] Periodic: storage not ready, skipping");
    return;
  }

  DeviceConfig config;
  if (!loadDeviceConfig(config)) {
    Serial.println("[CV] Periodic: calibration missing, skipping");
    return;
  }
  applyRuntimeSettings(config);

  AnalysisFrame frame;
  String frameError;
  if (!loadAnalysisFrame(frame, frameError)) {
    Serial.printf("[CV] Periodic: source load failed: %s\n", frameError.c_str());
    return;
  }

  camera_fb_t *fb = frame.fb;

  if (fb->format != PIXFORMAT_RGB565) {
    releaseAnalysisFrame(frame);
    Serial.println("[CV] Periodic: unexpected pixel format");
    return;
  }

  const uint32_t start = millis();
  GaugeReading readings[kGaugeCount];
  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    readings[i] = analyzeGauge(fb, config.gauges[i]);
  }
  updateAnalysisRegisters(readings);

  const int frameWidth = fb->width;
  const int frameHeight = fb->height;
  if (frame.fb != &frame.ownedFb) {
    saveFrameAsJpeg(fb, kPhotoPath);
  }
  const uint32_t elapsed = millis() - start;
  releaseAnalysisFrame(frame);

  Serial.printf("[CV] Periodic done in %u ms  source=%s  mode=%s  frame=%dx%d\n",
    elapsed, currentAnalysisSourceName(), analysisInputModeName(gAnalysisInputMode), frameWidth, frameHeight);
  for (uint8_t i = 0; i < kGaugeCount; ++i) {
    const GaugeReading &r = readings[i];
    const GaugeConfig  &g = config.gauges[i];
    if (r.detected) {
      Serial.printf("[CV] M%d %-16s  mode=%s  angle=%7.2f deg  value=%6.3f %s  conf=%.3f  dark=%.1f\n",
        g.id, g.name.c_str(), g.analysisMode.c_str(), r.angleDeg, r.value, g.unit.c_str(), r.confidence, r.darkness);
    } else {
      Serial.printf("[CV] M%d %-16s  mode=%s  NOT DETECTED\n", g.id, g.name.c_str(), g.analysisMode.c_str());
    }
  }
}

static uint32_t lastAnalysisMs = 0;
static volatile bool analysisInProgress = false;
static const bool kEnablePeriodicAnalysis = true;
static const uint32_t kHeartbeatIntervalMs = 10000;
static uint32_t lastHeartbeatMs = 0;

void loop() {
  server.handleClient();
  handleModbusTcp();

  const uint32_t now = millis();
  maintainEmergencyFlash(now);
  updateSystemRegisters(now);
  maintainWiFi(now);

  if (now - lastHeartbeatMs >= kHeartbeatIntervalMs) {
    lastHeartbeatMs = now;
    setRegisterU16(REG_HEARTBEAT, static_cast<uint16_t>(modbusHolding[REG_HEARTBEAT] + 1));
    const bool wifiOk = (WiFi.status() == WL_CONNECTED);
    Serial.printf("[SYS] alive uptime=%lus wifi=%s ip=%s\n",
      static_cast<unsigned long>(now / 1000),
      wifiOk ? "ok" : "down",
      wifiOk ? WiFi.localIP().toString().c_str() : "-");
  }

  if (kEnablePeriodicAnalysis && !analysisInProgress && (now - lastAnalysisMs >= gAnalysisIntervalMs)) {
    lastAnalysisMs = now;
    analysisInProgress = true;
    runPeriodicAnalysis();
    analysisInProgress = false;
  }
}
