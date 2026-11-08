/*
 * ASEP MOCHI - AI buddy ala Xiaozhi untuk ESP32-S3 CAM
 * ----------------------------------------------------
 * Hardware (sesuai tabel pin):
 *   LCD 1.5" 240x240 ST7789 : SCL=19 SDA=20 RES=21 DC=47 BLK=38 (CS tidak dipakai)
 *   INMP441 (mic)           : WS=1 SCK=2 SD=42  (L/R ke GND)
 *   MAX98357A (speaker)     : LRC=41 BCLK=40 DIN=39
 *   Tombol BOOT (GPIO0)     : tekan sekali = foto soal, tahan 3 detik = reset WiFi
 *
 * Library (Library Manager):
 *   - LovyanGFX
 *   - ArduinoJson (v7)
 *   - ESP32 core 3.x (sudah termasuk ESP_I2S, esp_camera, WebServer, dll)
 *
 * Tools Arduino IDE:
 *   Board: ESP32S3 Dev Module | PSRAM: OPI PSRAM (N16R8) atau QSPI sesuai board
 *   Partition Scheme: Huge APP (3MB No OTA) | Flash Size sesuai board
 *
 * Cara pakai:
 *   - Pertama nyala: layar "STARTING..." 3 detik, lalu muka mochi.
 *   - Belum ada WiFi -> ESP bikin WiFi "Asep-Setup". Sambungkan HP, halaman
 *     setup muncul otomatis. Isi SSID + password.
 *   - Bilang "Halo Asep" -> Asep diam dan dengerin -> tanya apa aja.
 *   - Bilang "foto soal ..." / "lihat ini ..." atau tekan BOOT -> ambil foto & jawab.
 *   - "terjemahkan ... ke bahasa Inggris" -> Asep menerjemahkan.
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ESP_I2S.h>
#include <vector>
#include <math.h>
#include "esp_camera.h"
#include "mbedtls/base64.h"

// Enum harus di atas supaya dikenali prototipe otomatis Arduino
enum UiState { UI_WIFI, UI_PORTAL, UI_IDLE, UI_LISTEN, UI_THINK, UI_CAMERA, UI_ANSWER, UI_ERROR };
enum Mood { M_NORMAL, M_HAPPY, M_SAD, M_THINK, M_WIDE };

// ======================= KONFIGURASI =======================
#define OPENROUTER_API_KEY "PASTE_OPENROUTER_KEY_DISINI"
#define OPENROUTER_MODEL   "openrouter/free"

// Speech-to-text (Whisper kompatibel OpenAI). Default: Groq (ada free tier).
// Isi STT_API_KEY dengan key dari console.groq.com, atau ganti URL/model ke provider lain.
#define STT_API_KEY   ""
#define STT_URL       "https://api.groq.com/openai/v1/audio/transcriptions"
#define STT_MODEL     "whisper-large-v3-turbo"
#define STT_LANGUAGE  "id"      // kosongkan "" untuk auto-detect bahasa

// Text-to-speech: diisi nanti. Kosong = Asep bisu (cuma tampil teks di layar).
#define TTS_API_KEY   ""

#define AP_NAME       "Asep-Setup"

// ======================= PIN =======================
// LCD
#define PIN_LCD_SCL 19
#define PIN_LCD_SDA 20
#define PIN_LCD_RES 21
#define PIN_LCD_DC  47
#define PIN_LCD_BLK 38
// Mic INMP441
#define PIN_MIC_WS  1
#define PIN_MIC_SCK 2
#define PIN_MIC_SD  42
// Speaker MAX98357A
#define PIN_SPK_LRC  41
#define PIN_SPK_BCLK 40
#define PIN_SPK_DIN  39
// Tombol
#define PIN_BTN 0

// Kamera (pinout umum ESP32-S3-WROOM CAM / Freenove / OV2640). Ubah jika boardmu beda.
#define CAM_PIN_PWDN  -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK  15
#define CAM_PIN_SIOD  4
#define CAM_PIN_SIOC  5
#define CAM_PIN_D7    16
#define CAM_PIN_D6    17
#define CAM_PIN_D5    18
#define CAM_PIN_D4    12
#define CAM_PIN_D3    10
#define CAM_PIN_D2    8
#define CAM_PIN_D1    9
#define CAM_PIN_D0    11
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF  7
#define CAM_PIN_PCLK  13

// ======================= PARAMETER AUDIO =======================
#define SAMPLE_RATE      16000
#define REC_MAX_SECONDS  12
#define REC_MAX_SAMPLES  (SAMPLE_RATE * REC_MAX_SECONDS)
#define PREROLL_SAMPLES  4800      // 300 ms sebelum suara mulai
#define MIN_SPEECH_RMS   350.0f    // naikkan kalau ruangan berisik, turunkan kalau mic kurang peka
#define FOLLOWUP_MS      10000     // setelah menjawab, Asep tetap dengerin tanpa "Halo Asep"
#define BOOT_MS          3000

// ======================= LCD (LovyanGFX) =======================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
public:
  LGFX() {
    {
      auto cfg = _bus.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 3;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = true;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = PIN_LCD_SCL;
      cfg.pin_mosi = PIN_LCD_SDA;
      cfg.pin_miso = -1;
      cfg.pin_dc = PIN_LCD_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs = -1;
      cfg.pin_rst = PIN_LCD_RES;
      cfg.pin_busy = -1;
      cfg.panel_width = 240;
      cfg.panel_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.invert = true;       // kalau warna kebalik (negatif) ganti ke false
      cfg.rgb_order = false;   // kalau merah/biru tertukar ganti ke true
      cfg.readable = false;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    {
      auto cfg = _light.config();
      cfg.pin_bl = PIN_LCD_BLK;
      cfg.invert = false;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

LGFX tft;
LGFX_Sprite canvas(&tft);

static inline uint16_t C565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
uint16_t BG, INK, BLUSH, SOFT, WHITEC, PINKD;

// ======================= STATE GLOBAL =======================
volatile UiState ui = UI_WIFI;
SemaphoreHandle_t mtx;
uint32_t bootStart = 0;

// teks di layar (dilindungi mtx)
String uiStatus = "";
String uiQuestion = "";
std::vector<String> textLines;
bool bigFont = true;
uint32_t pageStart = 0, speakUntil = 0, uiExpire = 0;
bool portalActive = false;

volatile bool btnShort = false, btnLong = false;
volatile float micLevel = 0;
volatile uint32_t happyUntil = 0;
uint32_t activeUntil = 0;
float noiseFloor = 120;

int16_t* recBuf = nullptr;
bool camOK = false;

I2SClass i2sMic, i2sSpk;
Preferences prefs;
WebServer server(80);
DNSServer dns;
bool portalDone = false;
String scanOpts = "";

String hU[3], hA[3];
int hN = 0;

const char* SYSTEM_PROMPT =
  "Kamu adalah Asep, teman ngobrol dekat berwujud mochi imut di perangkat kecil ESP32. "
  "Gaya bicara santai, hangat, ramah, seperti sahabat. Jawab dengan bahasa yang dipakai pengguna (default Bahasa Indonesia). "
  "Untuk obrolan biasa jawab singkat 1-3 kalimat karena tampil di layar kecil. "
  "Jika pengguna minta menerjemahkan, langsung berikan terjemahannya saja (tambahkan cara baca jika berguna). "
  "Jika ada foto soal: baca soal dengan teliti, kerjakan SEMUA soal yang terlihat langkah demi langkah, "
  "periksa ulang hitungannya, lalu tulis jawaban akhir dengan jelas diawali 'Jawaban:'. Jika foto buram, bilang bagian mana yang tidak terbaca. "
  "Layar hanya bisa teks ASCII polos: JANGAN pakai markdown, emoji, LaTeX, atau simbol khusus. "
  "Tulis matematika dengan notasi teks biasa seperti x^2, sqrt(9), 3/4, dan tanda * untuk kali boleh diganti 'x'.";

// ======================= UTIL TEKS =======================
String toAscii(const String& in) {
  String out;
  out.reserve(in.length());
  size_t i = 0, n = in.length();
  while (i < n) {
    uint8_t c = (uint8_t)in[i];
    if (c < 0x80) {
      i++;
      if (c == '*' || c == '#' || c == '`' || c == '\r') continue;
      out += (char)c;
      continue;
    }
    uint32_t cp = 0;
    int len = 0;
    if ((c & 0xE0) == 0xC0) { cp = c & 0x1F; len = 2; }
    else if ((c & 0xF0) == 0xE0) { cp = c & 0x0F; len = 3; }
    else if ((c & 0xF8) == 0xF0) { cp = c & 0x07; len = 4; }
    else { i++; continue; }
    for (int k = 1; k < len && i + k < n; k++) cp = (cp << 6) | ((uint8_t)in[i + k] & 0x3F);
    i += len;
    switch (cp) {
      case 0x2018: case 0x2019: out += '\''; break;
      case 0x201C: case 0x201D: out += '"'; break;
      case 0x2013: case 0x2014: case 0x2212: out += '-'; break;
      case 0x2026: out += "..."; break;
      case 0xD7: out += 'x'; break;
      case 0xF7: out += '/'; break;
      case 0x2248: out += '~'; break;
      case 0x2192: out += "->"; break;
      case 0xB2: out += "^2"; break;
      case 0xB3: out += "^3"; break;
      case 0xB0: out += " derajat"; break;
      case 0x221A: out += "akar"; break;
      case 0x2264: out += "<="; break;
      case 0x2265: out += ">="; break;
      case 0x2260: out += "!="; break;
      case 0x3C0: out += "pi"; break;
      case 0xA0: out += ' '; break;
      default: break;
    }
  }
  return out;
}

void wrapText(const String& s, int cols, std::vector<String>& out) {
  out.clear();
  int n = s.length();
  int i = 0;
  while (i <= n) {
    int j = s.indexOf('\n', i);
    if (j < 0) j = n;
    String para = s.substring(i, j);
    para.trim();
    String line = "";
    int p = 0;
    while (p < (int)para.length()) {
      int q = para.indexOf(' ', p);
      if (q < 0) q = para.length();
      String w = para.substring(p, q);
      p = q + 1;
      if (w.length() == 0) continue;
      while ((int)w.length() > cols) {
        if (line.length()) { out.push_back(line); line = ""; }
        out.push_back(w.substring(0, cols));
        w = w.substring(cols);
      }
      if (line.length() == 0) line = w;
      else if ((int)(line.length() + 1 + w.length()) <= cols) line += " " + w;
      else { out.push_back(line); line = w; }
    }
    if (line.length()) out.push_back(line);
    i = j + 1;
  }
}

String jsonEscape(const String& s) {
  String o;
  o.reserve(s.length() + 16);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': break;
      case '\t': o += ' '; break;
      default: if ((uint8_t)c >= 0x20) o += c; break;
    }
  }
  return o;
}

String htmlEscape(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '<') o += "&lt;";
    else if (c == '>') o += "&gt;";
    else if (c == '&') o += "&amp;";
    else if (c == '"') o += "&quot;";
    else if (c == '\'') o += "&#39;";
    else o += c;
  }
  return o;
}

void showText(const String& question, const String& answer, bool isErr) {
  String a = toAscii(answer);
  a.trim();
  if (a.length() == 0) a = "...";
  std::vector<String> l2, l1;
  wrapText(a, 20, l2);
  bool big = l2.size() <= 16;
  if (!big) wrapText(a, 40, l1);
  String q = toAscii(question);
  q.trim();
  xSemaphoreTake(mtx, portMAX_DELAY);
  textLines = big ? l2 : l1;
  bigFont = big;
  uiQuestion = q;
  pageStart = millis();
  speakUntil = isErr ? 0 : millis() + min((uint32_t)(a.length() * 70), (uint32_t)15000);
  int rows = big ? 8 : 16;
  int pages = (textLines.size() + rows - 1) / rows;
  uint32_t pageMs = big ? 6000 : 9000;
  uiExpire = millis() + (isErr ? 7000 : (uint32_t)pages * pageMs * 2 + 6000);
  xSemaphoreGive(mtx);
  ui = isErr ? UI_ERROR : UI_ANSWER;
}

void setStatus(const String& s) {
  xSemaphoreTake(mtx, portMAX_DELAY);
  uiStatus = s;
  xSemaphoreGive(mtx);
}

// ======================= GAMBAR MUKA MOCHI =======================
void drawMochi(int cx, int cy, float s, Mood mood, bool blink, float mouth, int lx, int ly) {
  auto S = [&](float v) { return (int)(v * s + 0.5f); };
  // pipi
  for (int sd = -1; sd <= 1; sd += 2)
    canvas.fillEllipse(cx + sd * S(68), cy + S(26), max(2, S(13)), max(2, S(9)), BLUSH);
  // mata
  for (int sd = -1; sd <= 1; sd += 2) {
    int x = cx + sd * S(42) + lx, y = cy + ly;
    if (mood == M_HAPPY) {
      canvas.fillCircle(x, y + S(4), S(14), INK);
      canvas.fillCircle(x, y + S(12), S(13), BG);
    } else {
      int rx = max(2, S(10));
      int ry = (mood == M_WIDE) ? S(18) : (mood == M_SAD ? S(13) : S(15));
      if (blink) ry = max(2, S(2));
      canvas.fillEllipse(x, y, rx, ry, INK);
      if (!blink) {
        canvas.fillCircle(x - S(3), y - ry / 2, max(1, S(3.5f)), WHITEC);
        canvas.fillCircle(x + S(3), y + ry / 3, max(1, S(1.6f)), WHITEC);
      }
    }
  }
  // mulut
  int my = cy + S(30);
  if (mouth > 0.12f) {
    int R = max(2, S(4 + 8 * mouth));
    canvas.fillCircle(cx, my + R / 2, R, INK);
    canvas.fillEllipse(cx, my + R / 2 + (R * 6) / 10, max(1, (R * 6) / 10), max(1, (R * 3) / 10), BLUSH);
  } else if (mood == M_SAD) {
    for (int k = -1; k <= 1; k += 2) {
      canvas.fillCircle(cx + k * S(6), my + S(8), S(6), INK);
    }
    for (int k = -1; k <= 1; k += 2) {
      canvas.fillCircle(cx + k * S(6), my + S(11), S(6), BG);
    }
  } else if (mood == M_THINK) {
    canvas.fillRoundRect(cx - S(7), my + S(2), S(14), max(2, S(3)), 1, INK);
  } else {
    for (int k = -1; k <= 1; k += 2) canvas.fillCircle(cx + k * S(6), my, S(6), INK);
    for (int k = -1; k <= 1; k += 2) canvas.fillCircle(cx + k * S(6), my - S(3), S(6), BG);
  }
}

void txtC(const String& s, int y, int size, uint16_t col) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(size);
  canvas.setTextColor(col);
  canvas.setTextDatum(textdatum_t::top_center);
  canvas.drawString(s, 120, y);
}
void txtL(const String& s, int x, int y, int size, uint16_t col) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(size);
  canvas.setTextColor(col);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.drawString(s, x, y);
}

void render() {
  uint32_t now = millis();
  canvas.fillScreen(BG);
  canvas.drawRoundRect(1, 1, 238, 238, 34, BLUSH);
  canvas.drawRoundRect(2, 2, 236, 236, 33, BLUSH);
  canvas.fillCircle(222, 18, 4, WiFi.status() == WL_CONNECTED ? C565(120, 200, 140) : C565(230, 120, 120));

  if (now - bootStart < BOOT_MS) {
    txtC("STARTING...", 100, 3, INK);
    int w = (int)((now - bootStart) * 160UL / BOOT_MS);
    canvas.drawRoundRect(40, 150, 160, 10, 5, INK);
    canvas.fillRoundRect(40, 150, max(w, 1), 10, 5, BLUSH);
    canvas.pushSprite(0, 0);
    return;
  }

  static uint32_t nextBlinkAt = 0, blinkEnd = 0;
  if (now > nextBlinkAt) { blinkEnd = now + 140; nextBlinkAt = now + random(2500, 5500); }
  bool blink = now < blinkEnd;

  xSemaphoreTake(mtx, portMAX_DELAY);
  UiState st = ui;
  switch (st) {
    case UI_WIFI: {
      drawMochi(120, 95, 1.2f, M_NORMAL, blink, 0, 0, 0);
      txtC(uiStatus.length() ? uiStatus : String("Menyambung WiFi..."), 185, 2, INK);
    } break;

    case UI_PORTAL: {
      drawMochi(120, 40, 0.55f, M_NORMAL, blink, 0, 0, 0);
      txtC("Sambungkan HP ke WiFi:", 88, 1, INK);
      txtC(AP_NAME, 104, 3, PINKD);
      txtC("Halaman setup muncul", 145, 1, INK);
      txtC("otomatis, atau buka", 157, 1, INK);
      txtC("192.168.4.1", 169, 2, INK);
      if (uiStatus.length()) txtC(uiStatus, 205, 1, PINKD);
    } break;

    case UI_IDLE: {
      float bob = sinf(now / 500.0f) * 4;
      int lx = (int)(sinf(now / 1700.0f) * 5);
      Mood m = (now < happyUntil) ? M_HAPPY : M_NORMAL;
      drawMochi(120, 95 + (int)bob, 1.3f, m, blink, 0, lx, 0);
      txtC("Panggil: Halo Asep", 188, 2, INK);
      txtC("BOOT = foto soal", 218, 1, SOFT);
    } break;

    case UI_LISTEN: {
      Mood m = (now < happyUntil) ? M_HAPPY : M_WIDE;
      drawMochi(120, 90, 1.2f, m, false, 0.25f, 0, 0);
      txtC("Mendengarkan...", 172, 2, INK);
      canvas.drawRoundRect(30, 205, 180, 12, 6, INK);
      float ml = micLevel;
      if (ml > 1.0f) ml = 1.0f;
      int w = (int)(ml * 176);
      if (w > 1) canvas.fillRoundRect(32, 207, w, 8, 4, BLUSH);
    } break;

    case UI_THINK: {
      drawMochi(120, 85, 1.1f, M_THINK, blink, 0, 7, -7);
      String dots = "";
      for (int i = 0; i < (int)((now / 400) % 4); i++) dots += ".";
      txtC("Mikir dulu" + dots, 165, 2, INK);
      if (uiQuestion.length()) {
        String q = uiQuestion;
        if (q.length() > 36) q = q.substring(0, 33) + "...";
        txtC("\"" + q + "\"", 205, 1, SOFT);
      }
    } break;

    case UI_CAMERA: {
      drawMochi(120, 90, 1.2f, M_WIDE, false, 0.4f, 0, 0);
      txtC("Cekrek! Foto soal...", 175, 2, INK);
    } break;

    case UI_ANSWER:
    case UI_ERROR: {
      bool err = (st == UI_ERROR);
      float mouth = (!err && now < speakUntil) ? (0.5f + 0.5f * sinf(now / 110.0f)) : 0;
      drawMochi(120, 36, 0.5f, err ? M_SAD : M_NORMAL, blink, mouth, 0, 0);
      if (uiQuestion.length()) {
        String q = uiQuestion;
        if (q.length() > 36) q = q.substring(0, 33) + "...";
        txtC("> " + q, 80, 1, SOFT);
      }
      int rows = bigFont ? 8 : 16;
      int lh = bigFont ? 16 : 8;
      int total = textLines.size();
      int pages = max(1, (total + rows - 1) / rows);
      uint32_t pageMs = bigFont ? 6000 : 9000;
      int page = ((now - pageStart) / pageMs) % pages;
      for (int i = 0; i < rows; i++) {
        int li = page * rows + i;
        if (li >= total) break;
        txtL(textLines[li], 12, 94 + i * lh, bigFont ? 2 : 1, err ? PINKD : INK);
      }
      if (pages > 1) txtL(String(page + 1) + "/" + String(pages), 200, 228, 1, SOFT);
    } break;
  }
  xSemaphoreGive(mtx);
  canvas.pushSprite(0, 0);
}

void displayTask(void*) {
  static uint32_t pressAt = 0;
  static bool longFired = false;
  for (;;) {
    uint32_t now = millis();
    // tombol BOOT
    bool down = digitalRead(PIN_BTN) == LOW;
    if (down) {
      if (!pressAt) { pressAt = now; longFired = false; }
      else if (!longFired && now - pressAt >= 3000) { longFired = true; btnLong = true; }
    } else if (pressAt) {
      uint32_t d = now - pressAt;
      if (!longFired && d >= 40 && d < 3000) btnShort = true;
      pressAt = 0;
    }
    render();
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// ======================= WIFI + CAPTIVE PORTAL =======================
String portalPage(const String& err, const String& ssid) {
  String h = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
               "<meta name='viewport' content='width=device-width,initial-scale=1'>"
               "<title>Asep WiFi</title><style>"
               "body{margin:0;font-family:sans-serif;background:#fff1ee;color:#5a3a44;display:flex;justify-content:center}"
               ".c{width:100%;max-width:380px;padding:24px 20px}"
               ".f{font-size:34px;text-align:center;margin:8px 0}"
               "h2{text-align:center;margin:0 0 16px}"
               "label{display:block;margin:14px 0 4px;font-weight:bold;font-size:14px}"
               "input[type=text],input[type=password]{width:100%;box-sizing:border-box;padding:12px;border:2px solid #ffb3c1;border-radius:14px;font-size:16px}"
               ".s{font-weight:normal;font-size:13px}"
               "button{width:100%;margin-top:18px;padding:14px;border:0;border-radius:16px;background:#ff8fa3;color:#fff;font-size:17px;font-weight:bold}"
               ".e{background:#ffd9d9;color:#b00020;padding:10px;border-radius:12px;margin-bottom:10px;text-align:center;font-weight:bold}"
               "a{display:block;text-align:center;margin-top:14px;color:#c0577a}"
               "</style></head><body><div class='c'><div class='f'>( ^ w ^ )</div><h2>Hubungkan Asep ke WiFi</h2>");
  if (err.length()) h += "<div class='e'>" + err + "</div>";
  h += "<form method='POST' action='/connect'><label>Nama WiFi (SSID)</label>"
       "<input type='text' name='ssid' list='n' value='" + htmlEscape(ssid) + "' required autocomplete='off'>"
       "<datalist id='n'>" + scanOpts + "</datalist>"
       "<label>Password</label><input id='p' type='password' name='pass'>"
       "<label class='s'><input type='checkbox' onclick=\"document.getElementById('p').type=this.checked?'text':'password'\"> Lihat password</label>"
       "<button type='submit'>Hubungkan</button></form><a href='/rescan'>Scan ulang WiFi</a></div></body></html>";
  return h;
}

void scanNetworks() {
  int n = WiFi.scanNetworks();
  scanOpts = "";
  for (int i = 0; i < n && i < 15; i++) scanOpts += "<option value='" + htmlEscape(WiFi.SSID(i)) + "'>";
  WiFi.scanDelete();
}

void redirectHome() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

void handleConnect() {
  String ssid = server.arg("ssid"), pass = server.arg("pass");
  setStatus("Mencoba: " + ssid.substring(0, 18));
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    if (WiFi.status() == WL_CONNECT_FAILED) break;
  }
  if (WiFi.status() == WL_CONNECTED) {
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    setStatus("Terhubung!");
    server.sendHeader("Connection", "close");
    server.send(200, "text/html",
                F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
                  "<body style='font-family:sans-serif;text-align:center;background:#fff1ee;color:#5a3a44;padding:40px'>"
                  "<h1>( ^ w ^ )</h1><h2>Terhubung!</h2><p>Asep sudah online. Kamu boleh tutup halaman ini.</p></body></html>"));
    portalDone = true;
  } else {
    bool noSsid = (WiFi.status() == WL_NO_SSID_AVAIL);
    WiFi.disconnect(false);
    setStatus("Gagal, coba lagi");
    server.send(200, "text/html",
                portalPage(noSsid ? "WiFi tidak ditemukan, cek nama WiFi anda" : "Password anda salah, coba lagi", ssid));
  }
}

void runPortal() {
  portalActive = true;
  portalDone = false;
  setStatus("Menunggu HP...");
  ui = UI_PORTAL;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_NAME);
  delay(300);
  dns.start(53, "*", WiFi.softAPIP());
  scanNetworks();

  server.on("/", HTTP_GET, []() { server.sendHeader("Cache-Control", "no-store"); server.send(200, "text/html", portalPage("", "")); });
  server.on("/rescan", HTTP_GET, []() { scanNetworks(); redirectHome(); });
  server.on("/connect", HTTP_POST, handleConnect);
  const char* probes[] = {"/generate_204", "/gen_204", "/hotspot-detect.html", "/library/test/success.html",
                          "/connecttest.txt", "/ncsi.txt", "/fwlink", "/canonical.html", "/success.txt", "/redirect"};
  for (auto p : probes) server.on(p, HTTP_ANY, redirectHome);
  server.onNotFound(redirectHome);
  server.begin();

  while (!portalDone) {
    dns.processNextRequest();
    server.handleClient();
    delay(5);
    if (btnShort) btnShort = false;
  }
  uint32_t t = millis();
  while (millis() - t < 2500) { server.handleClient(); delay(5); }
  server.stop();
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  portalActive = false;
  if (WiFi.status() != WL_CONNECTED) {
    String s = prefs.getString("ssid", ""), p = prefs.getString("pass", "");
    WiFi.begin(s.c_str(), p.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);
  }
}

void connectWiFi() {
  String s = prefs.getString("ssid", ""), p = prefs.getString("pass", "");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  if (s.length()) {
    setStatus("Menyambung WiFi...");
    ui = UI_WIFI;
    WiFi.begin(s.c_str(), p.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
  }
  if (WiFi.status() != WL_CONNECTED) runPortal();
}

void resetWiFiAndPortal() {
  prefs.remove("ssid");
  prefs.remove("pass");
  WiFi.disconnect(true, true);
  delay(200);
  runPortal();
  setStatus("");
  ui = UI_IDLE;
}

// ======================= KAMERA =======================
bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_PIN_D0; c.pin_d1 = CAM_PIN_D1; c.pin_d2 = CAM_PIN_D2; c.pin_d3 = CAM_PIN_D3;
  c.pin_d4 = CAM_PIN_D4; c.pin_d5 = CAM_PIN_D5; c.pin_d6 = CAM_PIN_D6; c.pin_d7 = CAM_PIN_D7;
  c.pin_xclk = CAM_PIN_XCLK; c.pin_pclk = CAM_PIN_PCLK;
  c.pin_vsync = CAM_PIN_VSYNC; c.pin_href = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD; c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_pwdn = CAM_PIN_PWDN; c.pin_reset = CAM_PIN_RESET;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_SXGA;     // 1280x1024, cukup tajam untuk baca soal
  c.jpeg_quality = 10;               // makin kecil makin bagus (4-12)
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Kamera gagal init: 0x%x\n", err);
    return false;
  }
  return true;
}

camera_fb_t* captureJpeg() {
  if (!camOK) return nullptr;
  for (int i = 0; i < 3; i++) {   // buang beberapa frame supaya exposure stabil
    camera_fb_t* f = esp_camera_fb_get();
    if (f) esp_camera_fb_return(f);
    delay(60);
  }
  return esp_camera_fb_get();
}

// ======================= AUDIO =======================
void initAudio() {
  i2sMic.setPins(PIN_MIC_SCK, PIN_MIC_WS, -1, PIN_MIC_SD);
  if (!i2sMic.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT))
    Serial.println("I2S mic gagal");
  i2sSpk.setPins(PIN_SPK_BCLK, PIN_SPK_LRC, PIN_SPK_DIN, -1);
  if (!i2sSpk.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_BOTH))
    Serial.println("I2S speaker gagal");
}

// Putar PCM 16-bit mono 16 kHz lewat speaker (dipakai nanti oleh TTS)
void speakerPlay(const int16_t* pcm, size_t samples) {
  i2sSpk.write((const uint8_t*)pcm, samples * 2);
}

// ---- TEXT TO SPEECH (placeholder, Asep masih bisu) ----
// Nanti isi TTS_API_KEY, lalu: minta audio PCM 16kHz mono dari provider TTS pilihanmu,
// simpan ke buffer, dan panggil speakerPlay(buffer, jumlahSample).
void ttsSpeak(const String& text) {
  if (strlen(TTS_API_KEY) == 0) return;   // bisu dulu
  // TODO: request ke API TTS pakai TTS_API_KEY, lalu speakerPlay(...)
  (void)text;
}

// Rekam satu ucapan. Return jumlah sample (0 = tidak ada suara / dibatalkan).
size_t recordUtterance(uint32_t maxWaitMs, uint32_t maxLenMs, uint32_t endSilenceMs) {
  const size_t CH = 320;  // 20 ms
  static int32_t raw[320];
  size_t idx = 0;
  bool started = false;
  uint32_t t0 = millis(), lastLoud = 0;
  uint16_t loud = 0;
  size_t maxS = min((size_t)(maxLenMs * (SAMPLE_RATE / 1000)), (size_t)(REC_MAX_SAMPLES - CH));

  while (true) {
    if (btnShort || btnLong) return 0;
    size_t got = i2sMic.readBytes((char*)raw, sizeof(raw)) / 4;
    uint32_t now = millis();
    if (got == 0) {
      delay(5);
      if (!started && now - t0 > maxWaitMs) return 0;
      continue;
    }
    double acc = 0;
    for (size_t i = 0; i < got; i++) {
      int32_t v = raw[i] >> 14;
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      recBuf[idx + i] = (int16_t)v;
      acc += (double)v * v;
    }
    float rms = sqrtf((float)(acc / got));
    micLevel = 0.7f * micLevel + 0.3f * min(1.0f, rms / 6000.0f);

    if (!started) {
      float thr = max(noiseFloor * 2.8f, MIN_SPEECH_RMS);
      idx += got;
      if (rms > thr) {
        started = true;
        lastLoud = now;
        loud = 1;
      } else {
        noiseFloor = 0.97f * noiseFloor + 0.03f * rms;
        if (idx > PREROLL_SAMPLES) {
          size_t drop = idx - PREROLL_SAMPLES;
          memmove(recBuf, recBuf + drop, PREROLL_SAMPLES * 2);
          idx = PREROLL_SAMPLES;
        }
        if (now - t0 > maxWaitMs) return 0;
      }
    } else {
      float thr = max(noiseFloor * 2.0f, MIN_SPEECH_RMS * 0.7f);
      idx += got;
      if (rms > thr) { lastLoud = now; loud++; }
      if (now - lastLoud > endSilenceMs || idx >= maxS) break;
    }
  }
  if (loud < 8) return 0;  // terlalu pendek (klik/ketukan)
  return idx;
}

// ======================= SPEECH TO TEXT =======================
String transcribe(const int16_t* pcm, size_t samples, bool& ok) {
  ok = false;
  if (strlen(STT_API_KEY) == 0) return "STT_API_KEY belum diisi";
  if (WiFi.status() != WL_CONNECTED) return "WiFi putus";

  const String B = "----AsepBoundary7MA4YWx";
  String head = "--" + B + "\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\n" + String(STT_MODEL) + "\r\n";
  if (strlen(STT_LANGUAGE))
    head += "--" + B + "\r\nContent-Disposition: form-data; name=\"language\"\r\n\r\n" + String(STT_LANGUAGE) + "\r\n";
  head += "--" + B + "\r\nContent-Disposition: form-data; name=\"prompt\"\r\n\r\nHalo Asep.\r\n";
  head += "--" + B + "\r\nContent-Disposition: form-data; name=\"temperature\"\r\n\r\n0\r\n";
  head += "--" + B + "\r\nContent-Disposition: form-data; name=\"response_format\"\r\n\r\njson\r\n";
  head += "--" + B + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"a.wav\"\r\nContent-Type: audio/wav\r\n\r\n";
  String tail = "\r\n--" + B + "--\r\n";

  size_t dataLen = samples * 2;
  size_t total = head.length() + 44 + dataLen + tail.length();
  uint8_t* body = (uint8_t*)ps_malloc(total);
  if (!body) return "Memori kurang";

  uint8_t* p = body;
  memcpy(p, head.c_str(), head.length());
  p += head.length();
  // header WAV
  uint32_t sr = SAMPLE_RATE, br = SAMPLE_RATE * 2, ds = dataLen, cs = 36 + dataLen;
  uint16_t fmt = 1, ch = 1, ba = 2, bps = 16;
  uint32_t sub1 = 16;
  memcpy(p, "RIFF", 4); memcpy(p + 4, &cs, 4); memcpy(p + 8, "WAVEfmt ", 8);
  memcpy(p + 16, &sub1, 4); memcpy(p + 20, &fmt, 2); memcpy(p + 22, &ch, 2);
  memcpy(p + 24, &sr, 4); memcpy(p + 28, &br, 4); memcpy(p + 32, &ba, 2); memcpy(p + 34, &bps, 2);
  memcpy(p + 36, "data", 4); memcpy(p + 40, &ds, 4);
  p += 44;
  memcpy(p, pcm, dataLen);
  p += dataLen;
  memcpy(p, tail.c_str(), tail.length());

  String result = "";
  for (int attempt = 0; attempt < 2; attempt++) {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(15000);
    http.setTimeout(30000);
    if (!http.begin(client, STT_URL)) { result = "Gagal konek STT"; continue; }
    http.addHeader("Authorization", String("Bearer ") + STT_API_KEY);
    http.addHeader("Content-Type", "multipart/form-data; boundary=" + B);
    int code = http.POST(body, total);
    String resp = code > 0 ? http.getString() : "";
    http.end();
    if (code == 200) {
      JsonDocument doc;
      if (!deserializeJson(doc, resp)) {
        result = String((const char*)(doc["text"] | ""));
        result.trim();
        ok = true;
        break;
      }
      result = "Respon STT rusak";
    } else if (code == 401 || code == 403) {
      result = "STT API key ditolak";
      break;
    } else {
      result = "STT error " + String(code);
      delay(1000);
    }
  }
  free(body);
  return result;
}

// ======================= OPENROUTER =======================
String askLLM(const String& userText, camera_fb_t* fb, bool& ok) {
  ok = false;
  if (WiFi.status() != WL_CONNECTED) {
    if (fb) esp_camera_fb_return(fb);
    return "WiFi putus, coba lagi nanti ya";
  }
  char* b64 = nullptr;
  size_t b64len = 0;
  if (fb) {
    size_t cap = 4 * ((fb->len + 2) / 3) + 8;
    b64 = (char*)ps_malloc(cap);
    if (!b64) { esp_camera_fb_return(fb); return "Memori kurang untuk foto"; }
    mbedtls_base64_encode((unsigned char*)b64, cap, &b64len, fb->buf, fb->len);
    esp_camera_fb_return(fb);
  }

  String body;
  body.reserve(b64len + 8000);
  body = "{\"model\":\"" OPENROUTER_MODEL "\",\"max_tokens\":";
  body += fb ? "2500" : "600";
  body += ",\"temperature\":";
  body += fb ? "0.2" : "0.7";
  body += ",\"messages\":[{\"role\":\"system\",\"content\":\"";
  body += jsonEscape(SYSTEM_PROMPT);
  body += "\"}";
  for (int i = 0; i < hN; i++) {
    body += ",{\"role\":\"user\",\"content\":\"" + jsonEscape(hU[i]) + "\"}";
    body += ",{\"role\":\"assistant\",\"content\":\"" + jsonEscape(hA[i]) + "\"}";
  }
  body += ",{\"role\":\"user\",\"content\":";
  if (fb) {
    body += "[{\"type\":\"text\",\"text\":\"" + jsonEscape(userText) + "\"},{\"type\":\"image_url\",\"image_url\":{\"url\":\"data:image/jpeg;base64,";
    body.concat(b64, b64len);
    body += "\"}}]";
    free(b64);
  } else {
    body += "\"" + jsonEscape(userText) + "\"";
  }
  body += "}]}";

  String answer = "";
  String lastErr = "Gagal menghubungi AI";
  for (int attempt = 0; attempt < 3; attempt++) {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(15000);
    http.setTimeout(60000);
    http.setReuse(false);
    if (!http.begin(client, "https://openrouter.ai/api/v1/chat/completions")) { delay(1500); continue; }
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + OPENROUTER_API_KEY);
    http.addHeader("HTTP-Referer", "https://github.com/asep-mochi");
    http.addHeader("X-Title", "Asep Mochi ESP32");
    int code = http.POST((uint8_t*)body.c_str(), body.length());
    String resp = code > 0 ? http.getString() : "";
    http.end();

    if (code == 200) {
      JsonDocument doc;
      if (!deserializeJson(doc, resp)) {
        String c = String((const char*)(doc["choices"][0]["message"]["content"] | ""));
        c = toAscii(c);
        c.trim();
        if (c.length()) { answer = c; ok = true; break; }
        lastErr = "AI jawab kosong, coba tanya lagi";
      } else {
        lastErr = "Jawaban AI tidak terbaca";
      }
      delay(1500);
      continue;
    }
    JsonDocument ed;
    String msg = "";
    if (resp.length() && !deserializeJson(ed, resp)) msg = String((const char*)(ed["error"]["message"] | ""));
    lastErr = "AI error " + String(code) + (msg.length() ? ": " + toAscii(msg).substring(0, 80) : String(""));
    if (code == 429 || code >= 500 || code < 0) { delay(2500); continue; }
    break;
  }
  body = "";
  if (ok) {
    if (hN == 3) { for (int i = 0; i < 2; i++) { hU[i] = hU[i + 1]; hA[i] = hA[i + 1]; } hN = 2; }
    hU[hN] = fb ? "[foto soal] " + userText : userText;
    hA[hN] = answer.substring(0, 400);
    hN++;
    return answer;
  }
  return lastErr;
}

// ======================= LOGIKA PERCAKAPAN =======================
bool detectWake(const String& textIn, String& rest) {
  String a = toAscii(textIn);
  String t = a;
  t.toLowerCase();
  const char* names[] = {"asep", "acep", "asef", "azep"};
  const char* hello[] = {"halo", "hallo", "hello", "hai", "hei", "hey", "alo", "helo", "hola"};
  int pos = -1, nlen = 0;
  for (auto n : names) {
    int p = t.indexOf(n);
    if (p >= 0 && (pos < 0 || p < pos)) { pos = p; nlen = strlen(n); }
  }
  if (pos < 0) return false;
  String before = t.substring(0, pos);
  bool g = false;
  for (auto h : hello) if (before.indexOf(h) >= 0) g = true;
  if (!g) return false;
  rest = a.substring(pos + nlen);
  while (rest.length() && !isalnum((uint8_t)rest[0])) rest.remove(0, 1);
  rest.trim();
  return true;
}

bool isPhotoCommand(const String& textIn) {
  String t = toAscii(textIn);
  t.toLowerCase();
  const char* keys[] = {"foto", "soal", "kamera", "cekrek", "jepret", "lihat ini", "liat ini", "baca ini", "scan"};
  for (auto k : keys) if (t.indexOf(k) >= 0) return true;
  return false;
}

void answerQuestion(const String& text, bool withPhoto) {
  setStatus("");
  xSemaphoreTake(mtx, portMAX_DELAY);
  uiQuestion = toAscii(text);
  xSemaphoreGive(mtx);

  bool ok = false;
  String ans;
  if (withPhoto) {
    ui = UI_CAMERA;
    delay(900);
    camera_fb_t* fb = captureJpeg();
    if (!fb) {
      showText(text, "Kamera belum siap atau gagal ambil foto", true);
      return;
    }
    ui = UI_THINK;
    ans = askLLM("Ini foto dari kamera. " + text, fb, ok);
  } else {
    ui = UI_THINK;
    ans = askLLM(text, nullptr, ok);
  }
  showText(text, ans, !ok);
  if (ok) ttsSpeak(ans);
  activeUntil = millis() + FOLLOWUP_MS;
}

// ======================= SETUP & LOOP =======================
void setup() {
  Serial.begin(115200);   // catatan: GPIO19/20 dipakai LCD, gunakan port UART (bukan USB native) untuk Serial Monitor
  pinMode(PIN_BTN, INPUT_PULLUP);
  mtx = xSemaphoreCreateMutex();
  randomSeed(esp_random());

  BG = C565(255, 246, 240);
  INK = C565(74, 46, 58);
  BLUSH = C565(255, 168, 184);
  SOFT = C565(176, 146, 156);
  WHITEC = 0xFFFF;
  PINKD = C565(214, 84, 120);

  tft.init();
  tft.setRotation(0);
  tft.setBrightness(255);
  canvas.setPsram(true);
  canvas.setColorDepth(16);
  canvas.createSprite(240, 240);

  bootStart = millis();
  xTaskCreatePinnedToCore(displayTask, "disp", 8192, nullptr, 1, nullptr, 0);

  if (!psramFound()) Serial.println("PSRAM TIDAK AKTIF! Tools > PSRAM harus diaktifkan");
  recBuf = (int16_t*)ps_malloc(REC_MAX_SAMPLES * 2);
  camOK = initCamera();
  initAudio();
  prefs.begin("asep", false);
  connectWiFi();

  // pastikan "STARTING..." tampil minimal 3 detik
  while (millis() - bootStart < BOOT_MS) delay(20);
  if (!recBuf) { showText("", "Memori rekam gagal. Aktifkan PSRAM di menu Tools!", true); uiExpire = 0xFFFFFFFF; while (true) delay(1000); }
  if (!camOK) Serial.println("Kamera tidak aktif (cek pin kamera)");
  setStatus("");
  ui = UI_IDLE;
}

void loop() {
  if (btnLong) {
    btnLong = false;
    resetWiFiAndPortal();
    return;
  }
  if (btnShort) {
    btnShort = false;
    answerQuestion("Baca soal pada foto ini dengan teliti, kerjakan langkah demi langkah, lalu tulis jawaban akhirnya dengan jelas.", true);
    return;
  }

  uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastTry = 0;
    if (now - lastTry > 10000) { lastTry = now; WiFi.reconnect(); }
  }
  if ((ui == UI_ANSWER || ui == UI_ERROR) && now > uiExpire) ui = UI_IDLE;

  bool active = now < activeUntil;
  if (active && ui == UI_IDLE) ui = UI_LISTEN;
  if (!active && ui == UI_LISTEN) ui = UI_IDLE;

  uint32_t wait = active ? (activeUntil - now) : 1500;
  size_t n = active ? recordUtterance(wait, 12000, 1200)
                    : recordUtterance(wait, 9000, 700);
  micLevel = 0;
  if (n == 0) return;

  UiState prev = ui;
  if (active) { ui = UI_THINK; xSemaphoreTake(mtx, portMAX_DELAY); uiQuestion = ""; xSemaphoreGive(mtx); }

  bool ok = false;
  String text = transcribe(recBuf, n, ok);
  Serial.println("STT: " + text);

  if (!ok) {
    if (active) showText("", text, true);
    else if (text.indexOf("belum diisi") >= 0 || text.indexOf("ditolak") >= 0) showText("", text, true);
    else ui = prev;
    return;
  }
  text.trim();
  if (text.length() < 2) { ui = (active ? UI_LISTEN : prev); return; }

  if (!active) {
    String rest;
    if (!detectWake(text, rest)) { ui = prev; return; }
    happyUntil = millis() + 900;
    if (rest.length() < 3) {            // cuma "Halo Asep" -> diam dan dengerin
      activeUntil = millis() + 9000;
      ui = UI_LISTEN;
      return;
    }
    text = rest;
  }
  answerQuestion(text, isPhotoCommand(text));
}
