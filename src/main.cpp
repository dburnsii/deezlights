#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <NeoPixelBus.h>
#include <WiFiUdp.h>
#include <SdFat.h>

// ── Constants ─────────────────────────────────────────────────────────────────

static const char*    CONFIG_FILE       = "/config.json";
static const char*    AP_SSID           = "PixelController";
static const char*    AP_PASSWORD       = "setup1234";
static const uint32_t CONNECT_TIMEOUT   = 15000;
static const uint16_t DDP_PORT          = 4048;
static const uint32_t DDP_TIMEOUT_MS    = 10000;
static const uint16_t FPP_PORT          = 32320;
static const uint8_t  MAX_SPARSE_RANGES = 8;
static const uint8_t  SD_CS_PIN         = 15; // D8; MISO=D6 MOSI=D7 CLK=D5

// ── Hardware ──────────────────────────────────────────────────────────────────

// WS2811 output on GPIO2 (D4) via UART1 — non-blocking, WiFi-safe
using PixelStrip = NeoPixelBus<NeoRgbFeature, NeoEsp8266Uart1Ws2811Method>;
PixelStrip* strip = nullptr;

SdFat32 sd;
bool    sdAvailable = false;

// ── Network ───────────────────────────────────────────────────────────────────

ESP8266WebServer      server(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiUDP               ddpUdp;
WiFiUDP               fppUdp;
bool                  apMode    = false;
uint32_t              lastDdpMs = 0;

// ── Config ────────────────────────────────────────────────────────────────────

struct Config {
    String   ssid;
    String   password;
    String   hostname         = "pixelcontroller";
    uint16_t pixelCount       = 30;
    bool     animationEnabled = true;
};
Config config;

// ── FSEQ playback ─────────────────────────────────────────────────────────────

struct SparseRange { uint32_t start, count; };

enum class PlayState { IDLE, PLAYING };

PlayState playState       = PlayState::IDLE;
File32    seqFile;
char      seqFileName[64] = "";
uint32_t  seqFrameCount   = 0;
uint32_t  seqFrameSize    = 0;   // bytes per frame (sum of sparse range counts)
uint16_t  seqDataOffset   = 0;
uint8_t   seqStepTimeMs   = 25;
uint32_t  syncFrame       = 0;   // dead-reckoning anchor: frame at syncMs
uint32_t  syncMs          = 0;   // millis() when syncFrame was set
uint32_t  lastRenderedFrame = UINT32_MAX;
SparseRange sparseRanges[MAX_SPARSE_RANGES];
uint8_t   sparseRangeCount = 0;

void closeSequence() {
    if (seqFile.isOpen()) seqFile.close();
    playState        = PlayState::IDLE;
    lastRenderedFrame = UINT32_MAX;
    seqFileName[0]   = '\0';
}

bool openSequence(const char* filename) {
    closeSequence();
    if (!sdAvailable) { Serial.println("FSEQ: no SD"); return false; }

    if (!seqFile.open(filename, O_RDONLY)) {
        Serial.printf("FSEQ: can't open %s\n", filename);
        return false;
    }

    uint8_t hdr[32];
    if (seqFile.read(hdr, 32) != 32) { seqFile.close(); return false; }

    if (hdr[0] != 'P' || hdr[1] != 'S' || hdr[2] != 'E' || hdr[3] != 'Q') {
        Serial.println("FSEQ: bad magic"); seqFile.close(); return false;
    }
    if (hdr[7] != 2) {
        Serial.printf("FSEQ: v%u unsupported (need v2)\n", hdr[7]);
        seqFile.close(); return false;
    }

    seqDataOffset  = (uint16_t)hdr[4] | ((uint16_t)hdr[5] << 8);
    uint32_t chCount = (uint32_t)hdr[10] | ((uint32_t)hdr[11] << 8)
                     | ((uint32_t)hdr[12] << 16) | ((uint32_t)hdr[13] << 24);
    seqFrameCount  = (uint32_t)hdr[14] | ((uint32_t)hdr[15] << 8)
                   | ((uint32_t)hdr[16] << 16) | ((uint32_t)hdr[17] << 24);
    seqStepTimeMs  = hdr[18];
    uint16_t numCompBlocks = (uint16_t)hdr[20] | ((uint16_t)hdr[21] << 8);
    uint8_t  numSparse     = hdr[22];

    if (numCompBlocks > 0) {
        Serial.println("FSEQ: compressed — export uncompressed from xLights");
        seqFile.close(); return false;
    }

    // Sparse ranges immediately follow the 32-byte fixed header
    sparseRangeCount = 0;
    seqFrameSize     = 0;
    if (numSparse > 0) {
        uint8_t n = min(numSparse, MAX_SPARSE_RANGES);
        for (uint8_t i = 0; i < n; i++) {
            uint8_t rb[6];
            seqFile.read(rb, 6);
            sparseRanges[i].start = (uint32_t)rb[0] | ((uint32_t)rb[1] << 8) | ((uint32_t)rb[2] << 16);
            sparseRanges[i].count = (uint32_t)rb[3] | ((uint32_t)rb[4] << 8) | ((uint32_t)rb[5] << 16);
            seqFrameSize += sparseRanges[i].count;
        }
        sparseRangeCount = n;
    } else {
        sparseRanges[0]  = {0, chCount};
        sparseRangeCount = 1;
        seqFrameSize     = chCount;
    }

    strncpy(seqFileName, filename, sizeof(seqFileName) - 1);
    Serial.printf("FSEQ: %s  frames=%u  step=%ums  size=%u/frame  sparse=%u\n",
        filename, seqFrameCount, seqStepTimeMs, seqFrameSize, numSparse);
    return true;
}

static uint8_t rdBuf[512];

void renderFrame(uint32_t frameNum) {
    if (!seqFile.isOpen()) return;
    uint32_t filePos = (uint32_t)seqDataOffset + frameNum * seqFrameSize;
    if (!seqFile.seekSet(filePos)) return;

    for (uint8_t ri = 0; ri < sparseRangeCount; ri++) {
        uint32_t chStart = sparseRanges[ri].start;
        uint32_t chCount = sparseRanges[ri].count;
        uint32_t done    = 0;

        while (done < chCount) {
            uint16_t chunk = (uint16_t)min((uint32_t)sizeof(rdBuf), chCount - done);
            if (seqFile.read(rdBuf, chunk) != chunk) return;

            for (uint16_t i = 0; i < chunk; i++) {
                uint32_t ch = chStart + done + i;
                uint16_t px = ch / 3;
                uint8_t  co = ch % 3;
                if (px >= config.pixelCount) continue;
                RgbColor c = strip->GetPixelColor(px);
                if      (co == 0) c.R = rdBuf[i];
                else if (co == 1) c.G = rdBuf[i];
                else              c.B = rdBuf[i];
                strip->SetPixelColor(px, c);
            }
            done += chunk;
        }
    }
    strip->Show();
}

void updatePlayback() {
    if (playState == PlayState::IDLE) return;
    if ((millis() - lastDdpMs) < DDP_TIMEOUT_MS) return; // DDP takes priority

    uint32_t frame = syncFrame + (millis() - syncMs) / seqStepTimeMs;

    if (frame >= seqFrameCount) {
        // Loop
        syncFrame = 0;
        syncMs    = millis();
        frame     = 0;
    }

    if (frame != lastRenderedFrame) {
        renderFrame(frame);
        lastRenderedFrame = frame;
    }
}

// ── DDP ───────────────────────────────────────────────────────────────────────

void handleDDP() {
    int size = ddpUdp.parsePacket();
    if (size < 10) return;

    uint8_t header[10];
    ddpUdp.read(header, 10);

    uint32_t byteOffset = ((uint32_t)header[4] << 24) | ((uint32_t)header[5] << 16)
                        | ((uint32_t)header[6] <<  8) |  (uint32_t)header[7];
    uint16_t byteLen    = ((uint16_t)header[8] << 8) | header[9];

    static uint32_t lastLog = 0;
    if (millis() - lastLog > 2000) {
        lastLog = millis();
        Serial.printf("DDP: size=%d hdr=%02x %02x %02x %02x  offset=%u len=%u push=%d\n",
            size, header[0], header[1], header[2], header[3],
            byteOffset, byteLen, (header[0] & 0x02) != 0);
    }

    uint16_t pixelStart = byteOffset / 3;
    uint16_t pixelCount = byteLen    / 3;

    for (uint16_t i = 0; i < pixelCount; i++) {
        uint8_t rgb[3] = {0, 0, 0};
        if (ddpUdp.read(rgb, 3) < 3) break;
        if (pixelStart + i < config.pixelCount)
            strip->SetPixelColor(pixelStart + i, RgbColor(rgb[0], rgb[1], rgb[2]));
    }

    // TODO: buffer multi-packet frames and only Show() on push flag for large
    // strings (>480 pixels) where a single frame spans multiple DDP packets.
    // Currently safe — single packet per frame at 340 pixels.
    strip->Show();
    lastDdpMs = millis();
}

// ── FPP packets ───────────────────────────────────────────────────────────────
// FPP MultiSync protocol (all packets start with "FPPD"):
//
// Ping packet (type 0x04) — 301 bytes:
//   [0-3]  "FPPD"
//   [4]    0x04
//   [5-6]  data_len = 294 (uint16 LE)
//   [7]    ping_version = 0x03
//   [8]    ping_subtype: 0x00=announce, 0x01=discover request
//   [9]    ping_hardware (0xC2 = ESP8266)
//   [10-11] versionMajor (uint16 LE)
//   [12-13] versionMinor (uint16 LE)
//   [14]   operatingMode (0x08 = bridge)
//   [15-18] ipAddress[4]
//   [19-83] hostName[65]
//   [84-124] version[41]
//   [125-165] hardwareType[41]
//   [166-286] ranges[121]
//
// Sync packet (type 0x01):
//   [0-3]  "FPPD"
//   [4]    0x01
//   [5-6]  data_len (uint16 LE)
//   [7]    sync_action: 0=start 1=stop 2=sync 3=open
//   [8]    sync_type:   0=sequence 1=media
//   [9-12] frame_number (uint32 LE)
//   [13-16] seconds_elapsed (float LE)
//   [17+]  filename (null-terminated)

void sendFPPPing(IPAddress dest) {
    uint8_t pkt[301];
    memset(pkt, 0, sizeof(pkt));
    pkt[0]='F'; pkt[1]='P'; pkt[2]='P'; pkt[3]='D';
    pkt[4]  = 0x04;          // CTRL_PKT_PING
    pkt[5]  = 294 & 0xFF;    // data_len LE
    pkt[6]  = 294 >> 8;
    pkt[7]  = 0x03;          // ping_version
    pkt[8]  = 0x00;          // ping_subtype: announce (not a discover request)
    pkt[9]  = 0xC2;          // ping_hardware: ESP8266
    pkt[10] = 4; pkt[11] = 0; // versionMajor = 4
    pkt[12] = 0; pkt[13] = 0; // versionMinor = 0
    pkt[14] = 0x08;           // operatingMode: bridge
    uint32_t ip = (uint32_t)WiFi.localIP();
    memcpy(pkt + 15, &ip, 4);
    strncpy((char*)(pkt + 19),  config.hostname.c_str(), 64);
    strncpy((char*)(pkt + 84),  "4.0",                   40);
    strncpy((char*)(pkt + 125), "ESPixelStick-ESP8266",  40);
    // ranges[121] at [166] — empty = whole universe

    fppUdp.beginPacket(dest, FPP_PORT);
    fppUdp.write(pkt, sizeof(pkt));
    fppUdp.endPacket();
}

static uint32_t lastPingMs = 0;

void handleFPPPacket() {
    // Periodic announce so xLights can find us without us responding first
    if (millis() - lastPingMs > 5000) {
        sendFPPPing(IPAddress(239, 70, 80, 80));
        lastPingMs = millis();
    }

    int size = fppUdp.parsePacket();
    if (size < 5) return;

    uint8_t buf[301];
    int n = fppUdp.read(buf, min(size, (int)sizeof(buf)));
    if (n < 5) return;

    if (buf[0] != 'F' || buf[1] != 'P' || buf[2] != 'P' || buf[3] != 'D') return;

    uint8_t pktType = buf[4];

    // ── Ping (0x04): discover request or peer announce ────────────────────────
    if (pktType == 0x04 && n >= 9) {
        uint8_t subtype = buf[8];
        if (subtype == 0x01) {
            // Discover request — respond directly to sender
            sendFPPPing(fppUdp.remoteIP());
            Serial.printf("FPP discover from %s — replied\n", fppUdp.remoteIP().toString().c_str());
        }
        return;
    }

    // ── Sync (0x01): sequence playback control ────────────────────────────────
    if (pktType == 0x01 && n >= 17) {
        uint8_t  action   = buf[7];
        uint8_t  syncType = buf[8];
        uint32_t frameNum = (uint32_t)buf[9]  | ((uint32_t)buf[10] << 8)
                          | ((uint32_t)buf[11] << 16) | ((uint32_t)buf[12] << 24);

        if (syncType != 0) return; // sequence only, not media

        // Normalize to "/filename.fseq" — FPP master sends bare basename
        char fpath[68] = "/";
        if (n > 17 && buf[17] != '\0') {
            const char* raw = (const char*)(buf + 17);
            if (raw[0] == '/') strncpy(fpath, raw, sizeof(fpath) - 1);
            else               strncat(fpath, raw, sizeof(fpath) - 2);
        }

        switch (action) {
            case 3: // Open — pre-load file
                if (strcmp(fpath, seqFileName) != 0)
                    openSequence(fpath);
                break;

            case 0: // Start
                if (strcmp(fpath, seqFileName) != 0)
                    openSequence(fpath);
                if (seqFile.isOpen()) {
                    syncFrame = frameNum;
                    syncMs    = millis();
                    playState = PlayState::PLAYING;
                    Serial.printf("FPP start: %s frame %u\n", fpath, frameNum);
                }
                break;

            case 2: // Sync — dead-reckoning correction
                if (seqFile.isOpen()) {
                    syncFrame = frameNum;
                    syncMs    = millis();
                    if (playState != PlayState::PLAYING)
                        playState = PlayState::PLAYING;
                }
                break;

            case 1: // Stop
                Serial.println("FPP stop");
                closeSequence();
                strip->ClearTo(RgbColor(0));
                strip->Show();
                break;
        }
    }
}

// ── Animation ─────────────────────────────────────────────────────────────────

void updateAnimation() {
    if (!strip) return;
    if ((millis() - lastDdpMs) < DDP_TIMEOUT_MS) return;
    if (playState != PlayState::IDLE) return;

    static bool     needClear = false;
    static uint32_t lastFrame = 0;

    if (!config.animationEnabled) {
        if (!needClear) {
            strip->ClearTo(RgbColor(0));
            strip->Show();
            needClear = true;
        }
        return;
    }
    needClear = false;

    uint32_t now = millis();
    if (now - lastFrame < 20) return;
    lastFrame = now;

    float   t          = (float)(now % 5000) / 5000.0f;
    float   brightness = t < 0.5f ? t * 2.0f : (1.0f - t) * 2.0f;
    uint8_t val        = (uint8_t)(brightness * 255);

    RgbColor white(val, val, val);
    for (uint16_t i = 0; i < config.pixelCount; i++)
        strip->SetPixelColor(i, white);
    strip->Show();
}

// ── SD helpers ────────────────────────────────────────────────────────────────

void initSD() {
    sdAvailable = sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(4)));
    Serial.println(sdAvailable ? "SD ready" : "SD not found");
}

static String humanSize(uint64_t bytes) {
    if (bytes < 1024)       return String((uint32_t)bytes) + " B";
    if (bytes < 1048576)    return String((uint32_t)(bytes >> 10)) + " KB";
    if (bytes < 1073741824) return String((uint32_t)(bytes >> 20)) + " MB";
    return String((float)(bytes >> 20) / 1024.0f, 1) + " GB";
}

static String buildSDContent() {
    if (!sdAvailable) initSD();
    if (!sdAvailable)
        return "<p class='none'>No SD card detected. Insert a FAT32 card and refresh.</p>";

    String html;

    uint64_t cardBytes  = (uint64_t)sd.card()->sectorCount() * 512ULL;
    uint64_t totalBytes = (uint64_t)sd.vol()->clusterCount() * sd.vol()->bytesPerCluster();
    // freeClusterCount() scans the full FAT — skip it to avoid blocking
    html += "<div class='row'><span class='key'>Card capacity</span><span>" + humanSize(cardBytes)  + "</span></div>";
    html += "<div class='row'><span class='key'>Volume size</span><span>"   + humanSize(totalBytes) + "</span></div>";

    // Playback status
    if (playState != PlayState::IDLE) {
        uint32_t curFrame = syncFrame + (millis() - syncMs) / seqStepTimeMs;
        html += "<div class='row'><span class='key'>Playing</span><span>" +
                String(seqFileName) + " &mdash; frame " + String(curFrame) +
                " / " + String(seqFrameCount) + "</span></div>";
    }

    // File listing — show all files, highlight .fseq with play button
    html += "<h2>Files</h2><table><tr><th>Name</th><th>Size</th><th></th></tr>";
    File32 root, entry;
    root.open("/");
    while (entry.openNext(&root, O_RDONLY)) {
        char name[64];
        entry.getName(name, sizeof(name));
        bool isDir = entry.isDir();
        bool isFseq = !isDir && (strstr(name, ".fseq") || strstr(name, ".FSEQ"));
        if (isDir) {
            html += "<tr><td class='dir'>&#128193;&nbsp;" + String(name) + "/</td><td>&mdash;</td><td></td></tr>";
        } else if (isFseq) {
            html += "<tr><td>" + String(name) + "</td><td>" +
                    humanSize(entry.fileSize()) + "</td><td>"
                    "<a href='/play?f=" + String(name) + "'>&#9654; Play</a></td></tr>";
        } else {
            html += "<tr><td>" + String(name) + "</td><td>" + humanSize(entry.fileSize()) + "</td><td></td></tr>";
        }
        entry.close();
    }
    root.close();
    html += "</table>";

    if (playState != PlayState::IDLE)
        html += "<a href='/stop' class='btn stop'>&#9632; Stop</a>";

    return html;
}

// ── Config persistence ────────────────────────────────────────────────────────

bool loadConfig() {
    if (!LittleFS.exists(CONFIG_FILE)) return false;
    File f = LittleFS.open(CONFIG_FILE, "r");
    if (!f) return false;
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) return false;
    config.ssid             = doc["ssid"]             | "";
    config.password         = doc["password"]         | "";
    config.hostname         = doc["hostname"]         | "pixelcontroller";
    config.pixelCount       = doc["pixelCount"]       | 30;
    config.animationEnabled = doc["animationEnabled"] | true;
    return config.ssid.length() > 0;
}

void saveConfig() {
    File f = LittleFS.open(CONFIG_FILE, "w");
    if (!f) return;
    JsonDocument doc;
    doc["ssid"]             = config.ssid;
    doc["password"]         = config.password;
    doc["hostname"]         = config.hostname;
    doc["pixelCount"]       = config.pixelCount;
    doc["animationEnabled"] = config.animationEnabled;
    serializeJson(doc, f);
    f.close();
}

// ── Web pages ─────────────────────────────────────────────────────────────────

static const char CONFIG_PAGE[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi Setup</title>
<style>body{font-family:sans-serif;max-width:420px;margin:40px auto;padding:0 16px;color:#333}
h1{margin-bottom:4px}p.sub{color:#666;margin-top:0;font-size:.9em}
label{display:block;margin-top:14px;font-weight:600;font-size:.9em}
select,input[type=text],input[type=password]{width:100%;padding:8px;margin-top:4px;box-sizing:border-box;border:1px solid #ccc;border-radius:4px;font-size:1em}
button{margin-top:20px;width:100%;padding:11px;background:#0d6efd;color:#fff;border:none;border-radius:4px;font-size:1em;cursor:pointer}
button:hover{background:#0b5ed7}.hint{color:#888;font-size:.82em;margin-top:18px}</style>
</head><body>
<h1>WiFi Setup</h1><p class="sub">Connect this device to your network.</p>
<form method="POST" action="/save">
  <label>Network</label><select name="ssid">%NETWORKS%</select>
  <label>Password</label><input type="password" name="password" placeholder="Leave blank if open network">
  <label>Hostname</label><input type="text" name="hostname" value="%HOSTNAME%" placeholder="pixelcontroller">
  <button type="submit">Save &amp; Connect</button>
</form>
<p class="hint">After saving the device restarts. Find it at <b>%HOSTNAME%.local</b>.</p>
</body></html>
)html";

static const char STATUS_PAGE[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>%HOSTNAME%</title>
<style>body{font-family:sans-serif;max-width:420px;margin:40px auto;padding:0 16px;color:#333}
h1{margin-bottom:4px}
.row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #eee;font-size:.95em}
.key{color:#888}
a.btn{display:block;margin-top:12px;padding:10px;text-align:center;border-radius:4px;text-decoration:none;color:#fff;font-size:.95em}
.blue{background:#0d6efd}.blue:hover{background:#0b5ed7}
.green{background:#198754}.green:hover{background:#157347}
.teal{background:#0d9488}.teal:hover{background:#0b7a70}
.grey{background:#6c757d}.grey:hover{background:#5a6268}</style>
</head><body>
<h1>%HOSTNAME%</h1>
<div class="row"><span class="key">Network</span><span>%SSID%</span></div>
<div class="row"><span class="key">IP address</span><span>%IP%</span></div>
<div class="row"><span class="key">Signal</span><span>%RSSI% dBm</span></div>
<div class="row"><span class="key">Pixels</span><span>%PIXEL_COUNT% (WS2811)</span></div>
<div class="row"><span class="key">DDP</span><span>port 4048 &mdash; %DDP_STATUS%</span></div>
<div class="row"><span class="key">SD card</span><span>%SD_STATUS%</span></div>
<div class="row"><span class="key">Playback</span><span>%PLAY_STATUS%</span></div>
<a class="btn green" href="/pixels">Configure Pixels</a>
<a class="btn teal" href="/sd">SD Card &amp; Sequences</a>
<a class="btn blue" href="/update">Update Firmware</a>
<a class="btn grey" href="/wifi">Reconfigure WiFi</a>
</body></html>
)html";

static const char PIXELS_PAGE[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pixel Config</title>
<style>body{font-family:sans-serif;max-width:420px;margin:40px auto;padding:0 16px;color:#333}
h1{margin-bottom:4px}p.sub{color:#666;margin-top:0;font-size:.9em}
label{display:block;margin-top:14px;font-weight:600;font-size:.9em}
input[type=number]{width:100%;padding:8px;margin-top:4px;box-sizing:border-box;border:1px solid #ccc;border-radius:4px;font-size:1em}
.check-row{display:flex;align-items:center;gap:10px;margin-top:16px}
.check-row input{width:auto;margin:0}.check-row label{margin:0;font-weight:normal;font-size:1em}
button{margin-top:20px;width:100%;padding:11px;background:#198754;color:#fff;border:none;border-radius:4px;font-size:1em;cursor:pointer}
button:hover{background:#157347}.hint{color:#888;font-size:.82em;margin-top:12px}</style>
</head><body>
<h1>Pixel Config</h1><p class="sub">Output: GPIO2 (D4) &mdash; WS2811</p>
<form method="POST" action="/pixels">
  <label>Number of pixels</label>
  <input type="number" name="pixelCount" value="%PIXEL_COUNT%" min="1" max="1024">
  <div class="check-row">
    <input type="checkbox" name="animationEnabled" id="anim" value="1" %ANIM_CHECKED%>
    <label for="anim">Enable idle animation (ramps when no DDP or sequence playing)</label>
  </div>
  <button type="submit">Save &amp; Restart</button>
</form>
<p class="hint">DDP (port 4048) and FPP sync always take priority. Animation resumes after %TIMEOUT%s of silence.</p>
</body></html>
)html";

static const char SD_PAGE[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SD Card</title>
<style>body{font-family:sans-serif;max-width:480px;margin:40px auto;padding:0 16px;color:#333}
h1,h2{margin-bottom:4px}h2{font-size:1em;margin-top:20px;color:#555}
.row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #eee;font-size:.95em}
.key{color:#888}
table{width:100%;border-collapse:collapse;margin-top:8px;font-size:.9em}
th{text-align:left;padding:6px 4px;border-bottom:2px solid #ddd;color:#555}
td{padding:5px 4px;border-bottom:1px solid #f0f0f0}
.dir{color:#0d6efd}.none{color:#888;font-style:italic}
td a{color:#198754;text-decoration:none;font-weight:600}
td a:hover{text-decoration:underline}
a.btn{display:inline-block;margin-top:16px;padding:9px 16px;border-radius:4px;text-decoration:none;color:#fff;font-size:.9em}
.back{background:#6c757d}.back:hover{background:#5a6268}
.stop{background:#dc3545;margin-left:8px}.stop:hover{background:#bb2d3b}</style>
</head><body>
<h1>SD Card</h1>
%SD_CONTENT%
<a class="btn back" href="/">&#8592; Back</a>
</body></html>
)html";

static const char SAVED_PAGE[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width,initial-scale=1"><title>Saved</title>
<style>body{font-family:sans-serif;max-width:420px;margin:40px auto;padding:0 16px;color:#333}</style>
</head><body><h1>Saved!</h1>
<p>Restarting and connecting to <b>%SSID%</b>.</p>
<p>Find this device at <b>%HOSTNAME%.local</b>.</p>
</body></html>
)html";

// ── Helpers ───────────────────────────────────────────────────────────────────

String buildNetworkOptions() {
    int n = WiFi.scanNetworks();
    String opts;
    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        opts += "<option value='" + ssid + "'>" + ssid +
                " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
    }
    WiFi.scanDelete();
    return opts;
}

// ── Request handlers ──────────────────────────────────────────────────────────

void handleRoot() {
    if (apMode) {
        String page = FPSTR(CONFIG_PAGE);
        page.replace("%NETWORKS%", buildNetworkOptions());
        page.replace("%HOSTNAME%", config.hostname);
        server.send(200, "text/html", page);
        return;
    }
    bool ddpActive = (lastDdpMs > 0) && ((millis() - lastDdpMs) < DDP_TIMEOUT_MS);
    String playStatus = "idle";
    if (playState != PlayState::IDLE) {
        uint32_t f = syncFrame + (millis() - syncMs) / seqStepTimeMs;
        playStatus = "playing " + String(seqFileName) + " (" + String(f) + "/" + String(seqFrameCount) + ")";
    }
    String page = FPSTR(STATUS_PAGE);
    page.replace("%HOSTNAME%",    config.hostname);
    page.replace("%SSID%",        config.ssid);
    page.replace("%IP%",          WiFi.localIP().toString());
    page.replace("%RSSI%",        String(WiFi.RSSI()));
    page.replace("%PIXEL_COUNT%", String(config.pixelCount));
    page.replace("%DDP_STATUS%",  ddpActive ? "active" : "idle");
    page.replace("%SD_STATUS%",   sdAvailable ? "present" : "not detected");
    page.replace("%PLAY_STATUS%", playStatus);
    server.send(200, "text/html", page);
}

void handleWifi() {
    String page = FPSTR(CONFIG_PAGE);
    page.replace("%NETWORKS%", buildNetworkOptions());
    page.replace("%HOSTNAME%", config.hostname);
    server.send(200, "text/html", page);
}

void handleSave() {
    if (server.hasArg("ssid"))     config.ssid     = server.arg("ssid");
    if (server.hasArg("password")) config.password = server.arg("password");
    if (server.hasArg("hostname") && server.arg("hostname").length() > 0)
        config.hostname = server.arg("hostname");
    saveConfig();
    String page = FPSTR(SAVED_PAGE);
    page.replace("%SSID%",     config.ssid);
    page.replace("%HOSTNAME%", config.hostname);
    server.send(200, "text/html", page);
    delay(1500);
    ESP.restart();
}

void handlePixelsGet() {
    String page = FPSTR(PIXELS_PAGE);
    page.replace("%PIXEL_COUNT%",  String(config.pixelCount));
    page.replace("%ANIM_CHECKED%", config.animationEnabled ? "checked" : "");
    page.replace("%TIMEOUT%",      String(DDP_TIMEOUT_MS / 1000));
    server.send(200, "text/html", page);
}

void handlePixelsPost() {
    if (server.hasArg("pixelCount")) {
        int count = server.arg("pixelCount").toInt();
        if (count > 0 && count <= 1024) config.pixelCount = (uint16_t)count;
    }
    config.animationEnabled = server.hasArg("animationEnabled");
    saveConfig();
    server.send(200, "text/html", "<html><body><h2>Saved! Restarting...</h2></body></html>");
    delay(1000);
    ESP.restart();
}

void handleSdGet() {
    String page = FPSTR(SD_PAGE);
    page.replace("%SD_CONTENT%", buildSDContent());
    server.send(200, "text/html", page);
}

void handlePlayGet() {
    if (!server.hasArg("f")) { server.send(400, "text/plain", "missing file"); return; }
    String fname = "/" + server.arg("f");
    if (openSequence(fname.c_str())) {
        syncFrame = 0;
        syncMs    = millis();
        playState = PlayState::PLAYING;
        server.sendHeader("Location", "/sd");
        server.send(302, "text/plain", "");
    } else {
        server.send(500, "text/plain", "Failed to open sequence");
    }
}

void handleStopGet() {
    closeSequence();
    strip->ClearTo(RgbColor(0));
    strip->Show();
    server.sendHeader("Location", "/sd");
    server.send(302, "text/plain", "");
}

// ── Startup modes ─────────────────────────────────────────────────────────────

void startAP() {
    apMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("AP mode — SSID: %s  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
    server.on("/",     HTTP_GET,  handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.begin();
}

void startStation() {
    ArduinoOTA.setHostname(config.hostname.c_str());
    ArduinoOTA.onStart([]()              { Serial.println("OTA start"); });
    ArduinoOTA.onEnd([]()                { Serial.println("\nOTA done"); });
    ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error[%u]\n", e); });
    ArduinoOTA.begin();

    ddpUdp.begin(DDP_PORT);
    fppUdp.beginMulticast(WiFi.localIP(), IPAddress(239, 70, 80, 80), FPP_PORT);
    sendFPPPing(IPAddress(239, 70, 80, 80));
    sendFPPPing(WiFi.broadcastIP());

    httpUpdater.setup(&server, "/update");
    server.on("/",       HTTP_GET,  handleRoot);
    server.on("/wifi",   HTTP_GET,  handleWifi);
    server.on("/save",   HTTP_POST, handleSave);
    server.on("/pixels", HTTP_GET,  handlePixelsGet);
    server.on("/pixels", HTTP_POST, handlePixelsPost);
    server.on("/sd",     HTTP_GET,  handleSdGet);
    server.on("/play",   HTTP_GET,  handlePlayGet);
    server.on("/stop",   HTTP_GET,  handleStopGet);
    server.begin();

    Serial.printf("Connected — IP: %s  hostname: %s.local  pixels: %u\n",
        WiFi.localIP().toString().c_str(), config.hostname.c_str(), config.pixelCount);
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println();
    LittleFS.begin();
    loadConfig();
    initSD();

    strip = new PixelStrip(config.pixelCount);
    strip->Begin();
    strip->ClearTo(RgbColor(0));
    strip->Show();

    if (config.ssid.length() > 0) {
        WiFi.mode(WIFI_STA);
        WiFi.hostname(config.hostname);
        WiFi.begin(config.ssid, config.password);
        Serial.printf("Connecting to %s", config.ssid.c_str());
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < CONNECT_TIMEOUT) {
            delay(250);
            Serial.print(".");
        }
        Serial.println();
    }

    if (WiFi.status() == WL_CONNECTED) {
        startStation();
    } else {
        Serial.println("No connection — starting config AP");
        startAP();
    }
}

void loop() {
    server.handleClient();
    if (!apMode) {
        ArduinoOTA.handle();
        handleDDP();
        handleFPPPacket();
        updatePlayback();
    }
    updateAnimation();
}
