#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <WebServer.h>
  #include <HTTPUpdateServer.h>
  #include <SPI.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266HTTPUpdateServer.h>
#endif
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#ifndef ESP32
  #include <NeoPixelBus.h>
#endif
#include <WiFiUdp.h>
#include <SdFat.h>

// ── Constants ─────────────────────────────────────────────────────────────────

static const char*    CONFIG_FILE       = "/config.json";
static const char*    AP_SSID           = "PixelController";
static const char*    AP_PASSWORD       = "setup1234";
static const uint32_t CONNECT_TIMEOUT   = 15000;
static const uint16_t DDP_PORT          = 4048;
static const uint32_t DDP_TIMEOUT_MS    = 10000;
static const uint8_t  DDP_HEADER_LEN    = 10;
static const uint8_t  DDP_FLAG_PUSH     = 0x01; // byte 0 bit 0 per the DDP spec
static const uint8_t  DDP_MAX_DRAIN     = 16;   // packets per loop(), bounds latency
static const char*    FW_VERSION        = "0.2.0";
static const uint16_t FW_VERSION_MAJOR  = 0;
static const uint16_t FW_VERSION_MINOR  = 2;
#ifdef ESP32
static const uint8_t  FPP_TYPE_ESP          = 0xC3; // FPP's id for an ESP32 board
static const char*    FPP_HW_NAME           = "ESPixelStick-ESP32";
static const char*    FPP_VARIANT           = "PixelController-ESP32";
static const uint8_t  PIXEL_PIN             = 4;  // default for a fresh config
static const uint8_t  SD_CS_PIN             = 5;  // VSPI: SCK=18 MISO=19 MOSI=23
static const uint8_t  SD_RESERVED_PINS[]    = {5, 18, 19, 23};
#else
static const uint8_t  FPP_TYPE_ESP          = 0xC2; // FPP's id for an ESP8266 board
static const char*    FPP_HW_NAME           = "ESPixelStick-ESP8266";
static const char*    FPP_VARIANT           = "PixelController-ESP8266";
static const uint8_t  PIXEL_PIN             = 2;  // GPIO2 (D4), UART1 — fixed on ESP8266
static const uint8_t  SD_CS_PIN             = 15; // D8; MISO=D6 MOSI=D7 CLK=D5
#endif
static const uint16_t FPP_PORT          = 32320;
static const uint8_t  MAX_SPARSE_RANGES = 8;

// ── Hardware ──────────────────────────────────────────────────────────────────

#ifndef ESP32
// ESP8266: one WS2811 string on GPIO2 (D4) via UART1 — non-blocking, WiFi-safe
using PixelStrip = NeoPixelBus<NeoRgbFeature, NeoEsp8266Uart1Ws2811Method>;
PixelStrip* strip = nullptr;
#endif

SdFat32 sd;
bool    sdAvailable = false;

// Upload state, valid only for the duration of one POST /upload
File32   uploadFile;
String   uploadPath;
String   uploadError;
uint32_t uploadBytes = 0;
bool     uploadOk    = false;

// ── Network ───────────────────────────────────────────────────────────────────

#ifdef ESP32
WebServer             server(80);
HTTPUpdateServer      httpUpdater;
#else
ESP8266WebServer      server(80);
ESP8266HTTPUpdateServer httpUpdater;
#endif
WiFiUDP               ddpUdp;
WiFiUDP               fppUdp;
bool                  apMode    = false;
uint32_t              lastDdpMs = 0;

// ── Config ────────────────────────────────────────────────────────────────────

// ── Outputs (config) ──────────────────────────────────────────────────────────
// Each output is one GPIO: a pixel string (WS2811/WS2812/SK6812 family, any
// colour order, 3 or 4 channels per pixel) or a single PWM channel. Outputs take
// consecutive slices of one channel space, in list order — that is the channel
// numbering DDP and FSEQ address.

static const uint8_t  MAX_OUTPUTS        = 16;
static const uint16_t MAX_PIXELS_PER_OUT = 1024;
#ifdef ESP32
static const uint32_t MAX_TOTAL_CHANNELS = 4096;
#else
static const uint32_t MAX_TOTAL_CHANNELS = 3072;
#endif

enum class OutType : uint8_t { PIXEL, PWM };

struct OutputCfg {
    uint8_t  pin      = PIXEL_PIN;
    OutType  type     = OutType::PIXEL;
    char     order[5] = "RGB";  // wire colour order; 4 letters = RGBW pixels
    uint16_t count    = 30;     // pixels (PIXEL only)
    uint16_t freq     = 1000;   // Hz (PWM only)
};

static uint32_t channelsOf(const OutputCfg& o) {
    return o.type == OutType::PWM ? 1 : (uint32_t)o.count * strlen(o.order);
}

struct Config {
    String   ssid;
    String   password;
    String   hostname         = "pixelcontroller";
    float    gamma            = 1.0f;
    OutputCfg outputs[MAX_OUTPUTS];
    uint8_t   outputCount     = 1;
};
Config config;

// ── Gamma correction ──────────────────────────────────────────────────────────

uint8_t gammaLUT[256];

void rebuildGammaLUT() {
    for (uint16_t i = 0; i < 256; i++) {
        gammaLUT[i] = (uint8_t)(powf((float)i / 255.0f, config.gamma) * 255.0f + 0.5f);
    }
}

// ── Outputs (runtime) ─────────────────────────────────────────────────────────

static bool orderValid(const char* order) {
    size_t n = strlen(order);
    if (n != 3 && n != 4) return false;
    for (size_t i = 0; i < n; i++) {
        if (!strchr("RGBW", order[i])) return false;
        for (size_t k = 0; k < i; k++) if (order[k] == order[i]) return false;
    }
    return n == 4 || !strchr(order, 'W');
}

static bool pinUsable(uint8_t pin) {
#ifdef ESP32
    if (pin > 33) return false;                    // 34-39 are input-only
    if (pin >= 6 && pin <= 11) return false;       // flash
    if (pin == 20 || (pin >= 24 && pin <= 31)) return false; // not broken out on ESP32
    if (pin == 1 || pin == 3) return false;        // UART0 / USB serial
    for (uint8_t r : SD_RESERVED_PINS) if (pin == r) return false;
    return true;
#else
    return pin == PIXEL_PIN;
#endif
}

bool validateOutputs(const OutputCfg* o, uint8_t n, String& err) {
    if (n < 1 || n > MAX_OUTPUTS) { err = "1 to " + String(MAX_OUTPUTS) + " outputs required"; return false; }
#ifndef ESP32
    if (n != 1) { err = "ESP8266 supports a single output"; return false; }
#endif
    uint32_t total = 0;
    for (uint8_t i = 0; i < n; i++) {
        String tag = "Output " + String(i + 1) + ": ";
        if (!pinUsable(o[i].pin)) { err = tag + "GPIO " + String(o[i].pin) + " not usable"; return false; }
        for (uint8_t k = 0; k < i; k++)
            if (o[k].pin == o[i].pin) { err = tag + "GPIO " + String(o[i].pin) + " used twice"; return false; }
        if (o[i].type == OutType::PIXEL) {
#ifndef ESP32
            if (strlen(o[i].order) != 3) { err = tag + "ESP8266 supports RGB-order (3 channel) pixels only"; return false; }
#endif
            if (!orderValid(o[i].order)) { err = tag + "bad colour order"; return false; }
            if (o[i].count < 1 || o[i].count > MAX_PIXELS_PER_OUT) {
                err = tag + "pixel count must be 1 to " + String(MAX_PIXELS_PER_OUT); return false;
            }
        } else {
#ifndef ESP32
            err = tag + "PWM needs an ESP32"; return false;
#endif
            if (o[i].freq < 1 || o[i].freq > 40000) { err = tag + "PWM frequency must be 1 to 40000 Hz"; return false; }
        }
        total += channelsOf(o[i]);
    }
    if (total > MAX_TOTAL_CHANNELS) { err = "Too many channels (" + String(total) + ", max " + String(MAX_TOTAL_CHANNELS) + ")"; return false; }
    return true;
}

struct Output {
    OutputCfg cfg;
    uint32_t  start    = 0;   // first channel
    uint32_t  channels = 0;
    uint8_t   bpp      = 3;   // channels per pixel
    uint8_t   map[4]   = {0, 1, 2, 3}; // wire byte i <- logical channel map[i]
#ifdef ESP32
    rmt_data_t* sym    = nullptr;
    bool        ok     = false;
#endif
};

Output   outputs[MAX_OUTPUTS];
uint8_t  outputCount = 0;
uint8_t* chan        = nullptr;   // logical channel buffer, gamma already applied
uint32_t totalCh     = 0;

#ifdef ESP32
// RMT tick = 100 ns. 0 = 0.3 us high / 0.9 us low, 1 = 0.7 us high / 0.5 us low
// — inside the tolerance of WS2811 (800 kHz), WS2812 and SK6812.
static rmt_data_t mkSym(uint16_t d0, uint8_t l0, uint16_t d1, uint8_t l1) {
    rmt_data_t s; s.duration0 = d0; s.level0 = l0; s.duration1 = d1; s.level1 = l1; return s;
}
static const uint32_t RMT_TICK_HZ = 10000000;
#endif

void initOutputs() {
    outputCount = config.outputCount;
    totalCh     = 0;
    for (uint8_t i = 0; i < outputCount; i++) {
        Output& o = outputs[i];
        o.cfg      = config.outputs[i];
        o.start    = totalCh;
        o.channels = channelsOf(o.cfg);
        o.bpp      = strlen(o.cfg.order);
        for (uint8_t b = 0; b < o.bpp; b++)
            o.map[b] = strchr("RGBW", o.cfg.order[b]) - "RGBW";
        totalCh   += o.channels;
    }
    free(chan);
    chan = (uint8_t*)calloc(totalCh ? totalCh : 1, 1);

    for (uint8_t i = 0; i < outputCount; i++) {
        Output& o = outputs[i];
#ifdef ESP32
        if (o.cfg.type == OutType::PWM) {
            o.ok = ledcAttach(o.cfg.pin, o.cfg.freq, 8);
        } else {
            o.sym = (rmt_data_t*)malloc((o.channels * 8 + 1) * sizeof(rmt_data_t));
            o.ok  = o.sym && rmtInit(o.cfg.pin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, RMT_TICK_HZ);
        }
        Serial.printf("Output %u: GPIO%u %s %s x%u  ch %u..%u  %s\n", i + 1, o.cfg.pin,
            o.cfg.type == OutType::PWM ? "PWM" : "pixel",
            o.cfg.type == OutType::PWM ? "" : o.cfg.order,
            o.cfg.type == OutType::PWM ? 1 : o.cfg.count,
            o.start, o.start + o.channels - 1, o.ok ? "ok" : "INIT FAILED");
#else
        strip = new PixelStrip(o.cfg.count);
        strip->Begin();
#endif
    }
}

// Push the channel buffer to every output. Never blocks: a pixel output whose
// previous frame is still on the wire skips this one and picks up the newer
// data next time.
void showOutputs() {
    for (uint8_t i = 0; i < outputCount; i++) {
        Output& o = outputs[i];
        const uint8_t* src = chan + o.start;
#ifdef ESP32
        if (!o.ok) continue;
        if (o.cfg.type == OutType::PWM) { ledcWrite(o.cfg.pin, src[0]); continue; }
        if (!rmtTransmitCompleted(o.cfg.pin)) continue;

        static const rmt_data_t ZERO = mkSym(3, 1, 9, 0), ONE = mkSym(7, 1, 5, 0), LATCH = mkSym(1500, 0, 1500, 0);
        size_t n = 0;
        for (uint16_t p = 0; p < o.cfg.count; p++) {
            for (uint8_t b = 0; b < o.bpp; b++) {
                uint8_t v = src[p * o.bpp + o.map[b]];
                for (int bit = 7; bit >= 0; bit--) o.sym[n++] = (v >> bit) & 1 ? ONE : ZERO;
            }
        }
        o.sym[n++] = LATCH;
        rmtWriteAsync(o.cfg.pin, o.sym, n);
#else
        uint8_t* dst = strip->Pixels();
        for (uint16_t p = 0; p < o.cfg.count; p++)
            for (uint8_t b = 0; b < 3; b++) dst[p * 3 + b] = src[p * 3 + o.map[b]];
        strip->Dirty();
        if (strip->CanShow()) strip->Show();
#endif
    }
}

void fillOutputs(uint8_t value) {
    memset(chan, value, totalCh);
    showOutputs();
}

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
                if (ch < totalCh) chan[ch] = gammaLUT[rdBuf[i]];
            }
            done += chunk;
        }
    }
    showOutputs();
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

// Drain every DDP packet queued this iteration, apply all of it to the pixel
// buffer, then latch once. Showing per-packet tears a frame that arrives as
// several packets, and consuming only one packet per loop() lets a fast sender
// overrun the small lwIP receive queue — both look like flicker on the strip.
void handleDDP() {
    bool     touched = false;
    bool     pushed  = false;
    uint8_t  drained = 0;

    while (drained < DDP_MAX_DRAIN) {
        int size = ddpUdp.parsePacket();
        if (size < DDP_HEADER_LEN) break;
        drained++;

        uint8_t header[DDP_HEADER_LEN];
        ddpUdp.read(header, DDP_HEADER_LEN);

        uint32_t byteOffset = ((uint32_t)header[4] << 24) | ((uint32_t)header[5] << 16)
                            | ((uint32_t)header[6] <<  8) |  (uint32_t)header[7];
        uint16_t byteLen    = ((uint16_t)header[8] << 8) | header[9];

        if (header[0] & DDP_FLAG_PUSH) pushed = true;

        uint8_t buf[128];
        uint16_t done = 0;
        while (done < byteLen) {
            int got = ddpUdp.read(buf, min((uint16_t)sizeof(buf), (uint16_t)(byteLen - done)));
            if (got <= 0) break;
            for (int i = 0; i < got; i++) {
                uint32_t ch = byteOffset + done + i;
                if (ch < totalCh) chan[ch] = gammaLUT[buf[i]];
            }
            done += got;
        }
        touched = true;
    }

    if (!touched) return;
    lastDdpMs = millis();

    static uint32_t lastLog = 0;
    if (millis() - lastLog > 2000) {
        lastLog = millis();
        Serial.printf("DDP: drained=%u push=%d\n", drained, pushed);
    }

    // Non-blocking: outputs still transmitting skip this frame (see showOutputs)
    showOutputs();
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
    pkt[9]  = FPP_TYPE_ESP;   // ping_hardware
    pkt[10] = FW_VERSION_MAJOR >> 8; pkt[11] = FW_VERSION_MAJOR & 0xFF; // BE
    pkt[12] = FW_VERSION_MINOR >> 8; pkt[13] = FW_VERSION_MINOR & 0xFF;
    pkt[14] = 0x08;           // operatingMode: bridge
    uint32_t ip = (uint32_t)WiFi.localIP();
    memcpy(pkt + 15, &ip, 4);
    strncpy((char*)(pkt + 19),  config.hostname.c_str(), 64);
    strncpy((char*)(pkt + 84),  FW_VERSION,              40);
    strncpy((char*)(pkt + 125), FPP_HW_NAME,             40);
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
                fillOutputs(0);
                break;
        }
    }
}

// ── Idle blanking ─────────────────────────────────────────────────────────────

// When DDP has gone quiet and nothing is playing, black everything out once.
// Any new DDP or playback re-arms it.
void updateIdle() {
    if (!chan) return;
    static bool blanked = false;

    bool active = (millis() - lastDdpMs) < DDP_TIMEOUT_MS || playState != PlayState::IDLE;
    if (active) { blanked = false; return; }
    if (blanked) return;

    fillOutputs(0);
    blanked = true;
}

// ── SD helpers ────────────────────────────────────────────────────────────────

#ifdef ESP32
static const uint8_t SD_SPI_MHZ = 20;
#else
static const uint8_t SD_SPI_MHZ = 4;
#endif

void initSD() {
    sdAvailable = sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(SD_SPI_MHZ)));
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

static void outputsToJson(JsonArray arr) {
    for (uint8_t i = 0; i < config.outputCount; i++) {
        const OutputCfg& o = config.outputs[i];
        JsonObject j = arr.add<JsonObject>();
        j["pin"] = o.pin;
        if (o.type == OutType::PWM) {
            j["type"] = "pwm";
            j["freq"] = o.freq;
        } else {
            j["type"]  = "pixel";
            j["order"] = o.order;
            j["count"] = o.count;
        }
    }
}

// Parses into scratch space and validates; only a fully valid list is returned.
static bool outputsFromJson(JsonArrayConst arr, OutputCfg* out, uint8_t& n, String& err) {
    if (arr.size() > MAX_OUTPUTS) { err = "Too many outputs"; return false; }
    n = 0;
    for (JsonObjectConst j : arr) {
        OutputCfg& o = out[n++];
        o = OutputCfg();
        o.pin  = j["pin"] | PIXEL_PIN;
        o.type = String(j["type"] | "pixel") == "pwm" ? OutType::PWM : OutType::PIXEL;
        o.freq = j["freq"] | 1000;
        o.count = j["count"] | 30;
        strlcpy(o.order, j["order"] | "RGB", sizeof(o.order));
    }
    return validateOutputs(out, n, err);
}

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
    config.gamma            = doc["gamma"]            | 1.0f;

    // No "outputs" key: configs from before multi-output carry a bare pixelCount
    OutputCfg tmp[MAX_OUTPUTS];
    uint8_t   n = 0;
    String    verr;
    if (doc["outputs"].is<JsonArray>() && outputsFromJson(doc["outputs"], tmp, n, verr)) {
        for (uint8_t i = 0; i < n; i++) config.outputs[i] = tmp[i];
        config.outputCount = n;
    } else {
        config.outputs[0]       = OutputCfg();
        config.outputs[0].count = doc["pixelCount"] | 30;
        config.outputCount      = 1;
    }
    rebuildGammaLUT();
    return config.ssid.length() > 0;
}

void saveConfig() {
    File f = LittleFS.open(CONFIG_FILE, "w");
    if (!f) return;
    JsonDocument doc;
    doc["ssid"]             = config.ssid;
    doc["password"]         = config.password;
    doc["hostname"]         = config.hostname;
    doc["gamma"]            = config.gamma;
    outputsToJson(doc["outputs"].to<JsonArray>());
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
<div class="row"><span class="key">Outputs</span><span>%OUTPUTS%</span></div>
<div class="row"><span class="key">Gamma</span><span>%GAMMA%</span></div>
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
<title>Outputs</title>
<style>body{font-family:sans-serif;max-width:640px;margin:40px auto;padding:0 16px;color:#333}
h1{margin-bottom:4px}p.sub{color:#666;margin-top:0;font-size:.9em}
label{display:block;margin-top:14px;font-weight:600;font-size:.9em}
table{width:100%;border-collapse:collapse;margin-top:14px;font-size:.9em}
th{text-align:left;padding:4px;border-bottom:2px solid #ddd;color:#555}
td{padding:4px;border-bottom:1px solid #f0f0f0}
select,input[type=number]{padding:6px;border:1px solid #ccc;border-radius:4px;font-size:1em;box-sizing:border-box;max-width:100%}
td input[type=number]{width:90px}.ch{color:#888;white-space:nowrap}
.check-row{display:flex;align-items:center;gap:10px;margin-top:16px}
.check-row label{margin:0;font-weight:normal;font-size:1em}
button{padding:10px 16px;border:none;border-radius:4px;font-size:1em;cursor:pointer;color:#fff;background:#198754}
button.sec{background:#6c757d;margin-top:12px}button.x{background:#dc3545;padding:4px 10px}
#msg{margin-top:12px;min-height:1.2em;font-size:.9em}.err{color:#dc3545}.hint{color:#888;font-size:.82em;margin-top:12px}</style>
</head><body>
<h1>Outputs</h1><p class="sub">Each output is one GPIO. Channels are assigned in list order.</p>
<table><thead><tr><th>#<th>GPIO<th>Type<th>Order / Hz<th>Pixels<th>Channels<th></tr></thead><tbody id="rows"></tbody></table>
<button class="sec" id="add">+ Add output</button>
<label>Gamma correction</label>
<input type="number" id="gamma" min="0.1" max="5" step="0.1">
<p><button id="save">Save &amp; Restart</button></p>
<div id="msg"></div>
<p class="hint">Gamma 1.0 = none; ~2.2 suits most WS2811 strips and also evens out PWM dimming. Pixel order is the strip's wire order; the channel data from xLights is always R,G,B(,W). PWM outputs use one channel each (8-bit).</p>
<p class="hint">DDP (port 4048) and FPP sync always take priority. All outputs go black after %TIMEOUT%s of silence.</p>
<script>
var cfg,rows=document.getElementById('rows'),msg=document.getElementById('msg');
var orders=[];
function perm(a,p,o){if(!a.length){o.push(p);return}for(var i=0;i<a.length;i++)perm(a.slice(0,i).concat(a.slice(i+1)),p+a[i],o)}
perm(['R','G','B'],'',orders);perm(['R','G','B','W'],'',orders);
function opt(vals,sel){return vals.map(function(v){return '<option'+(v==sel?' selected':'')+'>'+v+'</option>'}).join('')}
function addRow(o){
  var tr=document.createElement('tr');
  var pins=cfg.pins.map(function(p){return '<option'+(p==o.pin?' selected':'')+'>'+p+'</option>'}).join('');
  tr.innerHTML='<td class=n><td><select class=pin>'+pins+'</select>'
   +'<td><select class=type><option value=pixel>Pixels</option>'+(cfg.esp32?'<option value=pwm>PWM</option>':'')+'</select>'
   +'<td><select class=order>'+opt(cfg.esp32?orders:orders.slice(0,6),o.order||'RGB')+'</select><input type=number class=freq min=1 max=40000>'
   +'<td><input type=number class=count min=1 max='+cfg.maxPixels+'><td class=ch><td><button class=x>&times;</button>';
  tr.querySelector('.type').value=o.type||'pixel';
  tr.querySelector('.freq').value=o.freq||1000;
  tr.querySelector('.count').value=o.count||30;
  tr.addEventListener('input',refresh);
  tr.querySelector('.x').onclick=function(){tr.remove();refresh()};
  rows.appendChild(tr);
}
function refresh(){
  var c=0;
  [].forEach.call(rows.children,function(tr,i){
    var pwm=tr.querySelector('.type').value=='pwm';
    tr.querySelector('.n').textContent=i+1;
    tr.querySelector('.order').style.display=pwm?'none':'';
    tr.querySelector('.freq').style.display=pwm?'':'none';
    tr.querySelector('.count').style.display=pwm?'none':'';
    var n=pwm?1:tr.querySelector('.count').value*tr.querySelector('.order').value.length;
    tr.querySelector('.ch').textContent=n?(c+1)+'–'+(c+n):'';
    c+=n;
  });
  document.getElementById('add').style.display=rows.children.length>=cfg.maxOutputs?'none':'';
  document.querySelectorAll('.x').forEach(function(b){b.style.display=rows.children.length>1?'':'none'});
}
document.getElementById('add').onclick=function(){
  var used=[].map.call(rows.querySelectorAll('.pin'),function(s){return +s.value});
  var free=cfg.pins.filter(function(p){return used.indexOf(p)<0})[0];
  addRow({pin:free===undefined?cfg.pins[0]:free});refresh();
};
document.getElementById('save').onclick=function(){
  var outs=[].map.call(rows.children,function(tr){
    var pwm=tr.querySelector('.type').value=='pwm',o={pin:+tr.querySelector('.pin').value,type:tr.querySelector('.type').value};
    if(pwm)o.freq=+tr.querySelector('.freq').value;else{o.order=tr.querySelector('.order').value;o.count=+tr.querySelector('.count').value}
    return o});
  var body={gamma:+document.getElementById('gamma').value,outputs:outs};
  msg.className='';msg.textContent='Saving…';
  fetch('/api/outputs',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
   .then(function(r){return r.text().then(function(t){
     if(r.ok){msg.textContent='Saved. Restarting…';setTimeout(function(){location.href='/'},5000)}
     else{msg.className='err';msg.textContent=t}})})
   .catch(function(){msg.className='err';msg.textContent='Request failed'});
};
fetch('/api/outputs').then(function(r){return r.json()}).then(function(c){
  cfg=c;document.getElementById('gamma').value=c.gamma;
  c.outputs.forEach(addRow);refresh();
});
</script>
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
.stop{background:#dc3545;margin-left:8px}.stop:hover{background:#bb2d3b}
#up{margin-top:8px}
#bar{height:6px;background:#eee;border-radius:3px;overflow:hidden;margin-top:10px;display:none}
#fill{height:100%;width:0;background:#198754;transition:width .2s}
#msg{font-size:.85em;color:#666;margin-top:6px;min-height:1em}</style>
</head><body>
<h1>SD Card</h1>
%SD_CONTENT%
<h2>Upload sequence</h2>
<input type="file" id="up" accept=".fseq,.FSEQ">
<div id="bar"><div id="fill"></div></div>
<div id="msg"></div>
<a class="btn back" href="/">&#8592; Back</a>
<script>
document.getElementById('up').onchange=function(){
  var f=this.files[0]; if(!f) return;
  var bar=document.getElementById('bar'),fill=document.getElementById('fill'),
      msg=document.getElementById('msg'),fd=new FormData();
  fd.append('f',f,f.name);
  bar.style.display='block'; msg.textContent='Uploading '+f.name+'…'; this.disabled=true;
  var x=new XMLHttpRequest();
  x.upload.onprogress=function(e){
    if(e.lengthComputable) fill.style.width=(e.loaded/e.total*100)+'%';
  };
  x.onload=function(){
    if(x.status==200){ msg.textContent='Done. Reloading…'; location.reload(); }
    else { msg.textContent='Failed: '+x.responseText; fill.style.background='#dc3545'; }
  };
  x.onerror=function(){ msg.textContent='Upload failed (connection lost)'; fill.style.background='#dc3545'; };
  x.open('POST','/upload'); x.send(fd);
};
</script>
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
    page.replace("%OUTPUTS%",     String(config.outputCount) + " (" + String(totalCh) + " ch)");
    page.replace("%GAMMA%",       String(config.gamma, 1));
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
    page.replace("%TIMEOUT%", String(DDP_TIMEOUT_MS / 1000));
    server.send(200, "text/html", page);
}

void handleOutputsGet() {
    JsonDocument doc;
#ifdef ESP32
    doc["esp32"] = true;
#else
    doc["esp32"] = false;
#endif
    doc["gamma"]            = config.gamma;
    doc["maxOutputs"]       = MAX_OUTPUTS;
    doc["maxPixels"]        = MAX_PIXELS_PER_OUT;
    JsonArray pins = doc["pins"].to<JsonArray>();
    for (uint8_t p = 0; p < 40; p++) if (pinUsable(p)) pins.add(p);
    outputsToJson(doc["outputs"].to<JsonArray>());
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
}

void handleOutputsPost() {
    JsonDocument doc;
    if (deserializeJson(doc, server.arg("plain"))) {
        server.send(400, "text/plain", "Bad JSON");
        return;
    }
    OutputCfg tmp[MAX_OUTPUTS];
    uint8_t   n = 0;
    String    err;
    if (!outputsFromJson(doc["outputs"].as<JsonArrayConst>(), tmp, n, err)) {
        server.send(400, "text/plain", err);
        return;
    }
    float gamma = doc["gamma"] | config.gamma;
    if (gamma < 0.1f || gamma > 5.0f) { server.send(400, "text/plain", "Gamma must be 0.1 to 5"); return; }

    for (uint8_t i = 0; i < n; i++) config.outputs[i] = tmp[i];
    config.outputCount      = n;
    config.gamma            = gamma;
    saveConfig();
    server.send(200, "text/plain", "OK");
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
    fillOutputs(0);
    server.sendHeader("Location", "/sd");
    server.send(302, "text/plain", "");
}

// ── FPP-compatible HTTP API ───────────────────────────────────────────────────
// Just enough of the FPP surface for xLights FPP Connect to discover this
// board and push sequences to it. Field names mirror what FPP itself returns;
// xLights matches on them.

static void fppSysInfo(JsonObject o) {
    o["HostName"]        = config.hostname;
    o["HostDescription"] = config.hostname;
    o["Platform"]        = "ESPixelStick";
    o["Variant"]         = FPP_VARIANT;
    o["Mode"]            = sdAvailable ? "remote" : "bridge";
    o["Version"]         = FW_VERSION;
    o["majorVersion"]    = FW_VERSION_MAJOR;
    o["minorVersion"]    = FW_VERSION_MINOR;
    o["typeId"]          = FPP_TYPE_ESP;
    o["UUID"]            = WiFi.macAddress();

    JsonObject util = o["Utilization"].to<JsonObject>();
    util["MemoryFree"] = ESP.getFreeHeap();
    util["Uptime"]     = millis();

    o["rssi"] = WiFi.RSSI();
    JsonArray ips = o["IPS"].to<JsonArray>();
    ips.add(WiFi.localIP().toString());
}

void handleFppSysInfo() {
    JsonDocument doc;
    fppSysInfo(doc.to<JsonObject>());
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
}

void handleFppStatus() {
    JsonDocument doc;
    JsonObject o = doc.to<JsonObject>();
    fppSysInfo(o);
    o["fppd"]            = "running";
    o["status"]          = playState == PlayState::IDLE ? 0 : 1;
    o["status_name"]     = playState == PlayState::IDLE ? "idle" : "playing";
    o["current_sequence"] = playState == PlayState::IDLE ? "" : String(seqFileName);
    o["seconds_played"]  = playState == PlayState::IDLE
                         ? 0 : (millis() - syncMs) / 1000;
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
}

// GET /api/sequence/<name>/meta — xLights asks this before uploading so it can
// skip a file the card already has.
void handleFppSequenceMeta(const String& name) {
    if (!sdAvailable) initSD();
    File32 f;
    String path = name.startsWith("/") ? name : "/" + name;
    if (!sdAvailable || !f.open(path.c_str(), O_RDONLY)) {
        server.send(404, "application/json", "{\"error\":\"not found\"}");
        return;
    }

    uint8_t h[32];
    int got = f.read(h, sizeof(h));
    uint32_t fileSize = f.fileSize();
    f.close();
    if (got < (int)sizeof(h) || memcmp(h, "PSEQ", 4) != 0) {
        server.send(404, "application/json", "{\"error\":\"not an fseq\"}");
        return;
    }

    JsonDocument doc;
    JsonObject o = doc.to<JsonObject>();
    o["Name"]            = name;
    o["Version"]         = String(h[7]) + "." + String(h[6]);
    o["StepTime"]        = h[18];
    o["NumFrames"]       = (uint32_t)h[14] | ((uint32_t)h[15] << 8)
                         | ((uint32_t)h[16] << 16) | ((uint32_t)h[17] << 24);
    o["ChannelCount"]    = (uint32_t)h[10] | ((uint32_t)h[11] << 8)
                         | ((uint32_t)h[12] << 16) | ((uint32_t)h[13] << 24);
    o["MaxChannel"]      = o["ChannelCount"];
    o["CompressionType"] = h[20];
    o["FileSize"]        = fileSize;

    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
}

// /fpp and /fpp/... dispatch on a ?path= argument, the way FPP does.
void handleFppGet() {
    String path = server.hasArg("path") ? server.arg("path") : server.uri();
    if (path.startsWith("/fpp/")) path = path.substring(4);

    if (path.indexOf("system/info") >= 0)   { handleFppSysInfo(); return; }
    if (path.indexOf("system/status") >= 0) { handleFppStatus();  return; }
    if (path == "ping" || path == "/" || path.length() == 0) {
        server.send(200, "text/plain", "OK");
        return;
    }
    server.send(404, "text/plain", "");
}

// Catches the wildcard routes ESP8266WebServer can't register directly.
void handleNotFound() {
    String uri = server.uri();
    if (uri.startsWith("/api/sequence/")) {
        String seq = uri.substring(14);
        if (seq.endsWith("/meta")) seq = seq.substring(0, seq.length() - 5);
        handleFppSequenceMeta(seq);
        return;
    }
    server.send(404, "text/plain", "Not found");
}

// Streamed straight to the card in chunks — sequences run to many megabytes,
// far past what will fit in RAM.
void handleUploadData() {
    HTTPUpload& up = server.upload();

    if (up.status == UPLOAD_FILE_START) {
        uploadOk    = false;
        uploadBytes = 0;

        // Playback holds seqFile open on the same card; stop before writing.
        if (playState != PlayState::IDLE) closeSequence();
        if (!sdAvailable) initSD();
        if (!sdAvailable) { uploadError = "No SD card"; return; }

        // Some browsers send a full path — keep the basename only so an upload
        // can never escape the card root.
        String name = up.filename;
        int cut = name.lastIndexOf('/');
        if (cut >= 0) name = name.substring(cut + 1);
        cut = name.lastIndexOf('\\');
        if (cut >= 0) name = name.substring(cut + 1);
        if (name.length() == 0) { uploadError = "Empty filename"; return; }

        uploadPath = "/" + name;
        sd.remove(uploadPath.c_str());
        uploadOk = uploadFile.open(uploadPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC);
        if (!uploadOk) uploadError = "Could not open " + uploadPath + " for write";
        else           Serial.printf("Upload start: %s\n", uploadPath.c_str());

    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (uploadOk) {
            if (uploadFile.write(up.buf, up.currentSize) != up.currentSize) {
                uploadError = "Write failed — card full?";
                uploadFile.close();
                sd.remove(uploadPath.c_str());
                uploadOk = false;
            } else {
                uploadBytes += up.currentSize;
            }
        }

    } else if (up.status == UPLOAD_FILE_END) {
        if (uploadOk) {
            uploadFile.close();
            Serial.printf("Upload done: %s (%u bytes)\n", uploadPath.c_str(), uploadBytes);
        }

    } else if (up.status == UPLOAD_FILE_ABORTED) {
        if (uploadOk) {
            uploadFile.close();
            sd.remove(uploadPath.c_str());
            uploadOk = false;
        }
        uploadError = "Upload aborted";
    }
}

void handleUploadDone() {
    if (uploadOk) {
        server.send(200, "text/plain",
                    "OK " + uploadPath + " (" + String(uploadBytes) + " bytes)");
    } else {
        String msg = uploadError.length() ? uploadError : String("Upload failed");
        uploadError = "";
        server.send(500, "text/plain", msg);
    }
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
#ifdef ESP32
    fppUdp.beginMulticast(IPAddress(239, 70, 80, 80), FPP_PORT);
#else
    fppUdp.beginMulticast(WiFi.localIP(), IPAddress(239, 70, 80, 80), FPP_PORT);
#endif
    sendFPPPing(IPAddress(239, 70, 80, 80));
    sendFPPPing(WiFi.broadcastIP());

    httpUpdater.setup(&server, "/update");
    server.on("/",       HTTP_GET,  handleRoot);
    server.on("/wifi",   HTTP_GET,  handleWifi);
    server.on("/save",   HTTP_POST, handleSave);
    server.on("/pixels", HTTP_GET,  handlePixelsGet);
    server.on("/api/outputs", HTTP_GET,  handleOutputsGet);
    server.on("/api/outputs", HTTP_POST, handleOutputsPost);
    server.on("/sd",     HTTP_GET,  handleSdGet);
    server.on("/play",   HTTP_GET,  handlePlayGet);
    server.on("/stop",   HTTP_GET,  handleStopGet);
    server.on("/upload", HTTP_POST, handleUploadDone, handleUploadData);

    // FPP-compatible surface for xLights FPP Connect
    server.on("/fpp",                HTTP_GET,  handleFppGet);
    server.on("/fpp",                HTTP_POST, handleUploadDone, handleUploadData);
    server.on("/api/system/info",    HTTP_GET,  handleFppSysInfo);
    server.on("/api/system/status",  HTTP_GET,  handleFppStatus);
    server.onNotFound(handleNotFound);
    server.begin();

    Serial.printf("Connected — IP: %s  hostname: %s.local  outputs: %u (%u ch)\n",
        WiFi.localIP().toString().c_str(), config.hostname.c_str(), outputCount, totalCh);
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println();
#ifdef ESP32
    LittleFS.begin(true);
#else
    LittleFS.begin();
#endif
    loadConfig();
    rebuildGammaLUT();
    initSD();

    initOutputs();
    fillOutputs(0);

    if (config.ssid.length() > 0) {
        WiFi.mode(WIFI_STA);
#ifdef ESP32
        WiFi.setHostname(config.hostname.c_str());
#else
        WiFi.hostname(config.hostname);
#endif
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
    updateIdle();
}
