# PixelController

Pixel/PWM controller firmware for the Wemos D1 Mini (ESP8266) and ESP32. Plays FSEQ
sequences from an SD card, follows FPP MultiSync, and accepts DDP from xLights.

## Features

- **DDP input** (UDP 4048) — live control from xLights or any DDP source.
- **FSEQ v2 playback** from SD card, uncompressed, with sparse range support.
- **FPP MultiSync** (UDP 32320) — responds to discovery pings and follows
  Start / Stop / Sync / Open from an FPP master. Frame timing uses dead
  reckoning between sync packets.
- **Multiple outputs** (ESP32) — each on a selectable GPIO: WS2811/WS2812/SK6812
  pixels (any colour order, RGB or RGBW) or single-channel PWM.
- **Web UI** — status page, WiFi setup, output config, SD browser, OTA update.
- **Idle blanking** — all outputs go black after 10 s with no DDP or sequence.

Input priority is DDP > FPP/FSEQ; with neither, outputs are black.

## Hardware

**ESP8266 (`d1_mini`)** — one WS2811 RGB output, fixed pins.

| Function   | Pin            |
|------------|----------------|
| Pixel data | GPIO2 (D4), UART1 |
| SD CS      | GPIO15 (D8)    |
| SD MISO / MOSI / CLK | D6 / D7 / D5 |

**ESP32 (`esp32dev`)** — up to 16 outputs, GPIO chosen in the web UI.

| Function   | Pin            |
|------------|----------------|
| Outputs    | any of 0, 2, 4, 12-17, 21, 22, 32, 33 (default: GPIO4) |
| SD CS / SCK / MISO / MOSI | 5 / 18 / 19 / 23 |

Pixel outputs use the RMT peripheral (8 max); PWM uses LEDC (8-bit).
Boot-strapping pins (0, 2, 12, 15) are offered but can interfere with flashing
if the attached load pulls them the wrong way.

## Build

PlatformIO, envs `d1_mini` (`eagle.flash.4m1m.ld`, 1 MB LittleFS) and `esp32dev`.

```sh
pio run -e esp32dev          # build (or -e d1_mini)
pio run -t upload            # flash over USB
pio run -t uploadfs          # flash LittleFS (data/)
pio run -t upload --upload-port 192.168.1.50   # OTA
```

## Configuration

Two ways to get a board onto WiFi.

**Pre-flash (no AP dance):** copy the example config, fill it in, and upload the
filesystem.

```sh
cp data/config.example.json data/config.json
$EDITOR data/config.json
pio run -t uploadfs
```

**AP setup:** an unconfigured board brings up an access point named
`PixelController`. Join it and open `http://192.168.4.1` to pick a network.

`data/config.json` is gitignored — it holds your real SSID and PSK. Only the
example file is tracked.

### Config fields

| Key                | Meaning                                  |
|--------------------|------------------------------------------|
| `ssid` / `password`| WiFi credentials                          |
| `hostname`         | mDNS / OTA name                           |
| `outputs`          | List of outputs, see below                |
| `gamma`            | Gamma correction (1.0 = none, ~2.2 typical for WS2811) |

### Outputs

Outputs take consecutive slices of one channel space, in list order; that is the
channel numbering DDP and FSEQ address. Edit them at `/pixels` or in config:

```json
"outputs": [
  {"pin": 4,  "type": "pixel", "order": "GRB",  "count": 50},
  {"pin": 16, "type": "pixel", "order": "GRBW", "count": 20},
  {"pin": 13, "type": "pwm",   "freq": 1000}
]
```

`order` is the strip's wire order (data from xLights is always R,G,B(,W));
4 letters means RGBW pixels. A PWM output is one channel. Limits: 1024 pixels
per output, 4096 channels total (ESP8266: one RGB output, 3072 channels).
A config with a bare `pixelCount` (older firmware) is still read as one output.

## Endpoints

| Path            | Purpose                        |
|-----------------|--------------------------------|
| `/`             | Status                         |
| `/setup`        | WiFi configuration             |
| `/sd`           | List `.fseq` files on the card |
| `/play?f=NAME`  | Start a sequence               |
| `/stop`         | Stop playback                  |
| `/pixels`       | Output configuration           |
| `/update`       | OTA firmware upload            |

## Notes

- FSEQ must be **uncompressed** v2. Files with compression blocks are rejected.
- The setup AP uses a default password compiled into `src/main.cpp`. Change it
  there if you want something other than the shipped default.
- `reference/` (upstream ESPixelStick source, kept locally for reference) is not
  part of this repo.
