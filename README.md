# PixelController

WS2811 pixel controller firmware for the Wemos D1 Mini (ESP8266). Plays FSEQ
sequences from an SD card, follows FPP MultiSync, and accepts DDP from xLights.

## Features

- **DDP input** (UDP 4048) — live control from xLights or any DDP source.
- **FSEQ v2 playback** from SD card, uncompressed, with sparse range support.
- **FPP MultiSync** (UDP 32320) — responds to discovery pings and follows
  Start / Stop / Sync / Open from an FPP master. Frame timing uses dead
  reckoning between sync packets.
- **Web UI** — status page, WiFi setup, SD browser with play/stop, OTA update.
- **Standalone animation** when no source is driving the strip.

Input priority is DDP > FPP/FSEQ > built-in animation.

## Hardware

| Function   | Pin            |
|------------|----------------|
| Pixel data | GPIO2 (D4), UART1 |
| SD CS      | GPIO15 (D8)    |
| SD MISO    | D6             |
| SD MOSI    | D7             |
| SD CLK     | D5             |

## Build

PlatformIO, `d1_mini` env, `eagle.flash.4m1m.ld` (1 MB LittleFS).

```sh
pio run                      # build
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
| `pixelCount`       | Number of WS2811 pixels                   |
| `animationEnabled` | Run the built-in animation when idle      |

## Endpoints

| Path            | Purpose                        |
|-----------------|--------------------------------|
| `/`             | Status                         |
| `/setup`        | WiFi configuration             |
| `/sd`           | List `.fseq` files on the card |
| `/play?f=NAME`  | Start a sequence               |
| `/stop`         | Stop playback                  |
| `/update`       | OTA firmware upload            |

## Notes

- FSEQ must be **uncompressed** v2. Files with compression blocks are rejected.
- The setup AP uses a default password compiled into `src/main.cpp`. Change it
  there if you want something other than the shipped default.
- `reference/` (upstream ESPixelStick source, kept locally for reference) is not
  part of this repo.
