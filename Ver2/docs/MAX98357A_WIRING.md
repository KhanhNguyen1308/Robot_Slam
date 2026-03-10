# MAX98357A Stereo I2S Audio — Jetson Nano Wiring Guide

Two MAX98357A I2S Class-D amplifier boards are wired to the Jetson Nano
40-pin header to provide stereo text-to-speech output.

---

## Hardware overview

```
Jetson Nano 40-pin header
┌──────────────────────────────────────────────────────────────────┐
│ Pin 12  I2S0_CLK  ──────┬──────────► BCLK  (LEFT MAX98357A)     │
│                          └──────────► BCLK  (RIGHT MAX98357A)    │
│                                                                   │
│ Pin 35  I2S0_FS   ──────┬──────────► LRC   (LEFT MAX98357A)     │
│                          └──────────► LRC   (RIGHT MAX98357A)    │
│                                                                   │
│ Pin 40  I2S0_DOUT ──────┬──────────► DIN   (LEFT MAX98357A)     │
│                          └──────────► DIN   (RIGHT MAX98357A)    │
│                                                                   │
│ Pin  2  5 V       ──────┬──────────► VIN   (LEFT MAX98357A)     │
│                          └──────────► VIN   (RIGHT MAX98357A)    │
│                                                                   │
│ Pin  6  GND       ──────┬──────────► GND   (LEFT MAX98357A)     │
│                          └──────────► GND   (RIGHT MAX98357A)    │
│                                                                   │
│ Pin 17  3.3 V ──── 100 KΩ ─────────► SD_MODE (LEFT MAX98357A)  │
│ Pin  6  GND   ──── 100 KΩ ─────────► SD_MODE (RIGHT MAX98357A) │
└──────────────────────────────────────────────────────────────────┘
```

### SD_MODE channel selection (MAX98357A datasheet, Table 3)

| SD_MODE voltage | Output            |
|-----------------|-------------------|
| > 1.4 V (HIGH)  | **Left channel**  |
| < 0.4 V (LOW)   | **Right channel** |
| ~0.7 V (float)  | (L + R) / 2 mono  |
| Pulled to GND   | Shutdown          |

Pull the LEFT amplifier's SD_MODE **high** (3.3 V through 100 KΩ) and
the RIGHT amplifier's SD_MODE **low** (GND through 100 KΩ).

---

## Jetson Nano 40-pin header reference

```
 3.3 V  [ 1] [ 2]  5 V
 I2C1_SDA [ 3] [ 4]  5 V
 I2C1_SCL [ 5] [ 6]  GND
 GPIO09 [ 7] [ 8]  UART TX
   GND [ 9] [10]  UART RX
 GPIO17 [11] [12]  I2S0_CLK  ◄── BCLK
 GPIO27 [13] [14]  GND
 GPIO22 [15] [16]  GPIO23
  3.3 V [17] [18]  GPIO24     ◄── pull-up source for LEFT SD_MODE
  SPI0_MOSI [19] [20]  GND
  SPI0_MISO [21] [22]  GPIO25
  SPI0_CLK  [23] [24]  SPI0_CE0
   GND [25] [26]  SPI0_CE1
 I2C0_SDA [27] [28]  I2C0_SCL
 GPIO05 [29] [30]  GND
 GPIO06 [31] [32]  GPIO12
 I2S0_FS   [33] ────────────────────────────── LRCLK  ◄── Pin 35 in Nano pin-out
   GND [34] [35]  I2S0_FS   ◄── LRCLK
 GPIO16 [36] [37]  GPIO26
 I2S0_SDIN [38] [39]  GND
 I2S0_DOUT [40] ────────────────────────────── DIN    ◄── data out
```

> **Note:** Some Jetson Nano carrier boards label the pins differently.
> Verify with `pinmux-dts2cfg` or the NVIDIA pinout diagram for your
> specific carrier board revision.

---

## Bill of materials

| Qty | Component | Notes |
|-----|-----------|-------|
| 2   | Adafruit MAX98357A breakout (or clone) | 3 W mono Class-D amp |
| 2   | 4–8 Ω speaker (≥ 1 W) | One per amplifier |
| 2   | 100 KΩ resistor | SD_MODE pull-up / pull-down |
| –   | Dupont jumper wires | Female-to-female for header |

---

## Software setup on Jetson Nano (Ubuntu 18.04 / 20.04)

### 1 — Install ALSA utilities and espeak-ng

```bash
sudo apt update
sudo apt install -y alsa-utils espeak-ng
```

### 2 — Verify I2S card is visible

```bash
aplay -l
```

Expected output contains a line similar to:
```
card 0: tegrasndt210ref [tegra-snd-t210ref-mobile-rt565x], device 0: ...
```

If no card appears the device-tree / kernel I2S node may be disabled.
Enable it with:

```bash
# For JetPack 4.x (kernel 4.9 / 4.15):
sudo /opt/nvidia/jetson-io/jetson-io.py
# Select "Configure Jetson 40pin Header" → enable I2S
# Then reboot.
```

### 3 — Test audio playback manually

```bash
# Generate a 1-second 440 Hz sine wave and play it
speaker-test -D plughw:tegrasndt210ref,0 -t sine -f 440 -l 1

# Or play a WAV file:
aplay -D plughw:tegrasndt210ref,0 /usr/share/sounds/alsa/Front_Center.wav
```

### 4 — Set ALSA volume

```bash
amixer -D plughw:tegrasndt210ref,0 sset Master 80%
```

### 5 — Test espeak-ng → MAX98357A pipeline

```bash
espeak-ng -v en+m3 -s 150 --stdout "Hello, robot is ready" | \
  aplay -D plughw:tegrasndt210ref,0 --rate 22050 --channels 2 --format S16_LE
```

### 6 — Optional: high-quality online TTS (gTTS)

```bash
pip install gtts
sudo apt install -y ffmpeg       # required for MP3 → WAV conversion
```

Then set `use_gtts: true` in `config.yaml` `audio:` section.

---

## config.yaml reference

```yaml
audio:
  enabled: true
  alsa_device: "plughw:tegrasndt210ref,0"
  sample_rate: 22050
  channels: 2
  volume: 80
  language: "en"
  espeak_voice: "en+m3"
  espeak_speed: 150
  use_gtts: false
  cache_dir: "/tmp/robot_tts_cache"
```

---

## Python API quick-reference

```python
from tts_audio import TTSAudio, Priority

tts = TTSAudio(config)
tts.start()

tts.speak("Robot is ready.")                       # NORMAL priority
tts.speak_status("Mapping corridor B.")            # LOW — background
tts.speak_event("New area discovered.")            # NORMAL
tts.speak_warning("Battery below 20 percent.")     # HIGH
tts.speak_critical("Obstacle detected. Stopping.") # CRITICAL — interrupts

tts.set_volume(70)      # 0–100 %
tts.set_speed(130)      # words per minute
print(tts.is_speaking)  # True while utterance plays
print(tts.queue_size)   # items waiting

tts.stop()              # graceful shutdown
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| No sound, no error | Wrong ALSA device name | Run `aplay -l` and update `alsa_device` in config |
| Left speaker only | SD_MODE wiring on right amp | Check 100 KΩ pull-down to GND on RIGHT SD_MODE |
| Right speaker only | SD_MODE wiring on left amp | Check 100 KΩ pull-up to 3.3 V on LEFT SD_MODE |
| Crackling / noise | Ground loop | Ensure both GND pins share a common ground with Jetson |
| `aplay` not found | alsa-utils missing | `sudo apt install -y alsa-utils` |
| `espeak-ng` not found | espeak-ng missing | `sudo apt install -y espeak-ng` |
| TTS disabled at boot | `enabled: false` in config | Set `audio.enabled: true` in `config.yaml` |
| Very low volume | ALSA mixer or amp gain | Run `amixer` to raise volume; check GAIN pin on amp |
