# SƠ ĐỒ KẾT NỐI A4988 + NEMA 17 + JETSON NANO

## 📌 TỔNG QUAN KẾT NỐI

```
┌─────────────┐         ┌──────────────┐         ┌──────────────┐
│             │         │              │         │              │
│   NGUỒN     │────────▶│    A4988     │────────▶│   NEMA 17    │
│   12V       │         │   DRIVER     │         │    MOTOR     │
│             │         │              │         │              │
└─────────────┘         └──────────────┘         └──────────────┘
                              ▲
                              │
                              │ (STEP, DIR, EN)
                              │
                        ┌─────┴─────┐
                        │           │
                        │  JETSON   │
                        │   NANO    │
                        │           │
                        └───────────┘
```

---

## 🔌 CHI TIẾT KẾT NỐI

### 1. NGUỒN ĐIỆN (12V Power Supply)

```
Power Supply          A4988 Driver
┌──────────┐         ┌──────────┐
│          │         │          │
│   +12V   ├────────▶│  VMOT    │  (Nguồn cho motor)
│          │         │          │
│   GND    ├────┬───▶│  GND     │  (Mass chung)
│          │    │    │          │
└──────────┘    │    └──────────┘
                │
                │    Jetson Nano
                │    ┌──────────┐
                │    │          │
                └───▶│  GND     │  (GND chung!)
                     │          │
                     └──────────┘

⚠️ QUAN TRỌNG: GND của Jetson và A4988 PHẢI nối chung!
⚠️ Tụ điện 100μF nên gắn gần VMOT và GND của A4988 (giảm nhiễu)
```

### 2. JETSON NANO → A4988 (Control Signals)

```
Jetson Nano GPIO     A4988 Driver
┌──────────────┐    ┌──────────┐
│              │    │          │
│  Pin 33      ├───▶│  STEP    │  (Xung bước)
│  (GPIO 13)   │    │          │
│              │    │          │
│  Pin 35      ├───▶│  DIR     │  (Hướng quay)
│  (GPIO 19)   │    │          │
│              │    │          │
│  Pin 37      ├───▶│  ENABLE  │  (Bật/tắt driver)
│  (GPIO 26)   │    │          │
│              │    │          │
│  GND         ├───▶│  GND     │  (GND chung)
│              │    │          │
└──────────────┘    └──────────┘

OPTIONAL: MS pins để điều khiển microstepping
┌──────────────┐    ┌──────────┐
│              │    │          │
│  Pin 11      ├───▶│  MS1     │  (Microstepping bit 1)
│  (GPIO 17)   │    │          │
│              │    │          │
│  Pin 13      ├───▶│  MS2     │  (Microstepping bit 2)
│  (GPIO 27)   │    │          │
│              │    │          │
│  Pin 15      ├───▶│  MS3     │  (Microstepping bit 3)
│  (GPIO 22)   │    │          │
└──────────────┘    └──────────┘
```

### 3. A4988 → NEMA 17 MOTOR

```
A4988 Driver         NEMA 17 Motor
┌──────────┐        ┌──────────────┐
│          │        │              │
│   1B     ├───────▶│  Coil A1     │ (Thường đen hoặc xanh)
│          │        │              │
│   1A     ├───────▶│  Coil A2     │ (Thường xanh hoặc đỏ)
│          │        │              │
│   2A     ├───────▶│  Coil B1     │ (Thường đỏ hoặc vàng)
│          │        │              │
│   2B     ├───────▶│  Coil B2     │ (Thường vàng hoặc trắng)
│          │        │              │
└──────────┘        └──────────────┘

❓ CÁCH XÁC ĐỊNH COIL PAIRING:
Dùng đồng hồ đo điện trở (ohm):
- Đo giữa 2 dây bất kỳ
- Nếu điện trở ~1-4Ω → 2 dây cùng coil
- Nếu điện trở vô cùng (∞) → khác coil

Ví dụ motor 4 dây: BLACK, GREEN, RED, BLUE
- BLACK ↔ GREEN = 3Ω → Coil A
- RED ↔ BLUE = 3Ω → Coil B
- BLACK ↔ RED = ∞ → Khác coil
```

---

## 🎚️ MICROSTEPPING PINS (MS1, MS2, MS3)

### Option A: Nối cứng (Đơn giản - KHUYẾN NGHỊ)

```
MS1, MS2, MS3 → GND        = Full Step (1/1)
MS1, MS2, MS3 → VDD (5V)   = 1/16 Step (êm nhất)
```

**Sơ đồ nối cho 1/16 step:**
```
A4988
┌─────────────────┐
│                 │
│  MS1  ─────┐    │
│  MS2  ─────┼───▶│ VDD (3.3V hoặc 5V từ A4988)
│  MS3  ─────┘    │
│                 │
└─────────────────┘
```

**Sơ đồ nối cho Full step:**
```
A4988
┌─────────────────┐
│                 │
│  MS1  ─────┐    │
│  MS2  ─────┼───▶│ GND
│  MS3  ─────┘    │
│                 │
└─────────────────┘
```

### Option B: Điều khiển qua GPIO (Linh hoạt)

```
Jetson Pin 11 (GPIO 17) → MS1
Jetson Pin 13 (GPIO 27) → MS2
Jetson Pin 15 (GPIO 22) → MS3
```

**Bảng cấu hình:**
```
┌─────┬─────┬─────┬────────────────┬───────────────┐
│ MS1 │ MS2 │ MS3 │ Microstepping  │ Steps/Rev     │
├─────┼─────┼─────┼────────────────┼───────────────┤
│  L  │  L  │  L  │ Full Step      │ 200           │
│  H  │  L  │  L  │ Half Step      │ 400           │
│  L  │  H  │  L  │ 1/4 Step       │ 800           │
│  H  │  H  │  L  │ 1/8 Step       │ 1,600         │
│  H  │  H  │  H  │ 1/16 Step      │ 3,200         │
└─────┴─────┴─────┴────────────────┴───────────────┘

L = LOW (0V)
H = HIGH (3.3V hoặc 5V)
```

---

## ⚙️ LAYOUT A4988 DRIVER

```
         ┌─────────────────────────────┐
         │                             │
  ENABLE ●                             ● VMOT (12V input)
         │                             │
    MS1  ●     ┌───────────┐           ● GND (Power ground)
         │     │           │           │
    MS2  ●     │  A4988    │  [POT]    ● 2B (Motor coil B wire 2)
         │     │   CHIP    │   ↑       │
    MS3  ●     │           │   │       ● 2A (Motor coil B wire 1)
         │     └───────────┘   │       │
   RESET ●                  VREF       ● 1A (Motor coil A wire 1)
         │        adjust               │
   SLEEP ●                             ● 1B (Motor coil A wire 2)
         │                             │
    STEP ●                             ● VDD (Logic power 3.3-5V)
         │                             │
     DIR ●                             ● GND (Logic ground)
         │                             │
         └─────────────────────────────┘

[POT] = Potentiometer để điều chỉnh VREF
        Vặn bằng tuốc nơ vít nhỏ
        ↻ (phải) = tăng current
        ↺ (trái) = giảm current
```

---

## 🔧 CHECKLIST KẾT NỐI

### ✅ Nguồn điện:
- [ ] 12V power supply kết nối với VMOT và GND của A4988
- [ ] Tụ điện 100μF (electrolytic) gắn gần VMOT và GND
- [ ] GND của Jetson Nano nối chung với GND của A4988

### ✅ Control signals:
- [ ] Jetson Pin 33 → A4988 STEP
- [ ] Jetson Pin 35 → A4988 DIR  
- [ ] Jetson Pin 37 → A4988 ENABLE
- [ ] Jetson GND → A4988 GND

### ✅ Microstepping:
- [ ] MS1, MS2, MS3 được nối (GND hoặc VDD hoặc GPIO)
- [ ] KHÔNG để MS pins floating (không cắm gì)
- [ ] Config trong code khớp với hardware

### ✅ Motor:
- [ ] 4 dây motor đã xác định được 2 coils
- [ ] Coil A (2 dây) → A4988 pins 1A, 1B
- [ ] Coil B (2 dây) → A4988 pins 2A, 2B

### ✅ VREF:
- [ ] Đã đo VREF với multimeter
- [ ] VREF đặt đúng theo motor rated current
- [ ] Ví dụ: 1.5A motor → VREF ~ 0.42V

---

## 🧪 KIỂM TRA NHANH

### Test 1: Kiểm tra nguồn
```bash
# Dùng multimeter đo:
# - Giữa VMOT và GND của A4988: phải có ~12V
# - Giữa VDD và GND của A4988: phải có ~5V (tự cấp từ chip)
```

### Test 2: Kiểm tra coil
```bash
# Tắt nguồn
# Dùng multimeter (chế độ Ohm):
# - Đo giữa 2 dây của coil A: ~1-4Ω
# - Đo giữa 2 dây của coil B: ~1-4Ω
# - Đo giữa coil A và B: ∞ (vô cùng)
```

### Test 3: Kiểm tra motor holding
```bash
# Bật nguồn A4988
# Set ENABLE = LOW (enable motor)
# Motor shaft phải "khóa cứng" (khó xoay bằng tay)
# Nếu không khóa → check VREF hoặc wiring
```

### Test 4: Test bước đơn
```bash
sudo python3 test_motor_simple.py
# Motor phải quay 1 vòng mượt mà
# Không có tiếng "è è" hay rung
```

---

## 🐛 TROUBLESHOOTING NHANH

| Triệu chứng | Nguyên nhân | Giải pháp |
|-------------|-------------|-----------|
| Motor kêu "è è", rung mạnh | MS pins floating hoặc sai | Nối MS pins vào GND hoặc VDD |
| Motor không quay, chỉ rung | Coil pairing sai | Đổi chỗ 2 dây của 1 coil |
| Motor yếu, không giữ được | VREF quá thấp | Tăng VREF lên |
| Motor nóng quá | VREF quá cao | Giảm VREF xuống |
| A4988 nóng, shutdown | Ngắn mạch hoặc VREF cao | Kiểm tra wiring, giảm VREF |
| Motor quay ngược hướng | DIR signal đảo | Đổi DIR logic hoặc đổi coil |
| Motor skip steps | Tốc độ quá nhanh hoặc VREF thấp | Giảm tốc độ, tăng VREF |

---

## 📚 TÀI LIỆU THAM KHẢO

- A4988 Datasheet: https://www.pololu.com/file/0J450/a4988_DMOS_microstepping_driver_with_translator.pdf
- NEMA 17 Specs: Thường 200 steps/rev, 1.8° per step
- Jetson Nano GPIO Pinout: https://jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/

---

## 🔴 LƯU Ý AN TOÀN

⚠️ **QUAN TRỌNG:**
1. Luôn TẮT NGUỒN trước khi đấu nối/thay đổi wiring
2. Không chạm tay vào A4988 khi đang hoạt động (có thể nóng)
3. Kiểm tra kỹ không có ngắn mạch trước khi bật nguồn
4. Nếu A4988 quá nóng (>80°C), tắt nguồn ngay và kiểm tra lại
5. Gắn heatsink (tản nhiệt) lên A4988 nếu chạy continuous
6. VREF tối đa không nên vượt quá rated current của motor

---

**FILE NÀY LÀ TÀI LIỆU THAM KHẢO - KHÔNG PHẢI CODE**

Để bắt đầu chẩn đoán, chạy:
```bash
python3 calculate_vref.py        # Tính VREF cần thiết
sudo python3 test_motor_simple.py # Test motor
```

Đọc chi tiết: **A4988_DIAGNOSTIC.md**
