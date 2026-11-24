# ColorMix – Custom Paint Mixer (Web + QR + Node-RED + MQTT + ESP32 + Grafana)

```
 ┌──────────────┐  QR (JSON RGB)  ┌───────────────┐  HTTP POST /api/order  ┌───────────┐
 │ Web Generator│ ──────────────▶ │ Web Scanner   │ ─────────────────────▶ │  Node-RED │
 │ index.html   │   print / PNG   │ kamera laptop │ ◀── GET /api/status ── │           │
 └──────────────┘                 └───────────────┘                        └─────┬─────┘
                                       MQTT colormix/order (dose ml) ┌───────────┤
                                                                     ▼           ▼
                                                              ┌──────────┐  ┌──────────┐   ┌─────────┐
                                                              │  ESP32   │  │ InfluxDB │──▶│ Grafana │
                                                              │  pompa   │  └──────────┘   └─────────┘
                                                              └────┬─────┘        ▲
                                                                   └─ MQTT colormix/status ─┘ (via Node-RED)
```

## Struktur folder

| Path | Isi |
|---|---|
| `web/index.html` | Generator: pilih warna (picker/slider/preset), nama, volume → QR code (PNG/JSON/print) |
| `web/scan.html` | Scanner: kamera laptop (jsQR), validasi, kirim ke Node-RED, tampilkan status ESP32 |
| `web/common.js` | Helper bersama: validasi JSON + rumus RGB → takaran cat Merah/Hijau/Biru |
| `node-red/data/flows.json` | Flow: API order/status, MQTT, InfluxDB, simulator ESP32 |
| `esp32/ColorMixer/ColorMixer.ino` | Firmware ESP32 (3 pompa relay + stirrer + LED RGB) |
| `grafana/provisioning/` | Datasource & dashboard Grafana otomatis |
| `docker-compose.yml` | Mosquitto, Node-RED, InfluxDB 1.8, Grafana |

## Menjalankan

```powershell
docker compose up -d --build
```

| Layanan | URL |
|---|---|
| Generator warna | http://localhost:1880/ |
| Scanner | http://localhost:1880/scan.html |
| Node-RED editor | http://localhost:1880/admin |
| Grafana | http://localhost:3000 (admin / admin) |
| MQTT broker | `<IP-laptop>:1883` |

> **Bentrok port?** Kalau ada stack lain yang memakai 1880/1883/3000, matikan dulu stack itu
> atau pakai port lain:
> `$env:MQTT_PORT='1884'; $env:NODERED_PORT='1881'; $env:GRAFANA_PORT='3001'; docker compose up -d`

Kamera hanya bisa dibuka di **secure context**, jadi buka scanner lewat `http://localhost:...`,
bukan lewat IP (untuk akses dari HP/laptop lain butuh HTTPS).

## Format JSON di QR code

```json
{ "v": 1, "id": "CM-20261117-AB12", "name": "Biru Laut", "r": 30, "g": 110, "b": 200, "vol": 250 }
```

Node-RED menambahkan takaran pigmen lalu publish ke `colormix/order`:

```json
{ "id": "CM-20261117-AB12", "name": "Biru Laut", "r": 30, "g": 110, "b": 200, "hex": "#1E6EC8",
  "vol": 250, "dose": { "red": 22.1, "green": 80.9, "blue": 147.1 }, "ts": 1794916331162 }
```

**Rumus takaran** (3 cat dasar: Merah, Hijau, Biru): tiap cat diberi bagian sebanding dengan nilai R:G:B,
`ml_merah = vol × R / (R+G+B)` (begitu juga hijau dan biru). RGB(0,0,0) dicampur rata.
Rumus ini ada di `web/common.js` **dan** di function node Node-RED "Validate & hitung dosis",
jadi kalau diubah, ubah keduanya.

Keterbatasan: cat bercampur secara *subtraktif*, jadi mencampur merah, hijau, dan biru cenderung
menghasilkan warna gelap/kecokelatan. Warna terang (putih, kuning muda) tidak bisa dibuat tanpa cat
putih. Rumus ini mengikuti rasio RGB; hasil fisiknya perlu dikalibrasi dengan sampel.

## Topik MQTT

| Topik | Arah | Payload |
|---|---|---|
| `colormix/order` | Node-RED → ESP32 | order + `dose` (ml) |
| `colormix/status` | ESP32 → Node-RED (retained) | `{"device","state","order","progress","step","rssi"}`; `state` = idle / mixing / tuning / priming / done / error / offline (LWT) |
| `colormix/cmd` | → ESP32 | `{"cmd":"stop"}` untuk berhenti darurat; perintah S3 lain lihat di bawah |
| `colormix/telemetry` | ESP32-S3 → Node-RED | tiap 0.5 s saat TRIM: chroma target `tr/tg/tb`, terukur `yr/yg/yb`, RGB sensor `mr/mg/mb`, ml trim `ur/ug/ub`, `err` |
| `colormix/tune` | ESP32-S3 → Node-RED | hasil Cohen-Coon `{"type":"tune","pump","K","tau","theta","kp","Ti","Td","curve":[..]}` atau `{"type":"info",...}` |

API Node-RED: `POST /api/order` → 200 ok, 400 invalid, 409 `busy` (mixer sedang jalan/tuning) atau
`duplicate` (QR sudah pernah diproses; scanner menawarkan kirim ulang). `GET /api/status` → status ESP32 terakhir.

## ESP32-S3 + sensor warna + PID Cohen-Coon (`esp32/ColorMixerS3`)

Versi closed-loop: sensor warna (TCS34725 atau TCS3200) mengukur campuran, lalu satu PID per pompa
memutuskan pompa **Merah/Hijau/Biru** mana yang perlu ditambah dan berapa ml.

1. **BASE**: tuang 80% takaran dari Node-RED. **PREMIX**: aduk 8 s.
2. **TRIM**: pengaduk jalan, sensor dibaca tiap 100 ms. Error per channel = chroma target − terukur,
   dengan chroma = R/(R+G+B). PID → total ml tambahan per pompa; pompa yang paling kurang dinyalakan per pulsa ≤ 2 ml.
3. **DONE** saat semua error < 1% selama 4 s, atau jatah koreksi (20% + `OVERFILL` 10%) habis, atau timeout.

Cara pakai (tombol inject sudah ada di Node-RED, bagian ⑤):

| Langkah | Perintah `colormix/cmd` | Keterangan |
|---|---|---|
| Kalibrasi sensor | `{"cmd":"calwhite"}` lalu `{"cmd":"calblack"}` | sensor menghadap kertas putih, lalu hitam/gelap |
| Isi selang pompa | `{"cmd":"prime","pump":"red","ms":3000}` | supaya takaran pertama tidak kosong |
| Tuning Cohen-Coon | `{"cmd":"tune","pump":"red","ml":5,"vol":250,"ms":20000}` | gelas berisi ±250 ml campuran; ulangi untuk green & blue |
| Isi model manual | `{"cmd":"model","pump":"red","K":0.0024,"vol":250,"tau":3,"theta":1.5}` | kalau uji step dilakukan sendiri |
| Lihat parameter | `{"cmd":"info"}` | hasil muncul di debug Node-RED |

Rumus Cohen-Coon yang dipakai (r = θ/τ): `Kp = (1/K)(1/r)(4/3 + r/4)`, `Ti = θ(32+6r)/(13+8r)`,
`Td = 4θ/(11+2r)`, dengan τ dan θ dari metode dua titik (t28, t63). Gain dikali `DETUNE = 0.5`
karena cat yang sudah tertuang tidak bisa diambil lagi (overshoot tidak bisa dikoreksi).
Parameter tersimpan di flash (Preferences), jadi tidak hilang saat ESP32 restart.

Board: **ESP32S3 Dev Module**, *USB CDC On Boot: Enabled*. Library: PubSubClient, ArduinoJson 7,
**Adafruit TCS34725** (kalau pakai TCS3200: set `USE_TCS34725 0`, tidak perlu library ini).
Pin default: pompa 4/5/6, pengaduk 7, I2C SDA 8 / SCL 9 (TCS3200: S0–S3 10–13, OUT 14), LED 15/16/17.

## ESP32 biasa, open-loop (`esp32/ColorMixer`)

1. Arduino IDE: board **ESP32 Dev Module** (core 2.x/3.x), install library **PubSubClient** dan **ArduinoJson 7**.
2. Edit bagian `CONFIG` di `ColorMixer.ino`: WiFi, `MQTT_HOST` = IP laptop (`ipconfig`), pin relay, `ML_PER_SEC`.
3. **Kalibrasi pompa:** nyalakan tiap pompa 10 detik, ukur ml-nya, lalu isi `ML_PER_SEC = ml / 10`.
4. Izinkan port 1883 di Windows Firewall (ESP32 dan laptop harus di jaringan WiFi yang sama).

Default pin: pompa Merah/Hijau/Biru = 25/26/27, stirrer = 19, LED RGB = 16/17/18, relay active-LOW.

## Demo tanpa ESP32

Di Node-RED editor, buka tab **Simulator ESP32** → double-click tab → *Enabled* → Deploy.
Simulator membalas setiap order dengan status mixing → done → idle. Matikan lagi saat ESP32 asli dipakai.

Tes manual dengan CLI:

```bash
docker exec colormix-mosquitto mosquitto_sub -t 'colormix/#' -v
docker exec colormix-mosquitto mosquitto_pub -r -t colormix/status -m '{"device":"esp32-mixer","state":"idle","progress":0}'
```

## Dashboard Grafana

Total order, total ml, status ESP32, progress, RSSI WiFi, grafik R/G/B per order,
total pigmen terpakai, tabel order terbaru, riwayat progress, grafik PID (chroma target vs terukur,
ml trim per pompa), dan tabel hasil tuning Cohen-Coon. Dashboard bisa diedit di UI.
Kalau mau permanen, export JSON-nya ke `grafana/provisioning/dashboards/colormix.json`.
