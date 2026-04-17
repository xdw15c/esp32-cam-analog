# Modbus TCP map (ESP32-CAM)

ESP32-CAM wystawia dane jako Modbus TCP server na porcie `502`.

- Protocol: Modbus TCP
- Port: `502`
- Unit ID (slave): `1`
- Obsłużona funkcja: `0x03` (Read Holding Registers)
- Addressing: obsługiwane są trzy style
  - zero-based: `0..15`
  - 4xxxx od 40001: `40001..40016`
  - 4xxxx od 40000: `40000..40015`

Przykłady poprawnych zapytań FC03 dla całej mapy (16 rejestrów):

- start `0`, qty `16`
- start `40001`, qty `16`
- start `40000`, qty `16`

## Holding Registers

| Modbus addr | Rejestr | Opis | Typ / skala |
|---|---|---|---|
| 40001 (0) | `status` | 0=ok, 11=g1_not_detected, 12=g2_not_detected, 13=both_not_detected | `uint16` |
| 40002 (1) | `g1_x100` | Odczyt manometru 1 | `int16`, value = reg / 100 |
| 40003 (2) | `g2_x100` | Odczyt manometru 2 | `int16`, value = reg / 100 |
| 40004 (3) | `diff_x100` | Różnica `g1-g2` | `int16`, value = reg / 100 |
| 40005 (4) | `conf1_x1000` | Confidence dla M1 | `uint16`, conf = reg / 1000 |
| 40006 (5) | `conf2_x1000` | Confidence dla M2 | `uint16`, conf = reg / 1000 |
| 40007 (6) | `uptime_lo` | Uptime sekundy (low word) | `uint16` |
| 40008 (7) | `uptime_hi` | Uptime sekundy (high word) | `uint16` |
| 40009 (8) | `wifi_rssi` | RSSI Wi-Fi dBm, signed | `int16` |
| 40010 (9) | `heartbeat` | Licznik heartbeat (co ~10 s) | `uint16` |
| 40011 (10) | `fault_code` | Bitmask błędów: 1=storage, 2=camera | `uint16` |
| 40012 (11) | `analysis_source` | 0=unavailable, 1=camera_live, 2=sd_photo | `uint16` |
| 40013..40016 (12..15) | `reserved` | Rezerwa | `uint16` |

## Uwagi

- Wartość `-32768` (`0x8000`) w rejestrach signed oznacza brak poprawnej detekcji.
- `uptime_s` można złożyć jako:
  - `uptime = (uptime_hi << 16) | uptime_lo`
- `fault_code` można traktować jako maskę alarmową dla systemu nadrzędnego.
  - `0` = brak błędu krytycznego
  - `1` = błąd storage / karta SD
  - `2` = błąd kamery
  - `3` = jednoczesny błąd storage i kamery
- `analysis_source=2` oznacza tryb offline, w którym analiza działa na pliku `/latest.jpg` z karty SD.
- Zapytania z innym Unit ID niż `1` są ignorowane (brak odpowiedzi).
- Mapę można rozszerzać bez zmiany portu/protokołu.
