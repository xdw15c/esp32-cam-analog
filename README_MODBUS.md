# Modbus TCP map (ESP32-CAM)

Firmware wystawia Modbus TCP server na porcie 502.

## Parametry protokolu

- Protocol: Modbus TCP
- Port: 502
- Unit ID: 1
- Function: 0x03 (Read Holding Registers)

Obslugiwane style adresowania:

- zero-based: 0..15
- 4xxxx od 40001: 40001..40016
- 4xxxx od 40000: 40000..40015

## Holding Registers

| Modbus addr | Rejestr | Opis | Typ / skala |
|---|---|---|---|
| 40001 (0) | status | 0=ok, 11=g1_not_detected, 12=g2_not_detected, 13=both_not_detected | uint16 |
| 40002 (1) | g1_x100 | Odczyt manometru 1 | int16, value=reg/100 |
| 40003 (2) | g2_x100 | Odczyt manometru 2 | int16, value=reg/100 |
| 40004 (3) | diff_x100 | Roznica g1-g2 | int16, value=reg/100 |
| 40005 (4) | conf1_x1000 | Confidence M1 | uint16, conf=reg/1000 |
| 40006 (5) | conf2_x1000 | Confidence M2 | uint16, conf=reg/1000 |
| 40007 (6) | uptime_lo | Uptime sekundy (low word) | uint16 |
| 40008 (7) | uptime_hi | Uptime sekundy (high word) | uint16 |
| 40009 (8) | wifi_rssi | RSSI Wi-Fi w dBm | int16 |
| 40010 (9) | heartbeat | Licznik heartbeat | uint16 |
| 40011 (10) | fault_code | Bitmask bledow | uint16 |
| 40012 (11) | analysis_source | Zrodlo analizy obrazu | uint16 |
| 40013..40016 (12..15) | reserved | Rezerwa | uint16 |

## Kody i znaczenie

### status (addr 0)

- 0: ok
- 11: gauge_1_not_detected
- 12: gauge_2_not_detected
- 13: both_not_detected

### fault_code (addr 10)

Bitmask:

- bit0 (1): storage_not_ready
- bit1 (2): camera_not_ready

Przyklady:

- 0: brak bledu
- 1: tylko storage
- 2: tylko kamera
- 3: storage + kamera

### analysis_source (addr 11)

- 0: unavailable
- 1: camera_live
- 2: sd_photo

## Dodatkowe uwagi

- Sentinel braku pomiaru (signed): -32768 (0x8000).
- Uptime skladamy jako: (uptime_hi << 16) | uptime_lo.
- Zapytania z Unit ID != 1 sa ignorowane.
- Zakladka Modbus w WWW pokazuje teraz opis mapy i dekodowanie kodow online.
