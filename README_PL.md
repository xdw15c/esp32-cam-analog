# ESP32-CAM Manometry

## Co to jest

Firmware dla ESP32-CAM, ktory odczytuje dwa manometry analogowe z obrazu i udostepnia wyniki przez:

- panel WWW (HTTP)
- Modbus TCP (FC03)

## Aktualny stan projektu

Wersja jest dzialajaca i utrzymywana.
Najwazniejsze funkcje, ktore sa juz wdrozone:

- analiza 2 manometrow w jednej klatce
- 3 tryby analizy:
  - classic_darkness
  - color_target
  - hybrid_pca_hough
- kalibracja ROI i parametrow z poziomu WWW
- zapis konfiguracji na SD: /config/config.json
- log systemowy na SD: /logs/system.log
- zakladka Logi w WWW (podglad + czyszczenie)
- Modbus TCP z mapa 16 rejestrow
- fault_code i analysis_source w Modbus
- tryb offline SD (analiza z /latest.jpg, bez inicjalizacji kamery)
- upload JPEG z WWW do /latest.jpg
- debug decyzji analizy w JSON (powod wyboru/odrzucenia kata)

## Tryby zrodla obrazu

Firmware obsluguje 3 praktyczne scenariusze:

1. Kamera live
- docelowy tryb produkcyjny po naprawie hardware kamery

2. Plik z SD (/latest.jpg)
- aktywny tryb awaryjny/offline
- kamera nie jest wymagana

3. Upload z WWW
- przeslany plik JPEG jest zapisywany jako /latest.jpg
- dalsza analiza dziala jak dla zrodla SD

## API HTTP

Najwazniejsze endpointy:

- GET /
- POST /capture
- POST /analyze
- GET /photo.jpg
- GET /config
- POST /config
- GET /modbus/status
- GET /logs
- POST /logs/clear
- POST /upload

## Odpowiedz /analyze (skrot)

Przyklad (skrocony):

```json
{
  "status": "ok",
  "source": "sd_photo",
  "gauges": [
    {
      "id": 1,
      "detected": true,
      "angle_deg": 27.4,
      "value": 3.42,
      "confidence": 0.81,
      "debug": {
        "pointer_candidates": 2,
        "selected_score": 96.1,
        "opposite_score": 71.5,
        "selected_reason": "selected_best_color_score"
      }
    }
  ],
  "debug": {
    "detected_gauges": 2,
    "pointer_candidates_total": 3
  }
}
```

## Panel WWW

Zakladki:

- Kamera
- Kalibracja ROI
- Modbus
- Logi

W zakladce Modbus jest teraz:

- podglad surowych rejestrow
- opis mapy rejestrow
- dekodowanie kodow status/fault/source

## Szybkie uruchomienie

1. Uzupelnij include/secrets.h.
2. Wgraj firmware.
3. Wloz SD i przygotuj /latest.jpg (lub uzyj uploadu z WWW).
4. Wejdz na panel WWW i uruchom analize.

## Uwagi

- MQTT nie jest obecnie glowna sciezka integracji. Biezaca integracja operacyjna to WWW + Modbus TCP.
- Szczegoly mapy rejestrow: README_MODBUS.md
- Dalsze kroki: ROADMAP.md
