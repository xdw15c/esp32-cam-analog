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
- runtime wybor zrodla analizy (auto / camera / sd_photo)
- upload JPEG z WWW do /latest.jpg
- debug decyzji analizy w JSON (powod wyboru/odrzucenia kata)

## Tryby zrodla obrazu

Firmware obsluguje 3 praktyczne scenariusze w jednym buildzie:

1. auto
- rekomendowany tryb produkcyjny
- najpierw probuje kamere live
- przy niedostepnej kamerze przechodzi na /latest.jpg z SD

2. camera
- wymusza analize tylko z kamery live
- przydatne po naprawie hardware i do testow live

3. sd_photo
- wymusza analize tylko z /latest.jpg
- najlepsze do debugowania i testow na obrazach z komputera

Upload z WWW zawsze zapisuje obraz do /latest.jpg.

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
  "analysis_input_mode": "auto",
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

W ustawieniach urzadzenia dostepny jest wybor `analysis_input_source`:

- auto
- camera
- sd_photo

Wybor jest zapisywany do /config/config.json i odtwarzany po restarcie.

## Szybkie uruchomienie

1. Uzupelnij include/secrets.h.
2. Wgraj firmware.
3. Wloz karte SD.
4. Wejdz na panel WWW i wybierz tryb zrodla analizy (zalecane: auto).
5. W razie potrzeby wgraj obraz testowy przez upload (zapis do /latest.jpg).
6. Uruchom analize.

## Uwagi

- MQTT nie jest obecnie glowna sciezka integracji. Biezaca integracja operacyjna to WWW + Modbus TCP.
- Szczegoly mapy rejestrow: README_MODBUS.md
- Dalsze kroki: ROADMAP.md
