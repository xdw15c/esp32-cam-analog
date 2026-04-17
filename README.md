# ESP32-CAM Manometry

## Cel projektu

Zbudować urządzenie oparte o ESP32-CAM, które:

1. Odczytuje wskazania **dwóch manometrów analogowych** widocznych w kadrze kamery.
2. Wyznacza wartości obu manometrów oraz różnicę `manometr_1 - manometr_2`.
3. Udostępnia wyniki lokalnie przez panel WWW i Modbus TCP.

## Projekty referencyjne (punkt wyjścia)

1. Artykuł Circuits Ninja:
  - https://circuits-ninja.pl/odczyt-wskazania-z-analogowego-manometru-za-pomoca-modulu-esp32-cam-z-kamera-ov2640-i-opencv/
  - Wykorzystujemy jako inspirację do podejścia opartego o analizę obrazu i mapowanie kąta wskazówki na wartość.

2. Repozytorium AI on the edge device:
  - https://github.com/jomjol/AI-on-the-edge-device
  - Traktujemy jako referencję architektury i doświadczeń praktycznych z odczytem analogowych liczników.

## Decyzja projektowa

- Projekt ma działać **w całości na ESP32-CAM**.
- AI/ML **nie jest wymagane** w MVP.
- Preferowane podejście: klasyczna analiza obrazu (CV), czyli wyznaczanie kąta wychylenia wskazówki i przeliczenie na jednostki ciśnienia po kalibracji.
- Rozwiązania AI mogą być rozważone dopiero opcjonalnie, jeśli klasyczne CV okaże się niewystarczające.

## Założenia funkcjonalne (MVP)

- Kamera obserwuje oba manometry w **stałym położeniu**.
- Odczyt wykonywany cyklicznie.
- Wyniki dostępne przez HTTP JSON i Modbus TCP.
- W przypadku braku poprawnego odczytu publikowany jest status błędu.

## Założenia techniczne

- Platforma: **ESP32-CAM** (OV2640).
- Firmware: C++ (PlatformIO / Arduino framework).
- Komunikacja: Wi-Fi + HTTP + Modbus TCP.
- Przetwarzanie obrazu na urządzeniu, bez konieczności serwera pośredniego.
- Priorytet dla lekkich algorytmów CV możliwych do uruchomienia na ESP32-CAM (bez wymogu modeli AI).

## Aktualny tryb pracy

Obecna wersja firmware ma dwa istotne tryby architektoniczne:

- normalny tryb kamery: analiza na żywej klatce z OV2640,
- tymczasowy tryb offline z SD: analiza na istniejącym pliku `/latest.jpg` zapisanym na karcie SD.

Na ten moment aktywny jest tryb offline z SD. Kamera nie jest inicjalizowana, a cała analityka działa na obrazie JPEG wczytanym z karty i dekodowanym do bufora RGB565.

## Założenia dot. wizji komputerowej

- Dla każdego manometru definiujemy osobny ROI (obszar obrazu).
- Wskaźnik manometru wykrywany na podstawie kontrastu/krawędzi i geometrii wskazówki.
- Kąt wskazówki mapowany na wartość ciśnienia wg kalibracji:
  - `kat_min -> wartosc_min`,
  - `kat_max -> wartosc_max`.
- Dla każdego manometru osobna kalibracja (zakres, offset, orientacja).

## Model danych MQTT (propozycja)

Topic:

- `esp-cam/manometry/readings`

Payload JSON:

```json
{
  "device_id": "esp32cam-01",
  "ts": 1710000000,
  "gauge_1": 3.42,
  "gauge_2": 2.95,
  "difference": 0.47,
  "unit": "bar",
  "status": "ok"
}
```

Statusy przykładowe:

- `ok`
- `gauge_1_not_detected`
- `gauge_2_not_detected`
- `both_not_detected`
- `calibration_missing`

## Konfiguracja (do przygotowania)

- Dane Wi-Fi (`ssid`, `password`).
- MQTT (`host`, `port`, `user`, `password`, `topic`).
- Konfiguracja manometrów:
  - ROI 1 i ROI 2,
  ## Model danych HTTP JSON (aktualny)

  Endpoint:

  - `POST /analyze`

  Przykładowa odpowiedź:
- Urządzenie łączy się z Wi-Fi.
- Urządzenie publikuje dane do MQTT.
- Dla statycznego testu dwóch manometrów odczyt obu wartości jest stabilny.
- Różnica jest poprawnie liczona i publikowana.
    "status": "ok",
    "source": "sd_photo",
    "frame": {
      "width": 1024,
      "height": 768
    },
    "processing_ms": 184,
    "gauges": [
      {
        "id": 1,
        "name": "Manometr 1",
        "unit": "bar",
        "analysis_mode": "color_target",
        "detected": true,
        "angle_deg": 27.4,
        "value": 3.42,
        "confidence": 0.81,
        "darkness": 92.3
      },
      {
        "id": 2,
        "name": "Manometr 2",
        "unit": "bar",
        "analysis_mode": "color_target",
        "detected": true,
        "angle_deg": 15.9,
        "value": 2.95,
        "confidence": 0.77,
        "darkness": 98.1
      }
    ]
  }
  ```

  Statusy przykładowe:

  - `ok`
  - `gauge_1_not_detected`
  - `gauge_2_not_detected`
  - `both_not_detected`
  - `calibration_missing_or_invalid`

  ## Konfiguracja

  - Dane Wi-Fi (`ssid`, `password`).
  - Dane logowania do panelu WWW (`WEB_USERNAME`, `WEB_PASSWORD`).
  - Konfiguracja manometrów:
    - środek tarczy,
    - promień,
    - kąty min/max,
    - zakres wartości,
    - jednostka,
    - tryb analizy,
    - kolory dla trybu `color_target`.
  - Interwał analizy.

  Konfiguracja robocza zapisywana jest na SD jako `config/config.json`.

  ## Panel WWW

  Urządzenie udostępnia panel WWW z Basic Auth. Aktualnie dostępne są zakładki:

  - `Kamera`: podgląd ostatniego zdjęcia z SD i ręczne uruchomienie analizy,
  - `Kalibracja ROI`: edycja parametrów obu manometrów i zapis konfiguracji,
  - `Modbus`: podgląd stanu i rejestrów Modbus TCP,
  - `Logi`: podgląd końcówki pliku `logs/system.log` i czyszczenie logu.

  W trybie offline przyciski związane z kamerą operują na zdjęciu już zapisanym na SD, a nie na nowo wykonanej klatce.

  ## Modbus TCP

  Urządzenie udostępnia serwer Modbus TCP na porcie `502`.

  - Rejestry z wynikami zawierają dwa odczyty, różnicę, confidence, uptime, RSSI, heartbeat.
  - Dodatkowo wystawiany jest `fault_code`, dzięki któremu system nadrzędny może zgłosić alarm kamery lub pamięci.
  - Wystawiane jest również `analysis_source`, aby odróżnić analizę z kamery od analizy z pliku na SD.

  Szczegóły mapy rejestrów są w pliku `README_MODBUS.md`.

  ## Logi i diagnostyka

  Firmware zapisuje log do pliku `logs/system.log` na karcie SD.

  - Log obejmuje start urządzenia, aktywację trybu awaryjnego, zapis konfiguracji oraz start usług WWW.
  - Ten sam log jest widoczny z poziomu zakładki `Logi`.
  - W przypadku problemów z kamerą lub storage można jednocześnie diagnozować stan po wzorcu LED, logu WWW i rejestrach Modbus.

  ## Status aktualny

  Aktualnie działa stabilna baza funkcjonalna:

  - Wi-Fi,
  - panel WWW z Basic Auth,
  - zapis i odczyt konfiguracji z SD,
  - analiza dwóch manometrów w kilku trybach (`classic_darkness`, `color_target`, `hybrid_pca_hough`),
  - cykliczna analiza obrazu,
  - Modbus TCP,
  - logowanie do pliku tekstowego na SD i podgląd logu przez WWW,
  - tryb offline z analizą istniejącego zdjęcia z SD,
  - sygnalizacja błędu kamery/storage przez status, log i fault code.

  ## Ograniczenia i ryzyka

  - Obecna wersja pracuje tymczasowo bez żywej kamery i bazuje na jednym zdjęciu z SD.
  - Jakość analizy nadal zależy od jakości i rozdzielczości zdjęcia referencyjnego.
  - ESP32-CAM ma ograniczone zasoby RAM/CPU, więc rozbudowa algorytmów musi pozostać oszczędna pamięciowo.
  - Po przywróceniu sprawnej kamery trzeba będzie ponownie zweryfikować stabilność trybu live.

  ## Szybkie uruchomienie

  1. Uzupełnij dane w `include/secrets.h`.
  2. Wgraj firmware na ESP32-CAM.
  3. Włóż kartę SD z plikiem `/latest.jpg`.
  4. Odczytaj adres IP z logu lub z routera.
  5. Wejdź na panel WWW, zaloguj się i uruchom analizę.

  ## Roadmap

  Szczegółowy plan dalszych etapów znajduje się w pliku `ROADMAP.md`.
