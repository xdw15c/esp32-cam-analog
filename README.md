# ESP32-CAM Manometry -> MQTT

## Cel projektu

Zbudować urządzenie oparte o ESP32-CAM, które:

1. Odczytuje wskazania **dwóch manometrów analogowych** widocznych w kadrze kamery.
2. Wyznacza i publikuje przez MQTT:
   - wartość manometru 1,
   - wartość manometru 2,
   - różnicę: `manometr_1 - manometr_2`.

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
- Odczyt wykonywany cyklicznie (np. co 1-5 s, wartość do ustalenia).
- Wyniki wysyłane przez MQTT w postaci JSON.
- W przypadku braku poprawnego odczytu publikowany jest status błędu.

## Założenia techniczne

- Platforma: **ESP32-CAM** (OV2640).
- Firmware: C++ (PlatformIO / Arduino framework).
- Komunikacja: Wi-Fi + MQTT (broker lokalny lub chmurowy).
- Przetwarzanie obrazu na urządzeniu, bez konieczności serwera pośredniego.
- Priorytet dla lekkich algorytmów CV możliwych do uruchomienia na ESP32-CAM (bez wymogu modeli AI).

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
  - zakresy wartości,
  - kąty min/max,
  - jednostka (np. bar).
- Interwał publikacji.

## Kryteria akceptacji etapu 1

- Urządzenie łączy się z Wi-Fi.
- Urządzenie publikuje dane do MQTT.
- Dla statycznego testu dwóch manometrów odczyt obu wartości jest stabilny.
- Różnica jest poprawnie liczona i publikowana.

## Ograniczenia i ryzyka

- Zmienna ekspozycja/odbicia światła mogą pogarszać detekcję wskazówki.
- Drgania kamery lub przesunięcie manometrów wymagają ponownej kalibracji.
- ESP32-CAM ma ograniczone zasoby RAM/CPU, co ogranicza złożoność algorytmów.

## Roadmap

Szczegółowy plan etapów, backlog oraz sposób śledzenia postępu znajduje się w pliku `ROADMAP.md`.

## Status MVP v0 (zaimplementowane)

Aktualnie działa prosta wersja startowa:

- ESP32-CAM wykonuje zdjęcie i zapisuje je na karcie SD jako `/latest.jpg`.
- Urządzenie udostępnia panel WWW z Basic Auth.
- W panelu można:
  - wykonać nowe zdjęcie,
  - wyświetlić ostatnie zdjęcie,
  - klikać punkty (oznaczenia) na obrazie,
  - zapisać punkty na SD jako `/markers.json`,
  - wczytać zapisane punkty.

Pliki implementacji:

- `src/main.cpp`
- `include/secrets.h`
- `include/secrets.h.example`
- `platformio.ini`

## Szybkie uruchomienie

1. Uzupełnij dane w `include/secrets.h` (Wi-Fi oraz login/hasło do panelu WWW).
2. Wgraj firmware na ESP32-CAM (`pio run -t upload`).
3. Otwórz monitor portu szeregowego (`pio device monitor`) i odczytaj adres IP.
4. Wejdź w przeglądarce na adres ESP32 i zaloguj się.
5. Kliknij "Zrob nowe zdjecie" i oznacz punkty na obrazie.
