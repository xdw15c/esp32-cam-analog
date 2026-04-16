# Roadmap projektu

## Kamienie milowe (plan programistyczny)

### M0 - Setup projektu

Zakres:

- Utworzenie struktury PlatformIO (`src`, `include`, `lib`, `test`).
- Konfiguracja płytki ESP32-CAM i podstawowych flag kompilacji.
- Moduł konfiguracji (Wi-Fi, MQTT, parametry manometrów).

Definition of Done:

- Projekt buduje się lokalnie bez błędów.
- Firmware uruchamia się na ESP32-CAM i loguje start.

### M1 - Kamera i akwizycja obrazu

Zakres:

- Stabilny odczyt klatki z OV2640 (preferencyjnie grayscale/QVGA).
- Kontrola ekspozycji/jasności pod stałe warunki stanowiska.
- Endpoint diagnostyczny lub log do podglądu surowej klatki testowej.

Definition of Done:

- Minimum 100 kolejnych klatek bez restartu i bez błędów pamięci.
- Czas pobrania klatki zmierzony i zapisany w logu.

### M2 - Kalibracja geometrii manometrów

Zakres:

- Definicja dwóch ROI (manometr 1 i manometr 2).
- Ustalenie środka tarczy oraz zakresu kątów (`kat_min`, `kat_max`) dla obu manometrów.
- Mapowanie kąt -> wartość (`wartosc_min`, `wartosc_max`) osobno dla każdego.

Definition of Done:

- Parametry kalibracji są zapisane i wczytywane po restarcie.
- Dla klatek testowych algorytm zwraca sensowny kąt dla obu ROI.

### M3 - Odczyt jednego manometru (algorytm bazowy)

Zakres:

- Lekki pipeline CV: filtracja, progowanie, detekcja kierunku wskazówki.
- Wyznaczenie kąta wskazówki i przeliczenie na jednostkę ciśnienia.
- Wskaźnik jakości odczytu (`confidence`).

Definition of Done:

- Dla jednego manometru błąd średni testowy <= 3% zakresu.
- Odczyt stabilny (bez skoków niezgodnych ze stanem rzeczywistym).

### M4 - Dwa manometry i różnica

Zakres:

- Uruchomienie algorytmu dla dwóch ROI w tej samej klatce.
- Obliczanie `difference = gauge_1 - gauge_2`.
- Obsługa statusów błędów dla braków detekcji pojedynczej i podwójnej.

Definition of Done:

- Obie wartości i różnica są wyliczane w jednym cyklu pomiarowym.
- Przy braku detekcji pojawia się poprawny status zamiast błędnej liczby.

### M5 - Integracja MQTT i telemetria

Zakres:

- Publikacja JSON z odczytami, różnicą, statusem i timestamp.
- Konfigurowalny interwał publikacji.
- Reconnect Wi-Fi/MQTT i odporność na chwilowy brak sieci.

Definition of Done:

- Broker odbiera stabilny strumień wiadomości przez minimum 24 h testu.
- Po zerwaniu sieci urządzenie samo wraca do publikacji.

### M6 - Testy środowiskowe i strojenie

Zakres:

- Testy w różnych warunkach oświetlenia.
- Korekta progów, filtrów, ewentualne wygładzanie czasowe.
- Walidacja końcowa dokładności i stabilności.

Definition of Done:

- Ustalone finalne parametry dla docelowego stanowiska.
- Spisane ograniczenia i rekomendacje montażowe.

## Co bierzemy z AI-on-the-edge-device (fork xdw15c)

Nie robimy projektu na bazie forka — jest za duży i opiera się o ESP-IDF + TFLite.
Bierzemy z niego konkretne wzorce:

- **Format konfiguracji ROI na SD** (`config/config.json`) zamiast twardego kodowania w firmware.
- **Analog ROI editor** — koncepcja środka, kąta min/max, promienia jako parametry kalibracji wskazówki.
- **Web UI serwowany z karty SD** — pliki HTML/JS na SD zamiast PROGMEM, dzięki czemu UI można aktualizować bez reflashowania firmware.

## Backlog techniczny po MVP

- Autokalibracja ROI (opcjonalnie półautomatyczna).
- Lepsza detekcja refleksów i odblasków.
- Zdalna rekonfiguracja parametrów przez MQTT.
- Buforowanie odczytów przy braku sieci.
- OTA update firmware przez Web UI.

## Jak śledzić kamienie milowe w VS Code

Najprościej w 3 wariantach:

1. Checklisty w tym pliku (szybki start).
2. GitHub Issues + Milestones + Projects (kanban, najlepsze do dłuższej pracy).
3. TODO w plikach kodu i panel Problems/Tasks (do bieżącej implementacji).

Rekomendacja:

- Każdy milestone M0-M6 jako osobny milestone na GitHub.
- Zadania z sekcji "Zakres" jako Issues.
- "Definition of Done" przenieść 1:1 do kryteriów akceptacji issue.
