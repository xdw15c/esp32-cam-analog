# Roadmap projektu

Ten plik zawiera tylko otwarte prace. Zadania juz wykonane zostaly usuniete z planowania.

## Co jest juz wykonane (zamkniete)

- baza firmware ESP32-CAM + Wi-Fi + panel WWW
- Modbus TCP (FC03) i mapa 16 rejestrow
- kalibracja 2 manometrow i zapis config na SD
- analiza 2 manometrow + roznica + confidence
- trzy tryby analizy (classic/color/hybrid)
- logi na SD i zakladka Logi w WWW
- tryb offline SD photo mode
- upload JPEG z WWW do /latest.jpg
- debug decyzji analizy w JSON
- opis i dekodowanie mapy Modbus w zakladce WWW

## Otwarte zadania (aktywny plan)

### R1 - Strojenie jakosci odczytu

- testy na wiekszej probce realnych zdjec
- korekty progow i wag dla wskazowek z przeciwwaga
- walidacja stabilnosci przy granicach skali (okolice zera)

Definition of Done:

- stabilny odczyt dla obu manometrow na serii testowej
- mniej falszywych wyborow krotszego ramienia

### R2 - Powrot do trybu live camera

- naprawa/wymiana hardware kamery
- przelaczenie z offline SD na kamera live
- test ciagly i potwierdzenie stabilnosci

Definition of Done:

- stabilna praca live bez restartow i bez emergency fallback
- wyniki live porownywalne jakosciowo z trybem SD

### R3 - Testy systemowe i integracyjne

- dluzszy test pracy (minimum 24h)
- potwierdzenie poprawnych alarmow fault_code
- potwierdzenie integracji Modbus po stronie SCADA/PLC

Definition of Done:

- brak krytycznych bledow w tescie 24h
- poprawna interpretacja status/fault/source po stronie nadrzednej

### R4 - Usprawnienia operacyjne

- badge OFFLINE/ONLINE w zakladce Kamera
- reset reason + boot counter w logach
- drobne UX usprawnienia panelu WWW

Definition of Done:

- operator widzi stan trybu i latwo diagnozuje restart/fault

## Backlog (opcjonalne, po R1-R4)

- eksport historii pomiarow (CSV/JSON)
- OTA przez WWW
- opcjonalny MQTT jako dodatkowy kanal, nie zamiast Modbus
- polautomatyczna kalibracja ROI
