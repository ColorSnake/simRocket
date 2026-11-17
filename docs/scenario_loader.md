# Loader scenariuszy YAML — etap 3

Status: loader wdrożony 2026-10-20; runtime fazowy dodany 2026-10-28.
Aktualna komunikacja i harmonogram: [runtime komponentów](component_runtime.md). YAML jest wejściem do istniejącego runtime C++,
przez adapter presetu `rocket-tvc/v1`. Dotychczasowe konfiguracje JSON nadal działają.

## Uruchomienie

Python wybrany przez CMake wymaga PyYAML (`python3 -m pip install -r tools/requirements.txt`).
Po skonfigurowaniu i zbudowaniu projektu z katalogu repozytorium:

```sh
cmake --build build -j2
./build/simRocket examples/component_config/scenarios/native_imu.yaml --validate
./build/simRocket examples/component_config/scenarios/fmu_imu.yaml --dump-config
./build/simRocket examples/component_config/scenarios/native_imu.yaml
./build/simRocket examples/component_config/scenarios/fmu_imu.yaml
```

`--validate` i `--dump-config` nie uruchamiają symulacji ani nie tworzą logów.
Walidacja FMU czyta metadane archiwum, nie ładuje biblioteki. Zgodność funkcji
binarnego FMU jest sprawdzana dopiero podczas uruchomienia. Przykładowe FMU powstaje
w `build/fmu/imu_sensor.fmu`; trzeba je wcześniej zbudować.

## Co zmienia się dla użytkownika

Scenariusz wybiera implementację IMU w jednym miejscu:

```yaml
extends: common.yaml
components:
  imu:
    model: ../models/fmu_imu.yaml
```

Zmiana na `../models/native_imu.yaml` zachowuje pojazd, sterowanie i połączenia.
Mapowanie np. `true_gyro_x` na składową prędkości kątowej znajduje się w opisie FMU,
a nie w każdym scenariuszu. Ścieżki są liczone względem pliku, który je deklaruje.
Dziedziczenie scala słowniki; listy są zastępowane w całości.

Loader odrzuca m.in. literówki w polach, nieznane parametry, cykle dziedziczenia,
niezgodne porty i kierunki FMU, nieprawidłowe jednostki/ramy oraz częstotliwości,
których nie można wykonać na siatce czasu. Nieobsługiwane ustawienia są błędem.
`--dump-config` pokazuje także wynikowe parametry, harmonogram i opis połączeń.

Początkowy stan rzeczywisty i `estimator_prior` są niezależne. Można więc rozpocząć
lot z poprawną pozycją rakiety, ale błędnym początkowym przekonaniem estymatora.
Aktuator udostępnia osobno zadaną i rzeczywistą pozycję; logger zapisuje rzeczywistą.

Dla FMU przetwarzającego wejście zatrzaśnięte w t wynik dostępny w t+1 ms nadal
opisuje pomiar z t. Adapter zachowuje oba czasy, a powtórną próbkę t=0 po pierwszym
kroku odrzuca. Zbyt stary wymagany pomiar IMU lub sterowanie zatrzymuje przebieg;
przeterminowany opcjonalny GPS jest pomijany. Zachowanie czasu dla starych JSON
pozostaje bez zmian, chyba że jawnie wybiorą nową politykę FMU.

## Wyniki

Każdy przebieg tworzy unikalny podkatalog w `record.directory`, np.
`logs/component_examples/fmu_imu-XXXXXX/`, zawierający:

- `resolved.json` — pełną konfigurację używaną do uruchomienia;
- `samples.jsonl` — po jednym obiekcie JSON na zapis, z grupami wybranymi przez `record`.

Obsługiwane grupy: `truth`, `imu.measurement`, `estimator.state`, `actuator.state`.
`time_s` oznacza chwilę snapshotu przed krokiem fizyki (od etapu 4). Poszczególne grupy mogą opisywać
starsze dane: IMU ma `sample_time_s` i `available_time_s`, estymata własny
`sample_time_s`. Nie należy utożsamiać czasu wiersza z czasem pomiaru.
To zapis diagnostyczny, jeszcze nie serializacja pełnych kopert kontraktów v1.
Dotychczasowy CSV jest opcjonalny (`--log-csv`).

## Granice tego etapu

Obsługiwany jest jeden preset: jedna rakieta, jeden silnik ciekły, jeden aktuator,
dynamiczny model masy. Tylko implementację IMU można podmienić opisem FMU.
Pozostałe role wybierają określone istniejące modele natywne.

Częstotliwości muszą dzielić zegar bazowy. Dynamika, aktuator i napęd działają
z częstotliwością bazową; estymator ma niezależną częstotliwość, allocator działa z kontrolerem.
Dostępne są tylko polityki wejść zadeklarowane w przykładach. Import obejmuje
FMI 2.0 Co-Simulation, skalary Real i binaria Linux x86_64; brak konwersji jednostek.

Komunikacja w YAML korzysta z kolejek typowanych portów i schedulera fazowego.
Istniejące klasy C++ są podłączone przez izolowane adaptery. Szczegóły i pozostałe
ograniczenia opisuje [runtime komponentów](component_runtime.md). Etap 5a przeniósł
składanie z `main.cpp` do warstwy runtime. Rozdzielenie pozostałej wewnętrznej
kompozycji dynamiki pozostaje dalszą częścią etapu 5.

## Weryfikacja

`ctest --test-dir build --output-on-failure` uruchamia cztery zestawy testów.
Zestaw loadera zawiera 16 przypadków: walidację, błędne konfiguracje,
uruchomienia native/FMU, zgodność YAML z jego skompilowanym JSON, prior estymatora,
wybór danych zapisu i odrzucanie starych pomiarów. Test C++ sprawdza dodatkowo
czas akwizycji FMU i usuwanie powtórzonej próbki początkowej.
Oba dołączone scenariusze przeszły także pełny przebieg 10 s.

## Nagrywanie ROS bag

```sh
./run_sim.py examples/component_config/scenarios/fmu_imu.yaml --log-rosbag
```

Orchestrator rozwija YAML jeden raz przed startem kontenerów. Symulator i mostek ROS
korzystają z tego samego snapshotu JSON; kontener nie potrzebuje parsera YAML.
Mostek otrzymuje ścieżkę przez `SIMROCKET_CONFIG`, zamiast czytać domyślny config.json.
Dotychczasowe scenariusze JSON również przekazują swoją konfigurację mostkowi.
Bez argumentu nadal wybierany jest `config.json`.

Nagranie powstaje w `bags/simrocket_<data>/` jako MCAP, z `metadata.yaml` oraz
kopią `resolved.json`. Brak gotowości mostka lub nagrywarki blokuje start symulacji.
Błędna konfiguracja mostka jest błędem, zamiast cichego powrotu do domyślnych danych.
Kształt rakiety pochodzi z siatki `rocket_z_up.obj`, skalowanej przez
`visuals.rocket_length_m` (całkowita długość z nosem) i `visuals.rocket_radius_m`
(promień cylindrycznego kadłuba, bez stateczników). Stateczniki skalują się wraz
z kadłubem; nie są osobno parametryzowane. Marker zachowuje orientację jednostkową: przeglądarka importuje siatkę OBJ
do układu Z-up, więc dodatkowy obrót markera przechylał nos w stronę -Y.
Długość skalowana jest po osi Z zaimportowanej siatki. Skalowanie zachowuje początek siatki, nie centruje jej
ponownie; nie zmienia masy, aerodynamiki ani innych parametrów fizyki.
Położenie środka parcia, mocowań i lokalizacja startowa
są pobierane z konfiguracji wybranego scenariusza.

Integrację sprawdzono pełnym przebiegiem FMU 10 s z Dockerem i zapisem MCAP.
Testy przekazania konfiguracji: `python3 tools/test_ros_config.py`.

Pełny zrzut publikacji komunikacji: dodaj `--log-bus /tmp/bus.jsonl` do komendy
symulatora lub `run_sim.py`. Plik musi być nowy. Zapis obejmuje także direct i jest
niezależny od okresowego `samples.jsonl`; szczegóły w [opisie komunikacji](modules/communication.md).
