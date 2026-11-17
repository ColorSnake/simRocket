# Komunikacja komponentów i logowanie publikacji

Stan: 2026-10-28. Transport scenariuszy YAML znajduje się w dedykowanych katalogach:

- `rocket_sil_framework/include/communication/component_transport.hpp` — typowane porty i kolejki (szablony C++).
- `include/communication/publication_log.hpp` oraz `src/communication/publication_log.cpp` — zapis JSONL.
- `include/communication/preset_log_payloads.hpp` oraz `src/communication/preset_log_payloads.cpp` — serializacja payloadów obecnego presetu.

Ścieżki z pominiętym prefiksem są względem `rocket_sil_framework/`.
Scheduler fazowy pozostaje w `include/core/phased_scheduler.hpp`: odpowiada
za uruchamianie modeli, a nie przenoszenie danych. Składanie połączeń znajduje się w
`src/runtime/configured_system.cpp` (względem `rocket_sil_framework/`), poza CLI.

## Jak działa transport

`Output<T>` reprezentuje jeden nazwany port producenta, np. `imu.measurement`.
Metoda `connect()` tworzy `Input<T>` konkretnego odbiorcy. Typ T sprawdza kompilator;
nie ma centralnego brokera wyszukującego komponenty po napisach w każdym kroku.
Nazwy służą identyfikacji i diagnostyce.

`publish()` tworzy jeden niezmienny `Sample<T>`: payload wraz z kopertą zawierającą
źródło, sekwencję, czas pomiaru, czas dostępności, ważność i jakość. Odbiorcy mają
osobne kolejki wskaźników do tej samej próbki. Odczyt jednej kolejki nie usuwa danych
innemu odbiorcy. Czas jest liczony w całkowitych nanosekundach symulacji.

- `latest()` wybiera ostatnią dostępną publikację i może użyć jej ponownie.
- `take()` dla `all_samples` oddaje wszystkie nowe dostępne próbki, każdą raz.
- Zdarzenia czekają dodatkowo na `effective_time`; czas skutku musi pasować do siatki odbiorcy.

Każdy odczyt sprawdza dostępność, jakość i wiek. Wymagane wejście bez poprawnych
danych zatrzymuje przebieg z diagnostyką. Przepełnienie kolejki również jest błędem,
bez cichego wyrzucania danych. Publikacja nie wywołuje obliczeń odbiorcy — robi to
scheduler w odpowiedniej fazie.

Przykład przepływu: IMU → kolejka pomiarów → estymator → ostatnia estymata →
kontroler → zadanie TVC → alokator → bezpośrednia nastawa aktuatora → geometria ciągu.
IMU 1000 Hz i estymator 100 Hz mogą wymieniać wszystkie próbki, jeżeli `max_age`
uwzględnia okres aktywacji wolniejszego odbiorcy.

`BusTopic<T>` i `DirectPort<T>` są obecnie aliasami `Output<T>`. Mają wspólne
reguły transportu, ale inne znaczenie połączeń: informacja pokładowa versus
sterowanie/fizyka. Nie symulują CAN, Ethernetu, przepustowości ani strat.
Kod jest jednowątkowy; HIL wymagałby osobnego przekazania danych do wątku symulacji.

## Zrzut danych

```sh
./build/simRocket examples/component_config/scenarios/fmu_imu.yaml --log-bus /tmp/fmu-bus.jsonl
./run_sim.py examples/component_config/scenarios/fmu_imu.yaml --log-bus /tmp/fmu-run-bus.jsonl
```

Można dołączyć `--log-rosbag` do drugiej komendy. Ścieżka względna zrzutu jest
liczona względem katalogu uruchomienia, nie względem scenariusza. Istniejący plik
nie jest nadpisywany. `--validate` i `--dump-config` nie tworzą zrzutu.
Opcja wymaga scenariusza komponentowego (YAML lub jego rozwiniętego JSON);
legacy JSON bez `component_runtime` jest odrzucany.

Log zawiera każdą publikację siedmiu podłączonych portów: `imu.measurement`,
`gps.measurement`, `estimator.state`, `controller.demand`, `allocator.setpoint`,
`actuator.state`, `dynamics.state`. Wpis ma `transport: bus` lub `direct`.
Nie jest próbkowany częstotliwością `record.rate_hz` i nie zależy od `record.signals`.
Jeden wiersz JSONL to jedna publikacja, niezależnie od liczby odbiorców.

Przykładowa struktura wpisu:

```json
{"schema":"simrocket/publication-v1","transport":"bus","source":"imu.measurement","sequence":2,"sample_time_ns":2000000,"available_time_ns":3000000,"valid_until_ns":null,"quality":"valid","effective_time_ns":null,"payload":{"angular_velocity_body_rad_s":[0,0,0],"specific_force_body_m_s2":[0,0,0]}}
```

W tym przykładzie pomiar dotyczy 2 ms, ale jest dostępny od 3 ms. `null` w
`valid_until_ns` oznacza brak granicy ważności nadanej przez producenta; odbiorca
nadal stosuje własne `max_age`. Sekwencja jest niezależna dla każdego źródła.
Wiersze są w kolejności wywołań publikacji, niekoniecznie czasu akwizycji.
Zrzut rejestruje publikację, nie potwierdzenie odczytu przez odbiorcę ani jego ocenę
stale/invalid. W szczególności nie jest sam w sobie mechanizmem odtwarzania symulacji.

Logger jest pasywnym obserwatorem `publish()`, bez dodatkowej kolejki odbiorcy.
Po wyłączeniu opcji nie wykonuje serializacji JSON. Po włączeniu zapis jest
synchroniczny i buforowany; błąd zapisu zatrzymuje przebieg. Może zwiększyć czas
wykonania i rozmiar danych — nie jest optymalizacją wydajności.

Dla nowego typu payloadu trzeba dodać encoder i podłączyć logger przez `attach()`.
Logger nie introspektuje dowolnych struktur C++. Obecny format jest diagnostyką
adapterów, nie pełną serializacją kontraktów v1: np. `actuator.state` zawiera
transformację montażową używaną przez geometrię napędu, a GPS zachowuje jawnie
nazwane pole prędkości NED z obecnej struktury pomiarowej.

## Dotychczasowy MessageBus

`include/bus/message_bus.hpp` pozostaje dla legacy JSON i lokalnych adapterów klas.
Ma synchroniczne callbacki, routing przez typ C++ i subskrypcje wygaszane wraz
z właścicielem. Nie jest wielowątkowy, a wiadomości nie muszą dziedziczyć wspólnej
klasy bazowej. Nowy zrzut nie przechwytuje automatycznie jego wewnętrznych wiadomości.

Fazy i zakres migracji: [component_runtime.md](../component_runtime.md).
