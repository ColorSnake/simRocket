# FMU w simRocket — przewodnik i wyjaśnienie zmian

## Co jest obsługiwane

Importer obsługuje FMI 2.0 Co-Simulation, synchroniczne modele dla Linux x86_64
(`binaries/linux64`) oraz mapowanie skalarnych zmiennych `Real`. Można dodać model
z innymi nazwami zmiennych niż przykładowe IMU bez zmieniania C++.

Dostępne wyjścia: sygnały dla kolejnych FMU i telemetrii albo adapter do wiadomości
IMU, TVC i sterowania silnikiem ciekłym. Obsługa nie obejmuje FMI Model Exchange,
FMI 3.0, portów Integer/Boolean/String, asynchronicznych kroków, rollbacku ani
rozwiązywania pętli algebraicznych. Nie jest to jeszcze importer każdej możliwej FMU.

## Uruchomienie

W katalogu repozytorium:

```sh
cmake -S . -B build -DSIMROCKET_BUILD_TESTS=ON
cmake --build build -j 2
ctest --test-dir build --output-on-failure
./build/simRocket test_fmu.json
```

Jeżeli polecenie `ctest` wskazuje na niedziałający lokalny wrapper Pythona,
użyj systemowego `/usr/bin/ctest`. To problem środowiska, nie zestawu testów.

CMake tworzy `build/fmu/imu_sensor.fmu`; przy włączonych testach również
`build/fmu/integrator.fmu`. Stary plik `fmu/imu_sensor.fmu` nie jest używany przez
zaktualizowany przykład. Pakowanie nie nadpisuje plików źródłowych.
Potrzebne są CMake, kompilatory C/C++, Python 3 oraz zależności pobierane przez CMake.
Pythonowa biblioteka standardowa służy do rozpakowania ZIP — bez `unzip` i powłoki.

Ścieżki FMU, krzywych silników, profili przepustnicy i logu CSV są liczone względem
katalogu pliku JSON. Przykład zapisuje `logs/sim_fmu_log.csv`.

## 1. Czas: częstotliwość i krok to dwie strony tej samej informacji

100 Hz oznacza wykonanie co 0,01 s. Wcześniej moduł wykonywany 100 razy na sekundę
otrzymywał za każdym razem `dt=0,001`, więc sumował tylko 0,1 s. Model całkujący
stałe przyspieszenie 2 m/s² mógł przez to uzyskać prędkość 0,2 m/s zamiast 2 m/s.
Teraz callback dostaje własny okres: dla 100 Hz jest to 0,01 s.

Scheduler używa numerów taktów. Częstotliwość bazowa musi być wspólną wielokrotnością
częstotliwości modułów. Przykład:

```json
"scheduler": {
  "base_rate_hz": 4000,
  "execution_order": ["imu_fmu", "sensors", "control", "dynamics", "telemetry"]
}
```

Przy FMU 800 Hz daje to krok 0,00125 s i dokładnie 800 wywołań na sekundę.
Na bazie 1000 Hz program odrzuca takie ustawienie. W obecnej implementacji natywna
fizyka działa z częstotliwością bazową: ustawienie 4000 Hz zwiększa także jej koszt.
Wartość 0 w API schedulera oznacza każde wywołanie; konfiguracja FMU wymaga dodatniej
częstotliwości. Czas symulacji musi być wielokrotnością kroku bazowego i okresów FMU.

Na początku taktu t publikowane są gotowe wyniki FMU dla t. Następnie wykonywane są
sensory, sterowanie i kroki FMU w zadanej kolejności, potem dynamika, a na końcu
telemetria. Sensory i kontrolery muszą poprzedzać dynamikę. GPS ma niezależny okres
pobierania próbek; kolejka dostarczania jest sprawdzana w każdym takcie bazowym.
Opóźnienie GPS jest zaokrąglane w górę do chwili obsługi tej kolejki.

Wywołanie `doStep(t, h)` wylicza wynik na t+h. Wynik czeka w buforze do tej chwili.
Kontroler nie dostaje wyniku z przyszłości. Wejścia są utrzymywane stałe podczas
kroku (zero-order hold). Oznacza to jawne opóźnienie wymiany danych: FMU IMU nie
jest czasowo identyczne z natywnym czujnikiem próbkowanym bezpośrednio w t.
Wartości sprzężeń pomiędzy FMU podczas inicjalizacji zaczynają od zera; nie ma
iteracyjnego uzgadniania początkowego stanu wielu modeli.

## 2. Akcelerometr nie zwraca po prostu przyspieszenia trajektorii

Przyspieszenie w `RocketState` jest podane w świecie i uwzględnia grawitację.
Akcelerometr mierzy siłę właściwą w swoich osiach. Dla sensora w środku masy:

```
f_body = inverse(R_body_to_world) * (a_world - gravity_world)
```

Przykłady idealnego czujnika:

| Sytuacja | Przyspieszenie trajektorii Z | Odczyt akcelerometru przy pionowej rakiecie |
|---|---:|---:|
| Spoczynek na podłożu | 0 | +9,81 m/s² |
| Swobodny spadek | −9,81 m/s² | 0 |
| Przyspieszanie w górę 2 m/s² | +2 | +11,81 m/s² |

Przy przechyleniu rakiety te same wielkości rozkładają się na osie czujnika.
Wcześniej EKF ponownie dodawał grawitację do sygnału, który już ją zawierał.
Teraz natywne IMU i wejścia przykładowego FMU dostają poprawną wielkość.

Świat ma osie ENU: X wschód, Y północ, Z góra. Kwaternion obraca wektor z rakiety
do świata. Oś Z rakiety biegnie od ogona do nosa. IMU zwraca rad/s i m/s².

## 3. EKF i GPS

Wiadomości mają `timestamp_s`, czyli czas wykonania pomiaru. EKF wyznacza krok
z różnicy czasów IMU. Pierwsza próbka ustala czas początkowy, a zdublowane lub
starsze próbki nie są ponownie całkowane. Test przyspieszenia 2 m/s² przez sekundę
przy IMU 100 Hz sprawdza prędkość 2 m/s i przesunięcie 1 m.

GPS niesie też pozycję lokalną ENU. Wysokość początku układu nie jest już mylona
z wysokością rakiety nad tym początkiem. Dla początku 100 m n.p.m. i rakiety 40 m
wyżej GPS podaje 140 m n.p.m., a EKF koryguje lokalne Z do 40 m.

Opóźniony GPS jest przybliżany do aktualnego czasu za pomocą jego prędkości
Dopplera. To prosta kompensacja stałą prędkością, nie pełne odtwarzanie historii
EKF; podczas silnych przyspieszeń pozostaje błąd. Poprawiono też przypisanie
szumów biasu oraz użyto postaci Josepha przy aktualizacji kowariancji.

`random_seed` w głównym JSON ustala powtarzalne szumy natywnego IMU/GPS.
Powtarzalność dotyczy tego samego środowiska uruchomieniowego. Przykładowe FMU
ma własny deterministyczny generator; zewnętrzny model może mieć własny parametr seeda.

## 4. Dodanie własnego modelu

W `test_fmu.json` jest kompletny przykład IMU. Słowniki `inputs` i `outputs`
mają po lewej nazwę zmiennej wewnątrz FMU, a po prawej nazwę sygnału simRocket.

Poniższy fragment można dodać do konfiguracji z natywnym IMU, żeby uruchomić
przykładowy integrator. Dodaj też `velocity_integrator` przed `dynamics` w
`execution_order`:

```json
"external_modules": {
  "velocity_integrator": {
    "path": "build/fmu/integrator.fmu",
    "update_rate_hz": 100,
    "parameters": {"gain": 1.0},
    "inputs": {"drive": "truth.velocity.z"},
    "outputs": {"integral": "external.integrated_height"},
    "units": {"external.integrated_height": "m"},
    "publish": {"type": "signals"}
  }
}
```

Model sumuje prędkość pionową w czasie. Jego wynik jest dodatkowym sygnałem,
który można podłączyć do kolejnego FMU lub odczytać z telemetrii MessagePack
w `external`. CSV nadal ma stały zestaw kolumn; dowolne porty są w UDP.

Dostępne sygnały natywne, każdy z końcówką `.x`, `.y`, `.z`:

- `truth.position`, `truth.velocity` — świat ENU;
- `truth.gyro_body` — prawdziwa prędkość kątowa;
- `truth.specific_force_body` — idealny akcelerometr;
- `imu.gyro_body`, `imu.specific_force_body` — ostatni dostarczony pomiar IMU.

Każdy sygnał ma wartość, czas i opis jednostki. Jeżeli XML deklaruje jednostkę
bezpośrednio w `Real`, importer sprawdza zgodność jej nazwy. Nie przelicza jednostek
ani układów współrzędnych; dziedziczenie jednostek przez `declaredType` nie jest
jeszcze rozwinięte. Przy brakujących metadanych zgodność należy zapewnić w konfiguracji.

Adapter IMU:

```json
"publish": {
  "type": "imu",
  "gyro": ["external.imu.gyro.x", "external.imu.gyro.y", "external.imu.gyro.z"],
  "accel": ["external.imu.accel.x", "external.imu.accel.y", "external.imu.accel.z"]
}
```

Trzeba wyłączyć natywne IMU przez `sensors.imu.enabled=false`. Program odrzuca
konfigurację z dwoma producentami pomiarów IMU.

Inne adaptery:

- TVC: `{"type":"tvc", "pitch":"external.pitch", "yaw":"external.yaw"}`.
  Wymaga `control.tvc.enabled=false`. Kąty są w radianach; FMU odpowiada za limity komend.
- Silnik: `{"type":"engine", "engine_id":0, "throttle":"external.throttle", "active":"external.active"}`.
  Wymaga silnika ciekłego i usunięcia jego natywnego kontrolera profilu. `active`
  jest sygnałem Real interpretowanym jako włączony powyżej 0,5; przepustnica jest
  ograniczana przez model silnika do zakresu 0–1.

Wszystkie sygnały wymienione w adapterze muszą pochodzić z wyjść tego FMU.

## 5. Co zmieniło się wewnątrz

- `FmuModel` zajmuje się FMI: archiwum, XML, biblioteka, inicjalizacja i kroki.
  Nie zna rakiety ani nazw `in.gyro_x`.
- `FmuComponent` wiąże nazwy portów z `SignalRegistry` i buforuje wyniki do ich czasu.
- Parametry są ustawiane przed końcem inicjalizacji. Błędy FMI przerywają przebieg
  z nazwą operacji i kodem statusu; `Discard` i `Pending` nie są traktowane jak sukces.
- Importer i przykładowe modele używają oficjalnych nagłówków FMI 2.0.5.
- ZIP jest rozpakowywany do unikalnego, prywatnego katalogu. Odrzucane są m.in.
  wyjścia poza katalog, symlinki, duplikaty oraz nadmierny rozmiar archiwum.
  FMU nadal zawiera natywny kod wykonywany w procesie — obsługujemy zaufane modele,
  nie izolację dowolnych plików pobranych z internetu.
- `MessageBus` pozostaje synchroniczny i jednowątkowy. Subskrypcje modeli wygasają
  wraz z ich właścicielami; HIL wymaga osobnej kolejki między wątkami.
- Wspólna biblioteka `simrocket_core` jest linkowana do aplikacji i testów.
  CTest rejestruje testy, a testy scenariuszy korzystają ze schedulera.
- CSV rzeczywiście zapisuje próbki. Telemetria silników podaje ich indywidualny ciąg.
- Silnik ciekły gaśnie po wyczerpaniu paliwa. Ostatni krok jest dzielony na część
  z paliwem i bez paliwa. Masa pozostaje zamrożona w podkrokach RK4 danego odcinka;
  to nadal uproszczone sprzężenie, a nie pełne RK4 dla wszystkich stanów napędu.

## 6. Sprawdzenie i dalsze prace

Testy obejmują czas 100/800 Hz, IMU w spoczynku i spadku, propagację EKF,
opóźnienie GPS, zużycie ostatniej porcji paliwa, subskrypcje, stanowy model FMU,
mapowanie innych nazw, izolację dwóch instancji, błędy kroku, ZIP oraz uruchomienie
aplikacji z natywnym i zewnętrznym modelem. Przykładowy integrator jest osobnym
modelem testowym, nie walidacją zgodności importera ze wszystkimi narzędziami FMI.

Przed użyciem modelu wyeksportowanego z konkretnego narzędzia należy wykonać
jego test integracyjny. Dalszy backlog: pełna obsługa opóźnień EKF, weryfikacja
zbieżności całego modelu zmiennej masy, szersze typy FMI i jednostki, początkowe
sprzężenia modeli, osobny obiekt `Simulation`/fabryka konfiguracji oraz HIL.
