# Przykłady konfiguracji komponentów — punkty 2 i 3

**Status: uruchamialne scenariusze, 2026-10-20.** Loader waliduje YAML i składa
konfigurację dla obecnego runtime przez adaptery presetu `rocket-tvc/v1`.
Instrukcje CLI i ograniczenia: [loader scenariuszy](../../docs/scenario_loader.md).
Od 2026-10-28 preset używa [szyny i faz](../../docs/component_runtime.md); ogólne składanie grafu pozostaje punktem 5.

## Zacznij od tych dwóch plików

- [scenarios/native_imu.yaml](scenarios/native_imu.yaml)
- [scenarios/fmu_imu.yaml](scenarios/fmu_imu.yaml)

Ich jedyną różnicą jest wartość `components.imu.model`:

```yaml
extends: common.yaml
components:
  imu:
    model: ../models/native_imu.yaml  # albo ../models/fmu_imu.yaml
```

Użytkownik wybiera implementację roli. Nie dodaje równolegle drugiego sensora,
nie przełącza enabled, nie wpisuje nazw skalarnych zmiennych FMU i nie zmienia
kolejności execution_order. Połączenia pozostają takie same.

## Gdzie znajduje się reszta informacji

| Plik | Zawartość | Kiedy go zmieniać |
|---|---|---|
| [scenarios/common.yaml](scenarios/common.yaml) | Czas, otoczenie, stan początkowy, prior estymatora, zapis | Przy zmianie eksperymentu |
| [vehicles/demo_rocket.yaml](vehicles/demo_rocket.yaml) | Masa, zbiorniki, napęd, geometria i aerodynamika | Przy zmianie rakiety |
| [systems/rocket_tvc.yaml](systems/rocket_tvc.yaml) | Role, częstotliwości, sterowanie i idealna komunikacja | Przy zmianie architektury systemu |
| [models/native_imu.yaml](models/native_imu.yaml) | Interfejs i parametry implementacji natywnej | Przy dodawaniu implementacji |
| [models/fmu_imu.yaml](models/fmu_imu.yaml) | Plik FMU, porty, mapowanie i znaczenie czasu | Raz przy imporcie FMU |

W zwykłej pracy edytujesz scenariusz. Opisy systemu i modeli mogą być bardziej
szczegółowe, ale są wielokrotnie używane. Nie przenosimy ich zawartości do każdego
nowego eksperymentu.

Dane konstrukcji pochodzą z istniejącego test_fmu.json. Na potrzeby czytelnego
porównania obu implementacji IMU/GPS mają wyłączony szum, a profil przepustnicy
zawiera tylko punkty obejmujące 10-sekundowy lot. Oba scenariusze uruchomiono przez 10 sekund czasu symulacji. Testy sprawdzają też
zgodność uruchomienia YAML z jego skompilowaną konfiguracją JSON.

## Reguły składania konfiguracji

1. `extends` w przykładach wskazuje jeden wspólny scenariusz. W v1 nie potrzebujemy
   wielokrotnego dziedziczenia; cykle i nieznane pola są odrzucane.
2. Ładowany jest wspólny scenariusz, wskazany pojazd i system.
3. Wpisy `components` ze scenariusza nadpisują pola komponentu o tym samym
   identyfikatorze w systemie. Zmiana modelu zastępuje implementację, nie tworzy
   nowego komponentu. Rola, częstotliwość i połączenia pozostają przypisane do `imu`.
4. `parameters` są łączone po nazwach: domyślne z nowego opisu modelu → ustawienia
   systemu → ustawienia scenariusza. Parametr nieznany w nowym modelu jest błędem,
   a nie pozostałością ignorowaną po zamianie implementacji. Listy zastępujemy w całości.
5. Zmieniony model musi spełniać te same kontrakty portów. Źródłem prawdy o portach
   jest opis modelu i katalog kontraktów, nie nazwa komponentu.
6. Ścieżki są zawsze względem pliku, w którym zostały zapisane, także po dziedziczeniu.
   Identyfikatory `native/...` odwołują się do obsługiwanego zestawu wbudowanych modeli,
   a ścieżki zakończone .yaml do opisów plikowych. `id` modelu jest metadanymi,
   nie drugą metodą wyszukiwania w tym przykładzie.
7. Pole liczbowe ma jednostkę w nazwie (`rate_hz`, `duration_s`, `angle_x_rad`).
   Nie wymagamy jeszcze parsera zapisów typu „10 ms”. W bindings jednostki opisują
   porty FMU jawnie. Wszystkie wektory mają kolejność x,y,z; kwaterniony w,x,y,z.
8. `record.directory` określa katalog. Docelowy identyfikator przebiegu pochodzi
   z nazwy wejściowego scenariusza i unikalnego numeru uruchomienia, aby oba warianty
   nie nadpisywały sobie wyników. Każdy przebieg zapisuje `resolved.json` i `samples.jsonl`.

Loader akceptuje określony zestaw identyfikatorów natywnych. Opcja `--dump-config`
pokazuje rozwiązaną konfigurację z wartościami domyślnymi i połączeniami bez symulacji.

## Co dokładnie oznacza topology: rocket-tvc/v1

To obsługiwany preset dla jednej rakiety, jednego silnika ciekłego i jednego
aktuatora TVC w tym przykładzie. Adapter składa obecne klasy C++; nie jest to jeszcze dowolny graf pluginów. Zawiera połączenia:

| Producent | Odbiorca | Kontrakt | Połączenie |
|---|---|---|---|
| dynamics.imu_truth | imu.truth | ImuTruth/v1 | direct |
| imu.measurement | estimator.imu | ImuMeasurement/v1 | avionics |
| gps.measurement | estimator.gps | GpsMeasurement/v1 | avionics |
| estimator.state | controller.state | StateEstimate/v1 | avionics |
| controller.demand | allocator.demand | TvcDemand/v1 | avionics |
| allocator.setpoint | actuator.setpoint | ActuatorSetpoint/v1 | direct |
| actuator.state | propulsion.actuator_state | ActuatorState/v1 | direct |
| propulsion.wrench | dynamics.propulsion_wrench | Wrench/v1 | direct |

GPS pobiera pozycję i prędkość CG z wyjścia stanu fizycznego dynamiki przez direct.
Preset zawiera w tym celu jawny adapter `dynamics.state → gps.truth`:
`KinematicTruth/v1` obejmuje position_world (m), velocity_world (m/s),
orientation_body_to_world (wxyz) oraz angular_velocity_body (rad/s), z kopertą
czasową. Jest to stan prawdziwy, nie StateEstimate. Uzupełniono nim katalog kontraktów.

`dynamics` zawiera aerodynamikę, środowisko i model masy z opisu pojazdu;
`propulsion` zawiera model silnika oraz geometrię kierunku ciągu. Bilans paliwa
wymaga dodatkowego direct `propulsion.mass_flow → dynamics.propellant_flow`:
`PropellantFlow/v1` ma pole mass_flow_kg_s (Real >= 0, kg/s), a przypisanie do
układu zasilania wynika z engine_id pojazdu. Dostępność paliwa wraca przez direct
`dynamics.propellant → propulsion.propellant` jako `PropellantState/v1`:
remaining_mass_kg (Real >= 0, kg). Dla tego presetu zbiorniki tworzą jeden wspólny
układ zasilania. Inicjalny stan jest publikowany przed pierwszym obliczeniem ciągu;
zużycie paliwa zmienia stan dopiero po kroku, więc nie powstaje pętla feedthrough.
Otoczenie przekazuje też direct `dynamics.ambient_pressure → propulsion.ambient_pressure`
przez `AmbientPressure/v1`: pressure_pa (Real >= 0, Pa). Docelowo napęd korzysta wyłącznie z tych portów. Obecny adapter nadal korzysta
z istniejącej kompozycji C++ wewnątrz dynamiki; lista portów opisuje docelowe połączenia.

Geometria momentu ciągu otrzymuje direct `dynamics.mass_properties → propulsion.mass_properties`
przez `MassProperties/v1` (masa, CG względem datum body, tensor bezwładności w CG).
Dzięki temu moment wyjściowego Wrench jest liczony względem bieżącego CG, również
podczas zużycia paliwa.

Są to minimalne uzupełnienia potrzebne do opisu istniejącego silnika ciekłego.
Nie projektujemy teraz rozdzielonych instalacji paliwa/utleniacza ani solvera hydrauliki.
Fizyczna transformacja montażu aktuatora i parametry silnika są wiązane przez
vehicle_actuator_id / vehicle_engine_id. Stan inicjalny dynamiki oraz prior estymatora
pochodzą z osobnych sekcji scenariusza.

Powrót dynamics → sensory → sterowanie → plant → dynamics jest sprzężeniem
przez stan. Docelowo kolejność i fazy wynikają z kontraktów. Obecnie scheduler presetu wyznacza
kolejność z zależności i faz, niezależnie od kolejności tabeli YAML.
Idealny aktuator ma feedthrough; geometria napędu może odczytać jego bieżące
wyjście w t. Bardziej realistyczny model stanowy musi udostępniać tylko stan
osiągnięty w t. Zmiana plant modelu nie może ujawniać wyników z t+h przed czasem.

## Co jest wspólne, a co może się różnić w wynikach

Oba warianty mają identyczny pojazd, pogodę, sterowanie, prior, czas lotu,
częstotliwości i parametry szumu. Interfejsy i nazwy komponentów pozostają te same.
Różni się implementacja IMU oraz wynikająca z niej semantyka czasowa:

- Natywne IMU: pomiar dla t może być dostępny w t.
- Przykładowe FMU: przetwarza zatrzaśnięty pomiar z t podczas kroku h=1 ms;
  wynik ma sample_time=t, available_time=t+h.
- Inicjalizacja FMU daje próbkę początkową dla t=0. Wynik pierwszego doStep
  odnosi się ponownie do t=0; adapter odrzuca tę powtórkę zgodnie z opisem modelu,
  by nie dostarczać EKF dwukrotnie tej samej chwili pomiarowej.

Nie oczekujemy identycznej trajektorii zamkniętej pętli co do bitu. Oczekujemy
zgodności znaczenia i jednostek sygnałów oraz jawnej, sprawdzalnej różnicy czasu.
Jeśli włączymy szum, implementacje mogą mieć różne generatory liczb losowych.

Dla IMU 800 Hz ustawiamy jego `rate_hz` oraz częstotliwość estymatora na 800,
a `base_rate_hz` na 4000. Dopasowujemy też częstotliwości
komponentów, które mają pozostać związane z krokiem bazowym. Obecne przykłady
celowo pozostają przy 1000 Hz, aby sprawdzać jedną różnicę naraz. Rozdzielenie
częstotliwości fizyki od bazowego zegara wymaga wsparcia przyszłego schedulera.

## Weryfikacja tego etapu

Oba przykłady przeszły walidację i pełny przebieg 10 s. Testy automatyczne obejmują
błędy konfiguracji, mapowania FMU, czasy pomiarów, wybór danych do zapisu i oddzielny
prior estymatora. Szyna i fazy czasu presetu mają osobne testy od etapu 4.

Kontrakty: [interface_contracts.md](../../docs/interface_contracts.md).
