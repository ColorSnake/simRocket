# Kontrakty interfejsów v1

Aktualizacja 2026-10-28: scenariusze YAML używają [runtime fazowego](component_runtime.md)
z kolejkami idealnej szyny i portami direct. Specyfikacja obejmuje też docelowe
interfejsy nieudostępnione jeszcze przez preset; zakres i ograniczenia podano w opisie runtime.

Status: specyfikacja pierwszego etapu, 2026-10-17. Jest to podstawa implementacji,
a nie opis już wdrożonego API. Obecne C++ i JSON zachowują dotychczasowe działanie.
Dokument rozwija [architekturę komponentów](component_architecture.md).

## 1. Zakres i podstawowe decyzje

Wspólny kontrakt dotyczy implementacji natywnych i FMU. Interfejs określa
znaczenie danych, nie sposób wywoływania kodu ani protokół komunikacyjny.

- Logic models obejmują estymację, logikę misji i controller models.
- Plant models opisują fizyczne elementy oraz modele pomiaru.
- Szyna idealna przenosi informacje, bez strat i dodatkowego czasu transmisji.
- Bezpośrednie porty przekazują zadania do plant models i ich wielkości fizyczne.
- Wersja v1 używa dyskretnych punktów wymiany danych; nie jest solverem połączeń
  energetycznych, hydraulicznych ani pętli algebraicznych.
- Standardowe role mają domyślne połączenia. Graf niestandardowy korzysta z tych
  samych interfejsów i reguł walidacji.

Identyfikator kontraktu ma postać `ImuMeasurement/v1`. Zmiana znaczenia pola,
jednostki, ramy lub wymaganego pola wymaga nowej wersji. Implementacja komponentu
może zmienić się bez zmiany interfejsu.

## 2. Typy, jednostki i układy odniesienia

| Typ | Znaczenie |
|---|---|
| `Real` | Skończona liczba zmiennoprzecinkowa podwójnej precyzji |
| `Vector3` | Trzy Real w kolejności x, y, z, jedna jednostka i rama |
| `Quaternion` | Kolejność w, x, y, z; norma 1 z tolerancją numeryczną |
| `Bool` | Jawna wartość logiczna, nie umowny próg Real |
| `Enum` | Wartość z określonego zbioru, nie dowolny napis |
| `Time` | Czas symulacji od początku przebiegu; docelowo całkowite nanosekundy |

Czas wykonywania na komputerze nie wpływa na znaczenie Time. Obecny scheduler
pozostaje na stałej siatce; częstotliwości muszą być z nią zgodne. Konwersja sekund
FMI na Time jest sprawdzana na granicy adaptera, a nie przez porównywanie double
w każdym odbiorcy. Częstotliwości nieprzedstawialne na przyjętej siatce są odrzucane.

Wszystkie interfejsy systemowe używają SI: rad, rad/s, m, m/s, m/s², N, N·m.
Stopnie, g i inne jednostki mogą występować po stronie FMU, ale wymagają jawnej
konwersji w opisie adaptera. Brak jednostki w XML nie oznacza automatycznej zgodności.

- `world`: lokalny prawoskrętny ENU, X wschód, Y północ, Z góra. Początek i wysokość
  odniesienia definiuje scenariusz.
- `body`: prawoskrętna rama pojazdu, +Z od ogona do nosa. Położenie ramy względem
  konstrukcji określa model pojazdu. Pozycja/ruch w interfejsach lotu dotyczą CG.
- Wektory IMU są przeliczone do body; v1 odnosi je do sensora w CG. Sensor poza CG
  wymaga jawnego modelu ramienia i odpowiedniego przeliczenia przyspieszeń.
- `orientation_body_to_world` obraca wektor body do world, nie odwrotnie.
- Transformacja montażu dyszy/aktuatora jest parametrem pojazdu.

Dla TVC unikamy niejednoznacznych etykiet pitch/yaw. Używamy `angle_x`, `angle_y`,
dodatnich zgodnie z regułą prawej dłoni, i składania `R = Rx(angle_x) * Ry(angle_y)`
w lokalnej ramie montażowej. Najpierw działa Ry, potem Rx. To odpowiada aktualnemu
`yaw_rot * pitch_rot`; stary yaw jest angle_x, stary pitch jest angle_y.

## 3. Koperta danych: co wiemy i od kiedy

Payload (np. dwa wektory IMU) jest zawsze publikowany atomowo z kopertą:

| Pole | Znaczenie |
|---|---|
| `source` | Unikalny identyfikator komponentu i jego portu wyjściowego |
| `sequence` | Rosnący licznik publikacji na porcie, od 0 w każdym przebiegu |
| `sample_time` | Chwila, której dotyczy payload: pomiar, estymata lub zadana wartość |
| `available_time` | Najwcześniejsza chwila udostępnienia kompletnego payloadu |
| `valid_until` | Wyłączna granica ważności albo jawne `unbounded` |
| `quality` | `valid`, `invalid` lub `uninitialized` |

W v1: `sample_time <= available_time`. Przyszłe zaplanowane działania reprezentuje
się jako zdarzenia z `effective_time`, nie jako próbki dostępne przed pomiarem.
Koperta i payload są niezmienne po publikacji. Każdy odbiorca ma osobny kursor.

Odczyt w chwili t może wybrać wyłącznie próbkę z `available_time <= t`. Dodatkowo
musi być `quality=valid`, `t < valid_until` oraz `t - sample_time <= max_age`
ustalone na wejściu. Przeterminowanie jest wynikiem odczytu, nie mutacją quality.
Przy t równym valid_until próbka jest już nieważna. Nie przepisujemy sample_time
na czas odbioru. Brak próbki nie oznacza wektora zerowego.

Dla kontroli odczytu wyróżniamy `missing`, `invalid`, `stale`, `valid`. Nowsza
próbka invalid nie jest automatycznie maskowana ostatnią dobrą próbką.

## 4. Katalog interfejsów

W tabeli wymieniono wymagane payloady. Koperta z sekcji 3 jest wspólna.

| Kontrakt | Pola payloadu | Typowe połączenie |
|---|---|---|
| `KinematicTruth/v1` | `position_world`: Vector3 m; `velocity_world`: Vector3 m/s; `orientation_body_to_world`: Quaternion; `angular_velocity_body`: Vector3 rad/s | Dynamika → sensory, direct |
| `PropellantFlow/v1` | `mass_flow_kg_s`: Real >= 0, kg/s | Napęd → model masy, direct |
| `PropellantState/v1` | `remaining_mass_kg`: Real >= 0, kg | Model masy → napęd, direct |
| `AmbientPressure/v1` | `pressure_pa`: Real >= 0, Pa | Środowisko → napęd, direct |
| `MassProperties/v1` | `total_mass_kg`: Real > 0; `cg_body_m`: Vector3 m; `inertia_at_cg_body_kg_m2`: symetryczna dodatnio określona macierz 3×3 Real | Model masy → geometria napędu/dynamika, direct |
| `ImuTruth/v1` | `angular_velocity_body`: Vector3 rad/s; `specific_force_body`: Vector3 m/s² | Dynamika → model IMU, direct |
| `ImuMeasurement/v1` | `angular_velocity_body`: Vector3 rad/s; `specific_force_body`: Vector3 m/s² | IMU → estymator, bus |
| `GpsMeasurement/v1` | `position_world`: Vector3 m; `velocity_world`: Vector3 m/s | GPS → estymator, bus |
| `StateEstimate/v1` | `position_world`: Vector3 m; `velocity_world`: Vector3 m/s; `orientation_body_to_world`: Quaternion; `angular_velocity_body`: Vector3 rad/s | Estymator → kontroler, bus |
| `TvcDemand/v1` | `angle_x`: Real rad; `angle_y`: Real rad | Kontroler → mikser/alokator, bus |
| `ActuatorSetpoint/v1` | `angle_x`: Real rad; `angle_y`: Real rad | Mikser albo kontroler → konkretny plant aktuatora, direct |
| `ActuatorState/v1` | `angle_x`: Real rad; `angle_y`: Real rad; `rate_x`: Real rad/s; `rate_y`: Real rad/s | Plant aktuatora → geometria ciągu, direct |
| `ActuatorFeedback/v1` | `angle_x`: Real rad; `angle_y`: Real rad; `status`: Enum `ready`, `limited`, `fault` | Model pomiaru/elektroniki aktuatora → regulator, bus |
| `Wrench/v1` | `force_body`: Vector3 N; `moment_at_cg_body`: Vector3 N·m | Napęd/aerodynamika → sumator sił → dynamika, direct |

`ImuTruth` i `ImuMeasurement` mają te same typy pól, ale inne znaczenie. Dzięki
osobnym kontraktom nie podłączamy niechcący idealnego stanu do wejścia pomiarowego.
Orientacja nie jest częścią pomiaru surowego IMU. AHRS może publikować osobny
kontrakt w przyszłości. Biasy i kowariancje są diagnostyką estymatora, nie wymaganym
wejściem kontrolera w v1.

Akcelerometr przekazuje `specific_force_body = R⁻¹(a_world - gravity_world)`:
w swobodnym spadku zero, w spoczynku na podłożu wartość o normie około 9,81 m/s².
Nie używamy pola `linear_acceleration` bez określenia jego fizycznego znaczenia.

GPS publikuje lokalne ENU względem początku scenariusza. Latitude/longitude,
wysokość n.p.m. i velocity_ned należą do adaptera/diagnostyki, nie do podstawowego
kontraktu estymatora. StateEstimate.sample_time oznacza epokę oszacowanego stanu,
a nie czas najstarszego pomiaru, który wpłynął na estymację.

TvcDemand opisuje wspólne zadanie w ramach uproszczonego miksera v1.
Alokator tłumaczy je na ActuatorSetpoint w ramach montażowych poszczególnych
aktuatorów. Identyfikator docelowego aktuatora wynika z połączenia do jego portu,
nie z przeszukiwania wiadomości przez wszystkie plant models.

ActuatorSetpoint jest żądaniem; ActuatorState jest wynikiem dynamiki plant modelu.
Ograniczenia kąta i szybkości realizuje plant. Zmierzone sprzężenie zwrotne ma
osobny kontrakt, by nie utożsamiać rzeczywistego położenia z idealnym pomiarem.
W prostym modelu może to być jawnie skonfigurowany idealny czujnik.

Moment Wrench jest już liczony względem bieżącego CG. Sumator nie dodaje ponownie
ramienia siły. Połączenie z momentem względem innego punktu wymaga adaptera.
Wiele źródeł sił łączy się przez jawny sumator, nie przez zasadę „ostatni zapis wygrywa”.

## 5. Próbki, zdarzenia i polityki wejść

Każde wejście deklaruje sposób odczytu:

- `latest`: ostatnia udostępniona próbka; może być użyta ponownie, dopóki nie jest
  przeterminowana. Domyślnie dla StateEstimate w kontrolerze i ActuatorSetpoint.
- `all_samples`: uporządkowana kolejka próbek, każda przetwarzana raz. Domyślnie
  dla IMU w EKF — estymator nie może bezwiednie zgubić pomiarów między aktywacjami.
  Przetwarzanie GPS uwzględnia osobno opóźnione sample_time; kolejka nie zastępuje
  algorytmu obsługi pomiarów historycznych.
- `events`: kolejka pojedynczych działań, niezastępowanych następną wartością.

Kolejki porządkujemy według `(available_time, source, sequence)`; na pojedynczym
porcie sequence musi rosnąć. Wybór latest dotyczy ostatniej publikacji, nie
największego sample_time. Starszy pomiar dostarczony później zachowuje swój czas;
odbiorca stosuje politykę dla pomiarów historycznych. Cofnięcie sequence jest
błędem, a reset licznika wymaga rozpoczęcia nowego przebiegu.

To polityka odbiorcy. Jedna publikacja może trafić do telemetrii jako latest,
a do estymatora jako all_samples. Kolejki mają limit; przekroczenie limitu w v1
zatrzymuje przebieg z diagnostyką. Idealna szyna nie gubi po cichu danych.

Wejście określa też `required`, `max_age` i reakcję na missing/invalid/stale.
Domyślna reakcja dla wymaganych wejść to zatrzymanie symulacji z nazwą portu i
czasem. Domyślne max_age dla okresowych danych to
`2 * max(okres_producenta, okres_odbiorcy) + zadeklarowane_opóźnienie_modelu`.
Uwzględnia to starsze próbki w all_samples między aktywacjami wolniejszego odbiorcy.
Wartości te są widoczne w rozwiązanej konfiguracji.
Port bez okresu wymaga jawnego max_age albo unbounded. Funkcje awaryjne, np.
kontrolowane zejście do neutralnego kąta, wymagają jawnego modelu/polityki;
transport nie wymyśla takiej reakcji.

Utrzymywanie ostatniej próbki nie odnawia jej ważności. Inicjalna wartość zadania
aktuatora obowiązuje do pierwszej komendy lub do zadeklarowanego startup_deadline.

Zdarzenie, np. `EngineIgnition/v1`, zawiera `event_id=(source, sequence)` oraz
`effective_time >= available_time`. Kierowane jest do konkretnego portu silnika.
Silnik obsługuje je przy pierwszej zadeklarowanej aktywacji z t >= effective_time;
w v1 zdarzenia muszą być wyrównane do siatki odbiorcy, inaczej konfiguracja jest
odrzucana. Każdy subskrybent otrzymuje zdarzenie dokładnie raz w obrębie przebiegu.
Ta gwarancja nie obejmuje restartów procesu ani HIL. Błędna sekwencja, np. ponowny
zapłon niedopuszczony modelem silnika, jest oceniana przez odbiorcę. Kolejka nie
usuwa zdarzenia przed przetworzeniem. Nie ma ukrytego TTL; anulowanie byłoby
osobnym zdarzeniem. Obecne `EngineCommandMsg.is_active` pozostaje dotychczasowym
stanem do czasu wdrożenia adaptera zdarzeń — nie uznajemy go już teraz za event.

## 6. Fazy czasu i kolejność wykonania

Przy każdym t na siatce bazowej:

1. **Ujawnienie wyników:** atomowe udostępnienie wyników plant/FMU kończących krok
   w t oraz pomiarów z available_time=t. Nie wykonujemy jeszcze callbacków
   odbiorców. Odbiorca może zobaczyć również starszą dostępną próbkę.
2. **Obliczenia w chwili t:** aktywne sensory, estymatory, kontrolery i alokatory
   wykonują się w deterministycznej kolejności zależności. Komponent deklaruje,
   czy wyjście zależy bezpośrednio od aktualnego wejścia (feedthrough).
   Nowa publikacja zostaje zatwierdzona po zakończeniu wywołania producenta;
   późniejszy odbiorca w tej samej fazie może z niej skorzystać.
3. **Przygotowanie kroku:** każdy model stanowy, który rozpoczyna krok w t,
   zatrzaskuje kompletny zestaw wejść. Direct nie omija walidacji ani czasu.
4. **Przesunięcie modeli:** model wykonuje przedział [t, t+h]. Wejścia są utrzymywane
   stałe, chyba że jawny kontrakt modelu określa inaczej. Wyniki są buforowane;
   stają się dostępne dopiero na końcu przedziału, nie w chwili wykonania C++.
5. **Obserwacja:** logowanie stanu dostępnego dla t, z indywidualnymi czasami próbek.
   Snapshot nie odczytuje wyników z przyszłych końców kroków.

Scheduler uruchamia komponent zgodnie z jego częstotliwością; publikacja sama
nie wywołuje go rekurencyjnie. Równe terminy niezależnych komponentów rozstrzyga
stabilny identyfikator, a nie kolejność wpisów w pliku. Legacy MessageBus działa
inaczej; adaptery YAML izolują jego lokalne callbacki za schedulerem fazowym.

Cykl feedthrough bez opóźnienia jest błędem walidacji. Sprzężenie przez stan plant
jest dozwolone: wyjście w t wynika z wcześniejszego kroku, a nowa komenda wpływa
na krok zaczynający się w t. Gdy to nie wystarcza, potrzebny jest jawny blok delay
albo przyszły solver. Nie wstawiamy automatycznie opóźnienia w przypadkowym miejscu.
Nawet direct actuator → thrust → dynamics podlega tej regule: jeśli aktuator
przesuwa stan do t+h, dynamika w t nie może odczytać tego przyszłego kąta.

Przykład: IMU 800 Hz, kontroler 100 Hz, zegar 4000 Hz. IMU próbuje co 1,25 ms,
kontroler co 10 ms. W chwili 10 ms natywny sensor może opublikować pomiar dla
10 ms, estymator go przetworzyć, a kontroler użyć wyniku w tej samej fazie.
EKF z all_samples przetwarza wszystkie zaległe próbki, nie tylko ostatnią.

## 7. Inicjalizacja i FMU

Przed t=0 walidujemy cały graf, role, wersje interfejsów, częstotliwości, źródła,
jednostki, ramy i wymagane wejścia. Port wejściowy ma jednego producenta; wiele
źródeł wymaga jawnego selektora, fuzji albo sumatora. Wiele odbiorców jest dozwolone.

Scenariusz określa fizyczny stan początkowy. Plant publikuje stan początkowy;
estymator otrzymuje osobno swój prior (nie jest automatycznie zasilany prawdziwym
stanem). Każdy aktuator ma określoną pozycję początkową i początkowe zadanie.
Brak pomiaru GPS w t=0 jest normalny, jeśli wejście jest opcjonalne lub ma jawną
politykę rozruchu. Nie inicjalizujemy wszystkich portów zerami jako ważnymi danymi.

FMU jest mapowane na kompletne payloady. Adapter odczytuje wszystkie osie i dopiero
potem publikuje jedną próbkę. Opis wiązania deklaruje jednostki, ramy i semantykę
czasu, a nie tylko nazwy ScalarVariable. Typ Bool w kontrakcie nie rozszerza
samoczynnie obecnego importera Real-only; wymagany jest jawny, walidowany adapter.

- Wyjście modelu stanu po doStep(t,h) zwykle opisuje stan t+h i ma
  sample_time=available_time=t+h.
- FMU czujnika, które podczas doStep tylko przetwarza wejściową próbkę z t,
  publikuje sample_time=t, available_time=t+h. Nie oznaczamy tej próbki jako
  pomiaru z t+h. To dotyczy obecnego przykładowego FMU IMU.
- GPS zmierzony w 100 ms i dostarczony w 250 ms zachowuje oba te czasy.

XML nie wystarcza do rozpoznania tych trzech przypadków; opis adaptera jawnie
wybiera pochodzenie czasu payloadu. FMU deklaruje wspierany tryb inicjalizacji.
Jeśli zależne FMU wymagają sprzężonego rozwiązania początkowego, v1 odrzuca taki
układ bez jawnych danych rozruchowych. Nie zastępuje rozwiązania zerowymi wejściami.

Po zbudowaniu kompletnego stanu początkowego następuje publikacja w t=0 i zwykłe
fazy czasu. Kontroler nie uruchamia się na niezainicjalizowanym kwaternionie ani
przed zaspokojeniem swoich wymaganych wejść.

## 8. Przejście z obecnego kodu

| Obecny element | Docelowe znaczenie / wymagane dopasowanie |
|---|---|
| ImuStateMessage | ImuMeasurement; bez idealnej orientation, jawne specific_force i dwa czasy |
| GpsStateMessage | GpsMeasurement w ENU; zachowanie czasu akwizycji |
| EstimatedStateMessage | StateEstimate; oddzielenie biasów i niejednoznacznego acceleration do diagnostyki |
| TvcLogicalCommand | TvcDemand; yaw → angle_x, pitch → angle_y |
| ActuatorCommandMessage | ActuatorSetpoint na bezpośrednim porcie konkretnego plant |
| TvcActuatorModel.getTransform | Źródło dla ActuatorState/geometrii; montaż oddzielony od kątów |
| SignalRegistry | Adapter skalarnych portów FMU do atomowych interfejsów; sama mapa Real nie jest kontraktem |
| MessageBus | Transport informacji pod kontrolą faz, bez natychmiastowej rekurencji odbiorców |

Ważna różnica migracji: obecna telemetria po kroku dynamiki nie odpowiada dokładnie
fazom powyżej, a dawny adapter FMU IMU przypisywał czas dostarczenia do wiadomości.
Zmianę timestampów trzeba pokazać jako oczekiwaną zmianę zachowania, nie ukrywać
pod identyczną nazwą pola. Aktualny aktuator ma natychmiastową odpowiedź; kontrakt
ActuatorState dopuszcza ją jako jawny idealny model, ale nie narzuca jej wszystkim
implementacjom. Dla modelu idealnego z feedthrough trzeba zadeklarować tę zależność.

## 9. Kryteria weryfikacji implementacji

Poniższe przypadki stanowią specyfikację przyszłych testów, nie raport ich wykonania.

| Przypadek | Oczekiwany wynik |
|---|---|
| IMU 800 Hz, kontroler 100 Hz na 4000 Hz | Właściwa liczba aktywacji, pełna kolejka IMU do EKF |
| Wynik doStep(0,10 ms) | Niewidoczny przed 10 ms niezależnie od czasu wykonania C++ |
| FMU przetwarza pomiar z t=0 przez 10 ms | sample_time=0, available_time=10 ms |
| GPS ma opóźnienie 150 ms | Wiek pomiaru obejmuje te 150 ms |
| Dwa różne wektory kolejnych próbek IMU | Odbiorca nigdy nie widzi osi pochodzących z różnych próbek |
| Brak danych / nowe invalid / przekroczone max_age | Rozróżnialna diagnostyka i zadeklarowana reakcja |
| Odczyt w valid_until | Próbka nieważna; powtarzanie odczytu nie odnawia czasu |
| Dwa zdarzenia zapłonu między odczytami | Oba zachowane w kolejce, każde dostarczone raz do każdego odbiorcy |
| Przepełnienie kolejki | Jawny błąd, bez cichego zgubienia danych |
| Zmiana kolejności wpisów niezależnych komponentów | Identyczne wyniki i kolejność rozstrzygania remisów |
| Pętla feedthrough / dwa źródła na jednym wejściu | Odrzucenie bez jawnego delay lub komponentu łączącego |
| Brak priora estymatora lub inicjalnego zadania plant | Błąd inicjalizacji zamiast ukrytej wartości zero |
| Komenda 5°, plant z ograniczoną szybkością | Zadane 5° i chwilowa pozycja rzeczywista są różnymi wartościami |
| Zamiana natywnego IMU na FMU | Ten sam kontrakt pól, jawne różnice czasu i modelu szumów |
| Błędna rama, jednostka lub wersja kontraktu | Błąd przed rozpoczęciem symulacji |

Przykłady, loader i fazowy preset zostały wdrożone. Następny etap to dalsze
wydzielenie kompozycji komponentów. Nie wprowadzamy jeszcze modelu CAN/Ethernetu.

## Uzupełnienie przy przykładach konfiguracji — 2026-10-18

Dodano KinematicTruth oraz porty ciśnienia, przepływu, dostępności paliwa i właściwości
masowych, aby preset z istniejącym silnikiem ciekłym nie wymagał ukrytego dostępu
między modelami. Macierz bezwładności jest dodatkowym typem strukturalnym kontraktu;
nie rozszerza obsługi FMI o tablice. Wszystkie te kontrakty mają wspólną kopertę.
Stan masowy i CG dotyczą bieżącej chwili; przepływ zużywa paliwo w następnym
przedziale integracji. KinematicTruth jest oddzielony od estymaty, choć ma podobne pola.
Przykłady: [konfiguracja komponentów](../examples/component_config/README.md).
