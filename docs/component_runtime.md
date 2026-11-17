# Runtime komponentów — etap 4

Scenariusze YAML używają `phased-preset-v1`. Dotychczasowe JSON bez
`component_runtime` zachowują synchroniczny MessageBus i dotychczasowy scheduler.

## Dane i aktywacje

Typowane porty mają jednego producenta i dowolną liczbę odbiorców. Publikacja
zapisuje niezmienny payload z nazwą źródła, numerem sekwencji, czasem akwizycji,
dostępności, granicą ważności i jakością. Czasy transportu są całkowitymi
nanosekundami; adaptery C++/FMU przeliczają sekundy na granicy. Port bezpośredni
podlega tym samym regułom czasu co temat idealnej szyny.

Każdy odbiorca ma własną kolejkę. `latest` utrzymuje ostatnią dostarczoną próbkę,
`all_samples` oddaje wszystkie nowe próbki. Nowsze `invalid` nie jest maskowane
starszą dobrą wartością. Rozróżniamy missing, invalid, stale i valid. Przekroczenie
pojemności kolejki zatrzymuje symulację; transport nie wywołuje komponentów.

IMU/GPS trafiają do kolejek estymatora. Estymator uruchamia się według własnej
częstotliwości i przetwarza zaległe pomiary. Przykładowo IMU 800 Hz i estymator
100 Hz wymagają zegara bazowego 4000 Hz oraz odpowiednio dłuższego `max_age_s`
wejścia IMU (np. 0.03 s). Nie wystarczy zmienić samej częstotliwości: polityka
wieku próbki pozostaje jawną częścią konfiguracji.

Kontroler odczytuje ostatnią estymatę, a alokator ostatnie zadanie TVC. Nastawa
aktuatora idzie przez direct. Jego aktualna transformacja jest zatrzaskiwana
przez wejście geometrii napędu. Idealny aktuator presetu ma jawny feedthrough;
aktualizacja nie jest wykonywana ponownie wewnątrz integratora fizyki.

## Fazy

1. Reveal: stan dynamiki w t i gotowe wyniki FMU stają się dostępne.
2. Compute: sensory → estymator → kontroler → alokator; profil napędu ma własną aktywację.
3. Prepare: nastawa aktuatora, jego wyjście i wejścia modeli kroku zostają ustalone.
4. Observe: zapisujemy snapshot t, wraz z indywidualnymi czasami pomiarów.
5. Advance: modele wykonują kroki; wyniki FMU czekają do swojego czasu dostępności.

Obserwacja odbywa się przed Advance, ponieważ istniejące klasy natywne mutują
swój stan w miejscu. Daje to snapshot t bez odczytywania przyszłych wyników;
po Advance nie działa już żaden obserwator ani komponent obliczeniowy danego t.
Jest to jawne dopasowanie realizacji faz do obecnych modeli C++.

Kolejność zadań wynika z zależności i faz. Remisy rozstrzyga identyfikator.
Scheduler odrzuca cykle feedthrough, nieznane zależności i zależności do późniejszej
fazy. Obecny graf presetu jest składany w kodzie, a nie z kolejności pól YAML.

W YAML `samples.jsonl` i telemetria UDP mają teraz pierwszy snapshot w t=0.
Przy zapisie 100 Hz przez 10 s powstaje 1000 próbek od 0 do 9.99 s. Fizykę
nadal przesuwamy do 10 s. Stare JSON zachowują zapis po kroku.

## Zakres i następne prace

Transport udostępnia również kolejki zdarzeń z czasem skutku wyrównanym do siatki
odbiorcy i osobnym kursorem każdego subskrybenta. Istniejący profil przepustnicy
nadal przekazuje stan polecenia silnika; nie jest automatycznie interpretowany
jako seria zdarzeń zapłonu.

Porty adapterów używają istniejących struktur C++, zawierających także diagnostykę;
nie są jeszcze publicznym ABI ani kompletną serializacją kontraktów v1.
W szczególności GPS nadal korzysta z adaptera istniejącej struktury pomiarowej.
Napęd, masa i środowisko pozostają wewnętrznymi częściami modelu dynamiki.
Nie ma jeszcze dowolnego grafu modeli ani wymiany dowolnej roli na FMU.
Te elementy wymagają dalszego rozdzielenia kompozycji w etapie 5.

Analiza wydajności została dodana do dashboardu jako osobne przyszłe zadanie.
Ten etap weryfikuje semantykę czasu i komunikacji; nie jest optymalizacją szybkości.

## Sprawdzenie wdrożenia

2026-10-28: cztery zestawy CTest przeszły, w tym osiem nowych testów transportu
i schedulera oraz 16 testów loadera/scenariuszy. Przebiegi native i FMU wykonano
przez 10 s; każdy zapisał 1000 snapshotów od 0 do 9.99 s. Sprawdzono, że czasy
dostępności IMU i epoki estymat nie wykraczają poza czas snapshotu.
Osobno przeszły testy konfiguracji mostka ROS i geometrii wizualizacji.

## Organizacja transportu i zrzut publikacji

Transport przeniesiono z `include/core` do `include/communication`.
Zapis i serializacja są w `src/communication`. Opcja `--log-bus PATH` zapisuje
wszystkie publikacje podłączonych portów jako JSONL (bus oraz direct), niezależnie
od częstotliwości telemetrii. Opis API, pól i przykłady:
[komunikacja](modules/communication.md).

## Wydzielenie runtime — etap 5a, 2026-11-17

`src/main.cpp` obsługuje argumenty CLI, ładuje konfigurację i wywołuje
`simrocket::runConfiguredSystem(config, options)`. API znajduje się w
`rocket_sil_framework/include/runtime/simulation.hpp`. Nie wymaga argc/argv,
więc ten sam runtime można wywołać z innego programu C++.

Kod w `rocket_sil_framework/src/runtime/` ma następujące odpowiedzialności:

- `configured_system.cpp`: tworzenie modeli, połączenia, adaptery, rejestracja
  zadań, telemetria i uruchomienie presetu. Lokalne obiekty posiadają modele
  i subskrypcje przez cały przebieg; callbacki nie wychodzą poza ten zakres życia.
- `vehicle_factory.cpp`: fabryka silników i aktuatorów na podstawie konfiguracji.
  Zwraca właścicieli modeli oraz nieposiadające widoki do adapterów; przeniesienie
  unique_ptr do dynamiki zachowuje adresy obiektów.
- `simulation_loop.cpp`: zegar, wywołanie tick, ograniczanie czasu rzeczywistego
  i dotychczasowe warunki zakończenia/kontaktu z ziemią. Nie zna modeli ani portów.

To pierwszy krok reorganizacji, nie uniwersalny builder grafu. Moduł składania
pozostaje rozbudowany i obejmuje zgodność ze starym JSON. Rozdzielenie masy,
środowiska i napędu wewnątrz dynamiki oraz dalszy podział adapterów pozostają
do wykonania. Semantyka real_time_factor i warunki zakończenia nie zostały zmienione.
