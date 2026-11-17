# Architektura simRocket

Aktualizacja 2026-10-28: scenariusze YAML używają [runtime fazowego](component_runtime.md)
z kolejkami idealnej szyny i portami direct. Specyfikacja obejmuje też docelowe
interfejsy nieudostępnione jeszcze przez preset; zakres i ograniczenia podano w opisie runtime.

Natywne modele silników, aktuatorów, masy, aerodynamiki i środowiska są składane
w `NativeRocketDynamicsModel`. Integrator Euler/RK4 pozostaje wymienny.
Aplikacja i testy współdzielą bibliotekę `simrocket_core`.

`Scheduler` działa na stałej siatce czasu. Każdy callback otrzymuje własny okres.
Częstotliwości muszą dzielić częstotliwość bazową. Natywna dynamika działa w każdym
takcie bazowym. Sensory i sterowanie poprzedzają dynamikę; telemetria jest po niej.

`MessageBus` wywołuje subskrybentów synchronicznie w wątku symulacji. Nie jest
thread-safe. `subscribeScoped` pozwala odłączyć odbiorcę wraz z końcem jego życia.
EKF jest uruchamiany pomiarami IMU/GPS, a krok bierze ze znaczników czasu IMU.

`SignalRegistry` udostępnia nazwane sygnały Real z czasem i jednostką.
`FmuComponent` mapuje je przez JSON na zmienne FMI. `FmuModel` jest niezależnym
od rakiety runtime FMI 2.0 Co-Simulation dla Linux x86_64.
Wyniki kroku FMU są buforowane do czasu końca tego kroku. Wejścia są utrzymywane
stałe podczas kroku; sprzężenia początkowe nie są rozwiązywane iteracyjnie.

Świat: ENU. Kwaternion: body → world. IMU: body, rad/s i siła właściwa m/s².
Telemetria UDP MessagePack zawiera również nazwane sygnały z ich czasem.
CSV jest osobnym odbiornikiem wybranego zestawu pól.

Szczegóły kontraktów, konfiguracja, przykłady i ograniczenia:
[Integracja FMU](fmu_integration.md).

## Uzgodniony następny etap

[Komponenty, interfejsy i idealna szyna danych](component_architecture.md)
opisują docelowy podział logic/controller models i plant models oraz plan migracji.
To kierunek zaakceptowany 2026-10-17, jeszcze niewdrożony w runtime.

Szczegółowa specyfikacja następnego etapu: [Kontrakty interfejsów v1](interface_contracts.md).
