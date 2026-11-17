# Komponenty, interfejsy i idealna szyna danych

Status: punkty 1–3 zakończone; punkt 4 wdrożony dla presetu 2026-10-28. Konfiguracja YAML działa przez
[loader i adaptery](scenario_loader.md). Działa [runtime fazowy](component_runtime.md); dowolne składanie grafu pozostaje planem.
Obecny runtime opisano w [architecture.md](architecture.md).

## Podział odpowiedzialności

- **Logic models** przetwarzają informacje i podejmują decyzje: estymacja,
  logika misji, sterowanie.
- **Controller models** są podzbiorem logic models i wyznaczają sterowanie.
- **Plant models** opisują fizyczne zachowanie silników, aktuatorów i mechanizmów.
- Implementacja natywna lub FMU jest niezależna od kategorii komponentu.
  FMU może implementować logikę albo obiekt fizyczny.

Przykład TVC: kontroler otrzymuje estymowaną orientację z idealnej szyny,
wyznacza zadany kąt i przekazuje go bezpośrednim portem do plant modelu
aktuatora. Plant model wylicza rzeczywisty kąt, uwzględniając własną dynamikę
i ograniczenia. Ten kąt trafia do modelu kierunku ciągu, a siły i momenty do
dynamiki rakiety. Zadany kąt i rzeczywisty kąt są osobnymi wielkościami.

## Dwa rodzaje połączeń

1. **Szyna danych** — wymiana pomiarów, estymat, komend i statusów między
   elementami logicznymi oraz interfejsami urządzeń.
2. **Bezpośrednie porty** — zadawanie sterowania plant modelom i sprzężenia
   fizyczne między modelami. Nie tworzymy osobnej magistrali fizyki.

Każdy interfejs określa nazwy pól, typy, jednostki, układ współrzędnych,
kierunek przepływu i znaczenie czasu. Połączenie bezpośrednie nadal podlega
harmonogramowi symulacji. Nie daje swobodnego dostępu do wewnętrznego stanu modelu.
Sprzężenia wymagające rozwiązywania równań nie są zastępowane wymianą wiadomości.

## Idealna szyna

Na początku nie wybieramy CAN, Ethernetu ani innego protokołu. Szyna nie ma
strat, ograniczeń przepustowości ani dodatkowego opóźnienia transmisji.
Dostarcza dane deterministycznie według czasu symulacji i jawnych punktów
wykonania. Brak opóźnienia transmisji nie oznacza braku opóźnień próbkowania,
wykonania modelu albo kroku Co-Simulation.

Próbka zawiera źródło, czas pomiaru i ważność; numer sekwencji umożliwia
rozpoznanie nowych danych. Stan ciągły/próbkowany może korzystać z ostatniej
wartości. Zdarzenia, np. zapłon, wymagają kolejki — nie wolno ich nadpisywać
jak kolejnych wartości stanu. Odbiorca ma określoną reakcję na brak lub
przeterminowanie danych. Szczegóły faz dostarczania i początkowych wartości
należy określić i przetestować przed zmianą runtime.

Późniejszy model komunikacji może wprowadzać opóźnienia lub odwzorowywać
konkretną magistralę, bez zmiany znaczenia interfejsów modeli.

## Konfiguracja i import FMU

Rozdzielamy opis pojazdu, zestaw komponentów z połączeniami oraz scenariusz.
Typowe role (IMU, GPS, estymator, kontroler) otrzymują standardowe interfejsy
oraz domyślne połączenia. Wybór innej implementacji roli zastępuje poprzednią;
użytkownik nie musi osobno wyłączać natywnego modelu.

Po odczycie modelDescription.xml importer pokazuje porty FMU. Użytkownik
przypisuje je do interfejsów systemowych, a mapowanie jest zapisywane w osobnym,
wielokrotnie używanym opisie modelu. Nie zgadujemy znaczenia osi, jednostek
ani tego, czy przyspieszenie oznacza siłę właściwą.

Dla niestandardowych układów pozostaje jawny graf połączeń. Kolejność wykonania
wynika z zależności, częstotliwości i zdefiniowanych granic czasowych.
Cykl w grafie wymaga jawnej semantyki sprzężenia; samo sortowanie nie rozwiąże
pętli algebraicznej. YAML jest proponowanym zapisem, ale najpierw stabilizujemy
model danych; zmiana rozszerzenia pliku nie jest celem architektonicznym.

## Kontrakty pierwszego etapu

Zdefiniowano [kontrakty interfejsów v1](interface_contracts.md): payloady,
ramy/jednostki, czas pomiaru i dostępności, próbki/zdarzenia, fazy wykonania,
inicjalizację i kryteria przyszłych testów. To ukończona specyfikacja punktu 1;
wdrożenie kontraktów w runtime pozostaje w kolejnych etapach.

## Etapy i kryteria zakończenia

1. Zdefiniować interfejsy IMU, estymaty stanu, komendy TVC oraz zadanej/rzeczywistej
   pozycji aktuatora. Ustalić próbki, zdarzenia, inicjalizację i fazy dostarczania.
2. Przygotować dwa równoważne opisy scenariusza: natywne IMU i IMU jako FMU.
   Różnica ma dotyczyć wyboru implementacji oraz jej parametrów/częstotliwości.
   Mapowanie portów FMU pozostaje w opisie modelu, poza scenariuszem.
3. Wprowadzić ładowanie i walidację konfiguracji oraz adaptery obecnych modeli.
   Zachować możliwość uruchomienia dotychczasowych scenariuszy podczas migracji.
4. Wprowadzić idealną szynę i bezpośrednie porty z deterministycznym harmonogramem.
   Zweryfikować brak odczytu przyszłych danych, zachowanie różnych częstotliwości,
   dostarczanie zdarzeń i rozdzielenie kąta zadanego od rzeczywistego.
5. Zastąpić ręczne połączenia w main.cpp składaniem systemu z konfiguracji.
   Potwierdzić porównywalność wyników natywnego scenariusza przed i po migracji
   oraz jawnie oczekiwane różnice czasowe przy użyciu Co-Simulation.

Poza tym etapem: konkretne protokoły magistral, HIL, edytor graficzny,
pełny solver sprzężeń fizycznych i rozszerzenie obsługi wersji/typów FMI.

## Przykłady drugiego etapu — 2026-10-18

Przygotowano [scenariusze native/FMU](../examples/component_config/README.md).
Różnią się wyłącznie wyborem opisu modelu IMU. Pojazd, system i warunki lotu są
wspólne. Od 2026-10-20 przykłady są uruchamialne przez loader presetu.
Przy opisie pełnej ścieżki doprecyzowano w kontraktach porty GPS truth oraz napędu
(ciśnienie, przepływ paliwa, jego dostępność i właściwości masowe).
