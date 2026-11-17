# Aktualizacja 2026-11-17 — etap 5a

Wydzielono API runtime, moduł składania presetu, fabrykę silników/aktuatorów
i niezależną pętlę symulacji. CLI pozostało w main.cpp.
Dalszy krok: rozdzielenie wewnętrznych części dynamiki (5b).
Szczegóły: [runtime komponentów](docs/component_runtime.md).

# Następny etap — uzgodnienia 2026-10-17

Priorytetem jest uproszczenie konfiguracji oraz wspólny opis interfejsów modeli.
Uzgodniono logic/controller models, plant models, idealną szynę informacji
bez konkretnego protokołu i bezpośrednie połączenia sterowania/fizyki.

Plan etapów i kryteria zakończenia: [architektura komponentów](docs/component_architecture.md).
Status: punkt 1 zakończony jako specyfikacja — [kontrakty v1](docs/interface_contracts.md).
Ustalono payloady, jednostki/ramy, próbki/zdarzenia, dwa czasy danych, fazy wykonania
i inicjalizację.
Punkt 2 zakończony 2026-10-18: [dwa przykłady konfiguracji](examples/component_config/README.md)
z modelem pojazdu, wspólnym systemem i opisami importu IMU. YAML sprawdzono statycznie;
od punktu 3 jest obsługiwany przez executable.
Punkt 3 zakończony 2026-10-20: loader YAML, walidacja, adaptery istniejących modeli,
czasy pomiarów i oddzielny zapis przebiegów. [Instrukcja i ograniczenia](docs/scenario_loader.md).
Punkt 4 wdrożony 2026-10-28 dla presetu: [szyna, direct i fazy](docs/component_runtime.md).
Następny punkt: wydzielenie składania systemu i pozostałych komponentów dynamiki.
Do backlogu dodano analizę wydajności, bez optymalizacji w tym etapie.

---

# Status wdrożenia — 2026-09-05

Zaimplementowano scheduler ze ścisłą siatką czasu, runtime FMI 2.0,
mapowanie portów Real, adaptery wiadomości, bezpieczne rozpakowanie ZIP,
parametry w fazie inicjalizacji, obsługę błędów i testy modelu stanowego.
Poprawiono kontrakt IMU/EKF, zapis CSV i rejestrację testów.

Aktualny opis, przykłady i jawne ograniczenia: [docs/fmu_integration.md](docs/fmu_integration.md).
Poniższy pierwotny plan pozostaje jako historia. Propozycję `system("unzip")`
zastąpiono ekstraktorem Python ZIP uruchamianym bez powłoki. Nagłówki FMI są
oficjalne; artefakty budowane są w `build/fmu`.

---

# Tests Update & FMU Integration Plan

This document outlines the architectural plan to update existing tests to cover the new `Scheduler` and to implement a full Proof of Concept for FMI (Functional Mock-up Interface) Co-Simulation.

## User Review Required
- **FMI Version**: I plan to use **FMI 2.0 Co-Simulation**, as it is the most widely supported standard in the aerospace industry (Simulink, Amesim, etc.). FMI 3.0 is newer but requires more complex handling of arrays/terminals for a simple proof of concept.
- **Dependencies**: To read the FMU's `modelDescription.xml`, we will need an XML parser. I propose adding **pugixml** via CMake `FetchContent`, as it's extremely lightweight and fast.
- **Unzipping**: Instead of adding a heavy zip library (like libzip) to C++, the loader will use a standard `system("unzip ...")` call to extract the FMU to `/tmp`. This is robust for Linux/SIL environments.

## Proposed Changes

### 1. Test Updates
- **[NEW] `test_scheduler.cpp`**: Add unit tests verifying that the `Scheduler` respects the `execution_order` array and triggers frequencies correctly (e.g., verifying a 100Hz task runs 10 times for every 1000Hz task).
- **[MODIFY] `test_scenarios.cpp`**: Update the integration tests to use the `Scheduler` and the new JSON configuration format (removing hardcoded loops in favor of the new pipeline architecture).

### 2. FMU Export (IMU Model)
We will create a standalone directory `fmu/imu_fmu/` that compiles a real `.fmu` zip file.
- **[NEW] `fmu/imu_fmu/modelDescription.xml`**: Defines the FMI 2.0 variables (e.g., `true_gyro_x` -> `measured_gyro_x`).
- **[NEW] `fmu/imu_fmu/fmi2Functions.h`**: Standard FMI 2.0 headers.
- **[NEW] `fmu/imu_fmu/imu_fmu.c`**: The C implementation of the IMU noise model using the `fmi2DoStep` API.
- **[MODIFY] `CMakeLists.txt`**: Add custom commands to compile the `.so` and zip it into `imu_sensor.fmu` alongside the XML.

### 3. FMU Loader in simRocket
- **[MODIFY] `CMakeLists.txt`**: Add `pugixml` dependency.
- **[NEW] `rocket_sil_framework/include/core/fmu_model.hpp` & `.cpp`**: A class that:
  1. Extracts the FMU archive.
  2. Parses `modelDescription.xml` to find `ValueReferences` for variables.
  3. Uses `<dlfcn.h>` (`dlopen`, `dlsym`) to load the FMI functions.
  4. Interfaces with our `MessageBus` to read true physics state and publish `ImuStateMessage`.

### 4. New Test Scenario
- **[NEW] `test_fmu.json`**: A clone of `test_liquid.json`, but replaces the native IMU sensor with the `imu_sensor.fmu`.
- **[MODIFY] `src/main.cpp`**: Add support for parsing the `"external_modules"` block in the JSON, instantiating `FmuModel`, and hooking it into the `Scheduler` based on the `execution_order`.

## Verification Plan
1. Run `make -j` to verify the FMU compiles and zips correctly, and that `simRocket` links `pugixml` and `<dlfcn.h>`.
2. Run `./build/test_dynamics` to ensure the new Scheduler tests pass.
3. Run `./build/simRocket test_fmu.json` and verify via standard output/logs that the FMI model successfully instantiated, took simulation steps, and produced telemetry.
