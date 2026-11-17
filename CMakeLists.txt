cmake_minimum_required(VERSION 3.20)
project(simRocket VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    message(FATAL_ERROR "The current FMU loader supports Linux x86_64 (binaries/linux64) only")
endif()
include(FetchContent)

# Disable Eigen3 tests and doc generation before fetching
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
option(SIMROCKET_BUILD_TESTS "Build simRocket tests" ON)
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

# Fetch Eigen3 as header-only
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    # Only consume headers; do not register Eigen's build or test tree.
    SOURCE_SUBDIR simrocket_headers_only
)
FetchContent_MakeAvailable(Eigen3)

# Fetch nlohmann_json as header-only
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

# Fetch pugixml for FMU modelDescription.xml parsing
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG        v1.14
)
FetchContent_MakeAvailable(pugixml)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import yaml" RESULT_VARIABLE YAML_IMPORT_RESULT)
if(NOT YAML_IMPORT_RESULT EQUAL 0)
    message(FATAL_ERROR "YAML scenarios require PyYAML in ${Python3_EXECUTABLE}; install tools/requirements.txt")
endif()
# Build example FMUs in the build tree, never overwrite source artifacts.
function(add_example_fmu target identifier source xml)
    set(stage "${CMAKE_BINARY_DIR}/fmu_build/${target}")
    file(MAKE_DIRECTORY "${stage}/binaries/linux64")
    add_library(${target}_binary SHARED ${source})
    target_include_directories(${target}_binary PRIVATE "${CMAKE_SOURCE_DIR}/third_party/fmi2")
    target_link_libraries(${target}_binary PRIVATE m)
    set_target_properties(${target}_binary PROPERTIES PREFIX "" OUTPUT_NAME ${identifier}
        LIBRARY_OUTPUT_DIRECTORY "${stage}/binaries/linux64" C_VISIBILITY_PRESET hidden)
    set(output "${CMAKE_BINARY_DIR}/fmu/${target}.fmu")
    add_custom_command(OUTPUT "${output}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/fmu"
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_SOURCE_DIR}/${xml}" "${stage}/modelDescription.xml"
        COMMAND ${CMAKE_COMMAND} -E tar cf "${output}" --format=zip modelDescription.xml binaries
        WORKING_DIRECTORY "${stage}"
        DEPENDS ${target}_binary "${CMAKE_SOURCE_DIR}/${xml}" VERBATIM)
    add_custom_target(${target} ALL DEPENDS "${output}")
endfunction()
add_example_fmu(imu_sensor simRocket_ImuSensor fmu/imu_fmu/sources/imu_fmu.c fmu/imu_fmu/modelDescription.xml)

add_library(simrocket_core STATIC
    rocket_sil_framework/src/plant_models/propulsion_system.cpp
    rocket_sil_framework/src/physics/native_physics_system.cpp
    rocket_sil_framework/src/runtime/vehicle_factory.cpp
    rocket_sil_framework/src/runtime/configured_system.cpp
    rocket_sil_framework/src/runtime/simulation_loop.cpp
    rocket_sil_framework/src/communication/publication_log.cpp
    rocket_sil_framework/src/communication/preset_log_payloads.cpp
    rocket_sil_framework/src/config/scenario_loader.cpp
    rocket_sil_framework/src/core/euler_integrator.cpp
    rocket_sil_framework/src/core/rk4_integrator.cpp
    rocket_sil_framework/src/core/native_rocket_dynamics_model.cpp
    rocket_sil_framework/src/core/fmu_model.cpp
    rocket_sil_framework/src/plant_models/solid_motor_model.cpp
    rocket_sil_framework/src/vehicle/rigid_body_mass_model.cpp
    rocket_sil_framework/src/physics/simple_aerodynamics_model.cpp
    rocket_sil_framework/src/physics/simple_environment_model.cpp
    rocket_sil_framework/src/plant_models/tvc_actuator_model.cpp
    rocket_sil_framework/src/plant_models/thrust_curve.cpp
    rocket_sil_framework/src/plant_models/liquid_engine_model.cpp
    rocket_sil_framework/src/vehicle/dynamic_mass_model.cpp
    rocket_sil_framework/src/control/tvc_controller.cpp
    rocket_sil_framework/src/control/tvc_mixer.cpp
    rocket_sil_framework/src/control/profile_engine_controller.cpp
    rocket_sil_framework/src/telemetry/csv_logger.cpp
    rocket_sil_framework/src/sensors/imu_sensor_model.cpp
    rocket_sil_framework/src/sensors/gps_sensor_model.cpp
    rocket_sil_framework/src/estimation/error_state_kalman_filter.cpp
)

target_include_directories(simrocket_core PUBLIC ${eigen3_SOURCE_DIR} ${CMAKE_SOURCE_DIR})
target_link_libraries(simrocket_core PUBLIC nlohmann_json::nlohmann_json pugixml::pugixml ${CMAKE_DL_LIBS})
target_compile_definitions(simrocket_core PRIVATE
    SIMROCKET_PYTHON="${Python3_EXECUTABLE}"
    SIMROCKET_EXTRACTOR="${CMAKE_SOURCE_DIR}/tools/fmu_archive.py"
    SIMROCKET_SCENARIO_LOADER="${CMAKE_SOURCE_DIR}/tools/scenario_loader.py")
add_executable(simRocket src/main.cpp)
target_link_libraries(simRocket PRIVATE simrocket_core)

# --- GTest Setup ---
if(SIMROCKET_BUILD_TESTS)
enable_testing()
add_example_fmu(integrator Integrator fmu/integrator_fmu/integrator.c fmu/integrator_fmu/modelDescription.xml)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)
FetchContent_MakeAvailable(googletest)

add_executable(test_dynamics 
    rocket_sil_framework/tests/test_dynamics.cpp
    rocket_sil_framework/tests/test_aerodynamics.cpp
    rocket_sil_framework/tests/test_tvc_controller.cpp
    rocket_sil_framework/tests/test_message_bus.cpp
    rocket_sil_framework/tests/test_scenarios.cpp
    rocket_sil_framework/tests/test_physics_composition.cpp
    rocket_sil_framework/tests/test_component_runtime.cpp
    rocket_sil_framework/tests/test_scheduler.cpp
    rocket_sil_framework/tests/test_time_and_sensors.cpp
    rocket_sil_framework/tests/test_fmu.cpp
)
add_dependencies(test_dynamics imu_sensor integrator)
target_compile_definitions(test_dynamics PRIVATE TEST_INTEGRATOR_FMU="${CMAKE_BINARY_DIR}/fmu/integrator.fmu" TEST_IMU_FMU="${CMAKE_BINARY_DIR}/fmu/imu_sensor.fmu")
target_link_libraries(test_dynamics PRIVATE gtest_main simrocket_core)
add_test(NAME test_dynamics COMMAND test_dynamics)
add_test(NAME scenario_loader COMMAND ${Python3_EXECUTABLE} "${CMAKE_SOURCE_DIR}/tools/test_scenario_loader.py" $<TARGET_FILE:simRocket>)
add_test(NAME fmu_archive COMMAND ${Python3_EXECUTABLE} "${CMAKE_SOURCE_DIR}/tools/test_fmu_archive.py")
add_test(NAME simulation_cli COMMAND ${Python3_EXECUTABLE} "${CMAKE_SOURCE_DIR}/tools/test_simulation_cli.py" $<TARGET_FILE:simRocket> "${CMAKE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}")
set_tests_properties(test_dynamics PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()
