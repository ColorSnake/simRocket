cmake_minimum_required(VERSION 3.20)
project(simRocket VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# Disable Eigen3 tests and doc generation before fetching
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

# Fetch Eigen3 as header-only
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(Eigen3)

# Fetch nlohmann_json as header-only
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

add_executable(simRocket 
    src/main.cpp
    rocket_sil_framework/src/physics/euler_integrator.cpp
    rocket_sil_framework/src/physics/rk4_integrator.cpp
    rocket_sil_framework/src/physics/native_rocket_dynamics_model.cpp
    rocket_sil_framework/src/physics/solid_motor_model.cpp
    rocket_sil_framework/src/physics/rigid_body_mass_model.cpp
    rocket_sil_framework/src/physics/simple_aerodynamics_model.cpp
)

# Modern target-based approach
target_include_directories(simRocket PRIVATE 
    ${eigen3_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)
target_link_libraries(simRocket PRIVATE nlohmann_json::nlohmann_json)

# --- GTest Setup ---
include(CTest)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)
FetchContent_MakeAvailable(googletest)

add_executable(test_dynamics 
    rocket_sil_framework/tests/test_dynamics.cpp
    rocket_sil_framework/src/physics/euler_integrator.cpp
    rocket_sil_framework/src/physics/rk4_integrator.cpp
    rocket_sil_framework/src/physics/native_rocket_dynamics_model.cpp
    rocket_sil_framework/src/physics/solid_motor_model.cpp
    rocket_sil_framework/src/physics/rigid_body_mass_model.cpp
    rocket_sil_framework/src/physics/simple_aerodynamics_model.cpp
)
target_include_directories(test_dynamics PRIVATE 
    ${eigen3_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)
target_link_libraries(test_dynamics PRIVATE gtest_main nlohmann_json::nlohmann_json)

add_test(NAME test_dynamics COMMAND test_dynamics)

