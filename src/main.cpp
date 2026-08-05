#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include "rocket_sil_framework/include/physics/rocket_state.hpp"
#include "rocket_sil_framework/include/physics/rocket_inputs.hpp"
#include "rocket_sil_framework/include/physics/i_integrator.hpp"
#include "rocket_sil_framework/include/physics/euler_integrator.hpp"
#include "rocket_sil_framework/include/physics/rk4_integrator.hpp"
#include "rocket_sil_framework/include/physics/i_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/physics/native_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/physics/solid_motor_model.hpp"
#include "rocket_sil_framework/include/physics/rigid_body_mass_model.hpp"
#include "rocket_sil_framework/include/physics/rigid_body_mass_model.hpp"
#include "rocket_sil_framework/include/physics/simple_environment_model.hpp"
#include "rocket_sil_framework/include/physics/simple_aerodynamics_model.hpp"
#include "rocket_sil_framework/include/telemetry_packet.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

using namespace Eigen;
using namespace std::chrono_literals;

int main() {
    std::cout << "simRocket Lock-Step 6DoF Loop Initialized (1000Hz)." << std::endl;

    // Simulation timing parameters
    constexpr double dt = 0.001; // 1000 Hz -> 1 ms
    constexpr auto frame_duration = std::chrono::microseconds(1000);

    // Wczytywanie konfiguracji z pliku JSON
    std::ifstream config_file("config.json");
    if (!config_file.is_open()) {
        std::cerr << "Nie udalo sie otworzyc pliku config.json!" << std::endl;
        return 1;
    }
    nlohmann::json config;
    config_file >> config;

    // Odczyt parametrow
    double burn_time = config["rocket"]["engine"]["burn_time_s"];
    double thrust_n = config["rocket"]["engine"]["thrust_n"];
    double prop_mass_engine = config["rocket"]["engine"]["propellant_mass_kg"];
    double engine_pos_z = config["rocket"]["engine"]["engine_position_z_m"];
    
    double dry_mass = config["rocket"]["mass"]["dry_mass_kg"];
    double init_prop_mass = config["rocket"]["mass"]["initial_propellant_mass_kg"];
    Eigen::Vector3d inertia_diag(
        config["rocket"]["mass"]["inertia_diagonal_kg_m2"][0].get<double>(),
        config["rocket"]["mass"]["inertia_diagonal_kg_m2"][1].get<double>(),
        config["rocket"]["mass"]["inertia_diagonal_kg_m2"][2].get<double>()
    );
    double dry_cg_z = config["rocket"]["mass"]["dry_cg_z_m"];
    double prop_cg_z = config["rocket"]["mass"]["propellant_cg_z_m"];
    
    double drag_coeff = config["rocket"]["aerodynamics"]["drag_coefficient"];
    double normal_force_coeff = config["rocket"]["aerodynamics"]["normal_force_coefficient"];
    double ref_area = config["rocket"]["aerodynamics"]["reference_area_m2"];
    double cop_z = config["rocket"]["aerodynamics"]["center_of_pressure_z_m"];
    double pitch_yaw_damping = config["rocket"]["aerodynamics"]["pitch_yaw_damping_coefficient"];
    double roll_damping = config["rocket"]["aerodynamics"]["roll_damping_coefficient"];

    double gravity_z = config["environment"]["gravity_z"];
    double wind_x = config["environment"].value("wind_velocity_x_m_s", 0.0);
    double wind_y = config["environment"].value("wind_velocity_y_m_s", 0.0);
    double wind_z = config["environment"].value("wind_velocity_z_m_s", 0.0);
    double initial_pitch_y_deg = config["environment"].value("initial_pitch_y_deg", 0.0);
    double initial_yaw_x_deg = config["environment"].value("initial_yaw_x_deg", 0.0);

    // --- Physics Model Setup ---
    std::cout << "Initializing Rocket Dynamics (RK4) with Modular Physics..." << std::endl;
    std::unique_ptr<IIntegrator> integrator = std::make_unique<RK4Integrator>();
    
    auto engine_model = std::make_unique<SolidMotorModel>(burn_time, thrust_n, prop_mass_engine, engine_pos_z);
    auto mass_model = std::make_unique<RigidBodyMassModel>(dry_mass, init_prop_mass, inertia_diag, dry_cg_z, prop_cg_z);
    auto aero_model = std::make_unique<SimpleAerodynamicsModel>(drag_coeff, normal_force_coeff, ref_area, cop_z, pitch_yaw_damping, roll_damping);
    auto env_model = std::make_unique<SimpleEnvironmentModel>(Eigen::Vector3d(0, 0, gravity_z), Eigen::Vector3d(wind_x, wind_y, wind_z));

    std::unique_ptr<IRocketDynamicsModel> dynamics_model = std::make_unique<NativeRocketDynamicsModel>(
        std::move(integrator), std::move(engine_model), std::move(mass_model), std::move(aero_model), std::move(env_model)
    );

    RocketState state;
    state.position = Eigen::Vector3d(0, 0, 0); // Z=0 is ground level? 
    
    // Initial rotation: yaw around X, then pitch around Y
    Eigen::AngleAxisd pitch_rot(initial_pitch_y_deg * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rot(initial_yaw_x_deg * M_PI / 180.0, Eigen::Vector3d::UnitX());
    state.orientation = yaw_rot * pitch_rot;
    
    // Physics constants/properties (will be passed as inputs each frame)
    RocketInputs inputs;
    inputs.gravity_inertial = Vector3d(0.0, 0.0, gravity_z);

    // --- Telemetry UDP Socket Setup ---
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in telemetry_addr;
    std::memset(&telemetry_addr, 0, sizeof(telemetry_addr));
    telemetry_addr.sin_family = AF_INET;
    telemetry_addr.sin_port = htons(9876);
    inet_pton(AF_INET, "127.0.0.1", &telemetry_addr.sin_addr);

    bool running = true;
    uint64_t step_count = 0;

    std::cout << "Entering Hot-Loop..." << std::endl;

    // Main Deterministic Lock-Step Loop
    while (running) {
        // Mark the start time of the frame
        auto frame_start = std::chrono::steady_clock::now();

        // ---------------------------------------------------------
        // 1. Środowisko / FMU (Environment & Disturbances)
        // ---------------------------------------------------------
        // Calculate atmospheric density, wind models, external disturbances.
        // Step any connected FMUs via standard C-interfaces.
        Vector3d wind_velocity(0.0, 0.0, 0.0);

        // ---------------------------------------------------------
        // 2. Fizyka 6DoF (Rigid Body Dynamics)
        // ---------------------------------------------------------
        // Sum of forces and torques acting on the rocket
        inputs.force_body = Vector3d(0.0, 0.0, 0.0);
        inputs.torque_body = Vector3d(0.0, 0.0, 0.0);
        
        // Update physics model (integration)
        dynamics_model->update(dt, inputs, state);

        // ---------------------------------------------------------
        // 3. Szum Sensorów (Sensor Noise & Modelling)
        // ---------------------------------------------------------
        // Apply analytical noise to true physics state to generate sensor readings.
        // e.g., Gaussian noise to IMU, ray-casting against terrain for Altimeter.
        Vector3d imu_accel_reading = state.acceleration;    // Add noise here
        Vector3d imu_gyro_reading = state.angular_velocity; // Add noise here

        // ---------------------------------------------------------
        // 4. GNC / Sterowanie (Guidance, Navigation, Control)
        // ---------------------------------------------------------
        // Execute state estimation (Kalman Filters) using noisy sensor data.
        // Execute path planning and control algorithms (PID, LQR, TVC).
        // Produce actuator commands.
        Vector3d tvc_command(0.0, 0.0, 0.0); // E.g., Commanded gimbal angles

        // ---------------------------------------------------------
        // 5. Aktuatory (Actuators)
        // ---------------------------------------------------------
        // Apply servo delays, saturation limits, engine spool-up delays.
        // Convert commands to actual forces/torques applied in the NEXT frame.
        // force_body += ... (applied in next iteration's physics step)
        
        // ---------------------------------------------------------
        // 6. Wypychanie Telemetrii Fire-and-Forget (Telemetry)
        // ---------------------------------------------------------
        // Dispatch UDP / ZeroMQ packets non-blocking. 
        // No dynamic memory allocations here (use fixed-size structs / buffers).
        if (step_count % 10 == 0) { // Publish at 100Hz (1000Hz / 10)
            TelemetryPacket packet;
            packet.timestamp_us = step_count * 1000;
            
            packet.pos_x = state.position.x();
            packet.pos_y = state.position.y();
            packet.pos_z = state.position.z();
            packet.vel_x = state.velocity.x();
            packet.vel_y = state.velocity.y();
            packet.vel_z = state.velocity.z();
            packet.acc_x = state.acceleration.x();
            packet.acc_y = state.acceleration.y();
            packet.acc_z = state.acceleration.z();
            
            packet.quat_w = state.orientation.w();
            packet.quat_x = state.orientation.x();
            packet.quat_y = state.orientation.y();
            packet.quat_z = state.orientation.z();
            packet.ang_vel_x = state.angular_velocity.x();
            packet.ang_vel_y = state.angular_velocity.y();
            packet.ang_vel_z = state.angular_velocity.z();

            // Wypełnianie diagnostyki
            RocketDiagnostics diag = dynamics_model->getDiagnostics();
            packet.mass_kg = diag.current_mass_kg;
            packet.cg_z = diag.current_cg_z_m;
            packet.thrust_x = diag.thrust_body.x();
            packet.thrust_y = diag.thrust_body.y();
            packet.thrust_z = diag.thrust_body.z();
            packet.aero_force_x = diag.aero_force_body.x();
            packet.aero_force_y = diag.aero_force_body.y();
            packet.aero_force_z = diag.aero_force_body.z();
            packet.inertia_x = diag.inertia_diagonal_kg_m2.x();
            packet.inertia_y = diag.inertia_diagonal_kg_m2.y();
            packet.inertia_z = diag.inertia_diagonal_kg_m2.z();
            
            packet.wind_x = diag.wind_velocity_inertial.x();
            packet.wind_y = diag.wind_velocity_inertial.y();
            packet.wind_z = diag.wind_velocity_inertial.z();

            sendto(udp_socket, &packet, sizeof(packet), 0, (struct sockaddr*)&telemetry_addr, sizeof(telemetry_addr));
        }

        if (step_count % 1000 == 0) { // Print at 1Hz for debugging
            std::cout << "[simRocket] Time: " << (step_count * dt) 
                      << "s | Pos Z: " << state.position.z() << "m\n";
        }

        // ---------------------------------------------------------
        // 7. Limitacja czasu do czasu rzeczywistego (Real-Time Throttle)
        // ---------------------------------------------------------
        // Bez tego uśpienia całe 5 sekund symulacji wykonuje się w ułamek sekundy,
        // przez co ROS2/UDP zatyka się od nadmiaru pakietów z tym samym timestampem.
        std::this_thread::sleep_for(std::chrono::microseconds(1000));


        // ---------------------------------------------------------
        // Lock-Step Timing Enforcement
        // ---------------------------------------------------------
        step_count++;
        
        // Measure how long this step actually took to compute
        auto frame_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start);

        // Sleep for the remainder of the 1000Hz frame duration
        if (elapsed < frame_duration) {
            std::this_thread::sleep_for(frame_duration - elapsed);
        } else {
            // Overrun detection (computation took longer than 1ms)
            // std::cerr << "[WARNING] Frame overrun! Took " << elapsed.count() << " us\n";
        }

        // For this demo, stop after 5 seconds (5000 steps)
        if (step_count >= 5000) {
            running = false;
        }
    }

    close(udp_socket);
    std::cout << "Simulation loop completed cleanly." << std::endl;
    return 0;
}
