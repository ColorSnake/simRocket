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
#include "rocket_sil_framework/include/physics/tvc_actuator_model.hpp"
#include "rocket_sil_framework/include/physics/rigid_body_mass_model.hpp"
#include "rocket_sil_framework/include/physics/simple_environment_model.hpp"
#include "rocket_sil_framework/include/physics/simple_aerodynamics_model.hpp"
#include "rocket_sil_framework/include/telemetry_packet.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/control/tvc_controller.hpp"
#include "rocket_sil_framework/include/control/tvc_mixer.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>

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
    std::vector<std::unique_ptr<IEngineModel>> engine_models;
    std::vector<std::unique_ptr<IActuatorModel>> actuator_models;
    std::vector<uint32_t> active_actuator_ids;

    // --- Message Bus ---
    std::shared_ptr<MessageBus> message_bus = std::make_shared<MessageBus>();

    if (config["rocket"].contains("engines")) {
        for (const auto& eng_cfg : config["rocket"]["engines"]) {
            uint32_t id = eng_cfg.value("engine_id", 0);
            double prop_mass = eng_cfg.value("propellant_mass_kg", 0.0);
            
            auto curve = std::make_shared<ThrustCurve>();
            std::string profile = eng_cfg.value("thrust_profile", "constant");
            
            if (profile == "curve") {
                std::string curve_file = eng_cfg.value("curve_file", "");
                if (!curve->loadFromFile(curve_file)) {
                    std::cerr << "Blad wczytywania krzywej ciagu z pliku: " << curve_file << std::endl;
                }
            } else {
                // legacy support for constant thrust
                double burn_time = eng_cfg.value("burn_time_s", 0.0);
                double thrust_n = eng_cfg.value("thrust_n", 0.0);
                curve->addPoint(0.0, thrust_n);
                curve->addPoint(burn_time, thrust_n);
                curve->calculateProperties();
            }
            
            engine_models.push_back(std::make_unique<SolidMotorModel>(id, curve, prop_mass));
        }
    }

    if (config["rocket"].contains("actuators")) {
        for (const auto& act_cfg : config["rocket"]["actuators"]) {
            uint32_t act_id = act_cfg.value("actuator_id", 0);
            uint32_t eng_id = act_cfg.value("engine_id", 0);
            Eigen::Vector3d pos(
                act_cfg["position_m"][0].get<double>(),
                act_cfg["position_m"][1].get<double>(),
                act_cfg["position_m"][2].get<double>()
            );
            actuator_models.push_back(std::make_unique<TvcActuatorModel>(act_id, eng_id, pos, message_bus));
            active_actuator_ids.push_back(act_id);
        }
    }
    
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
    double simulation_time_s = config["environment"].value("simulation_time_s", 5.0);
    double real_time_factor = config["environment"].value("real_time_factor", 1.0);
    uint64_t max_steps = static_cast<uint64_t>(simulation_time_s / dt);
    
    // TVC Control Parameters
    double tvc_max_gimbal_deg = config["control"]["tvc"].value("max_gimbal_deg", 10.0);
    double tvc_kp = config["control"]["tvc"].value("pid_kp", 0.5);
    double tvc_kd = config["control"]["tvc"].value("pid_kd", 0.1);

    // --- Control Logic ---
    std::cout << "Initializing TVC Controller and Mixer..." << std::endl;
    double max_gimbal_rad = tvc_max_gimbal_deg * M_PI / 180.0;
    auto tvc_controller = std::make_unique<TvcController>(message_bus, tvc_kp, tvc_kd, max_gimbal_rad);
    auto tvc_mixer = std::make_unique<TvcMixer>(message_bus, active_actuator_ids);

    // --- Physics Model Setup ---
    std::cout << "Initializing Rocket Dynamics (RK4) with Modular Physics..." << std::endl;
    std::unique_ptr<IIntegrator> integrator = std::make_unique<RK4Integrator>();
    
    auto mass_model = std::make_unique<RigidBodyMassModel>(dry_mass, init_prop_mass, inertia_diag, dry_cg_z, prop_cg_z);
    auto aero_model = std::make_unique<SimpleAerodynamicsModel>(drag_coeff, normal_force_coeff, ref_area, cop_z, pitch_yaw_damping, roll_damping);
    auto env_model = std::make_unique<SimpleEnvironmentModel>(Eigen::Vector3d(0, 0, gravity_z), Eigen::Vector3d(wind_x, wind_y, wind_z));

    std::unique_ptr<IRocketDynamicsModel> dynamics_model = std::make_unique<NativeRocketDynamicsModel>(
        std::move(integrator), std::move(engine_models), std::move(actuator_models), std::move(mass_model), std::move(aero_model), std::move(env_model)
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
    bool has_launched = false;
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
        ImuStateMessage imu_msg;
        imu_msg.orientation = state.orientation;
        imu_msg.angular_velocity = state.angular_velocity;
        // In reality, add noise here.
        message_bus->publish(imu_msg);

        // ---------------------------------------------------------
        // 4. GNC / Sterowanie (Guidance, Navigation, Control)
        // ---------------------------------------------------------
        tvc_controller->update(dt);

        // ---------------------------------------------------------
        // 5. Aktuatory (Actuators)
        // ---------------------------------------------------------
        // Apply servo delays, saturation limits, engine spool-up delays.
        // Convert commands to actual forces/torques applied in the NEXT frame.
        // force_body += ... (applied in next iteration's physics step)
        
        // ---------------------------------------------------------
        // 6. Wypychanie Telemetrii Fire-and-Forget (Telemetry)
        // ---------------------------------------------------------
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
            packet.total_thrust_x = diag.thrust_body.x();
            packet.total_thrust_y = diag.thrust_body.y();
            packet.total_thrust_z = diag.thrust_body.z();
            packet.aero_force_x = diag.aero_force_body.x();
            packet.aero_force_y = diag.aero_force_body.y();
            packet.aero_force_z = diag.aero_force_body.z();
            packet.inertia_x = diag.inertia_diagonal_kg_m2.x();
            packet.inertia_y = diag.inertia_diagonal_kg_m2.y();
            packet.inertia_z = diag.inertia_diagonal_kg_m2.z();
            
            packet.wind_x = diag.wind_velocity_inertial.x();
            packet.wind_y = diag.wind_velocity_inertial.y();
            packet.wind_z = diag.wind_velocity_inertial.z();

            packet.tvc_cmd_pitch = tvc_controller->getCmdPitch();
            packet.tvc_cmd_yaw = tvc_controller->getCmdYaw();
            packet.tvc_error_pitch = tvc_controller->getPitchError();
            packet.tvc_error_yaw = tvc_controller->getYawError();

            // Dynamiczny payload
            uint32_t active_engines = config["rocket"].contains("engines") ? config["rocket"]["engines"].size() : 0;
            packet.num_engines = active_engines;
            
            size_t total_size = sizeof(TelemetryPacket) + packet.num_engines * sizeof(EngineTelemetry);
            std::vector<uint8_t> payload(total_size);
            
            // Kopiowanie nagłowka
            std::memcpy(payload.data(), &packet, sizeof(TelemetryPacket));
            
            // W tym momencie, aby uzyskać ciąg z poszczególnych silników (wektor lokalny z obrotem), 
            // the current architecture just sums them into total thrust. 
            // For now, to fulfill the dynamic telemetry layout, we send the total thrust divided by N, 
            // or we could extract it from dynamics_model if we exposed it. Let's evenly divide total thrust for visuals.
            EngineTelemetry* engine_telemetry_ptr = reinterpret_cast<EngineTelemetry*>(payload.data() + sizeof(TelemetryPacket));
            for (uint32_t i = 0; i < packet.num_engines; ++i) {
                engine_telemetry_ptr[i].thrust_x = diag.thrust_body.x() / packet.num_engines;
                engine_telemetry_ptr[i].thrust_y = diag.thrust_body.y() / packet.num_engines;
                engine_telemetry_ptr[i].thrust_z = diag.thrust_body.z() / packet.num_engines;
            }

            sendto(udp_socket, payload.data(), payload.size(), 0, (struct sockaddr*)&telemetry_addr, sizeof(telemetry_addr));
        }

        if (step_count % 1000 == 0) { // Print at 1Hz for debugging
            std::cout << "[simRocket] Time: " << (step_count * dt) 
                      << "s | Pos Z: " << state.position.z() << "m\n";
        }

        // ---------------------------------------------------------
        // 7. Limitacja czasu do czasu rzeczywistego (Real-Time Throttle)
        // ---------------------------------------------------------
        step_count++;
        
        if (real_time_factor > 0.0) {
            auto frame_end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start);
            auto target_duration = std::chrono::duration<double, std::micro>(frame_duration.count() * real_time_factor);
            
            if (elapsed < target_duration) {
                std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::microseconds>(target_duration - elapsed));
            }
        }

        // For this demo, stop after the configured simulation time
        if (step_count >= max_steps) {
            std::cout << "[simRocket] Reached configured simulation time of " << simulation_time_s << "s. Ending simulation." << std::endl;
            running = false;
        }

        // Ground collision detection
        if (state.position.z() > 1.0) {
            has_launched = true;
        }
        if (has_launched && state.position.z() <= 0.0) {
            std::cout << "[simRocket] Rocket hit the ground (Z <= 0). Ending simulation." << std::endl;
            running = false;
        }
    }

    // Wysłanie pakietu kończącego symulację
    TelemetryPacket eof_packet;
    std::memset(&eof_packet, 0, sizeof(eof_packet));
    eof_packet.timestamp_us = 0xFFFFFFFFFFFFFFFF;
    eof_packet.num_engines = 0;
    sendto(udp_socket, &eof_packet, sizeof(eof_packet), 0, (struct sockaddr*)&telemetry_addr, sizeof(telemetry_addr));

    close(udp_socket);
    std::cout << "Simulation loop completed cleanly." << std::endl;
    return 0;
}
