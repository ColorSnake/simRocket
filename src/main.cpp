#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include "rocket_sil_framework/include/core/rocket_state.hpp"
#include "rocket_sil_framework/include/core/rocket_inputs.hpp"
#include "rocket_sil_framework/include/core/i_integrator.hpp"
#include "rocket_sil_framework/include/core/euler_integrator.hpp"
#include "rocket_sil_framework/include/core/rk4_integrator.hpp"
#include "rocket_sil_framework/include/core/i_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/core/native_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/hardware/solid_motor_model.hpp"
#include "rocket_sil_framework/include/hardware/liquid_engine_model.hpp"
#include "rocket_sil_framework/include/hardware/tvc_actuator_model.hpp"
#include "rocket_sil_framework/include/vehicle/rigid_body_mass_model.hpp"
#include "rocket_sil_framework/include/vehicle/dynamic_mass_model.hpp"
#include "rocket_sil_framework/include/physics/simple_environment_model.hpp"
#include "rocket_sil_framework/include/physics/simple_aerodynamics_model.hpp"
#include "rocket_sil_framework/include/sensors/imu_sensor_model.hpp"
#include "rocket_sil_framework/include/sensors/gps_sensor_model.hpp"
#include "rocket_sil_framework/include/estimation/error_state_kalman_filter.hpp"
#include "rocket_sil_framework/include/messages/estimated_state_message.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/control/tvc_controller.hpp"
#include "rocket_sil_framework/include/control/tvc_mixer.hpp"
#include "rocket_sil_framework/include/control/profile_engine_controller.hpp"
#include "rocket_sil_framework/include/telemetry/csv_logger.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>
#include <filesystem>

using namespace Eigen;
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    std::cout << "simRocket Lock-Step 6DoF Loop Initialized (1000Hz)." << std::endl;

    // Simulation timing parameters
    constexpr double dt = 0.001; // 1000 Hz -> 1 ms
    constexpr auto frame_duration = std::chrono::microseconds(1000);

    // Wczytywanie konfiguracji z pliku JSON
    std::string config_path = "config.json";
    bool log_csv = false;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--log-csv") {
            log_csv = true;
        } else if (arg[0] != '-') {
            config_path = arg;
        }
    }

    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        std::cerr << "Nie udalo sie otworzyc pliku " << config_path << "!" << std::endl;
        return 1;
    }
    nlohmann::json config;
    config_file >> config;
    
    std::unique_ptr<CsvLogger> csv_logger = nullptr;
    if (log_csv) {
        std::filesystem::create_directories("logs");
        csv_logger = std::make_unique<CsvLogger>("logs/sim_log.csv");
        std::cout << "[simRocket] CSV Logging enabled: logs/sim_log.csv" << std::endl;
    }

    // Odczyt parametrow
    std::vector<std::unique_ptr<IEngineModel>> engine_models;
    std::vector<std::unique_ptr<IActuatorModel>> actuator_models;
    std::vector<std::unique_ptr<IEngineController>> engine_controllers;
    std::vector<uint32_t> active_actuator_ids;

    // --- Message Bus ---
    std::shared_ptr<MessageBus> message_bus = std::make_shared<MessageBus>();

    // --- Sensors ---
    std::cout << "Initializing Sensor Models..." << std::endl;
    auto imu_model = std::make_unique<ImuSensorModel>(*message_bus);
    auto gps_model = std::make_unique<GpsSensorModel>(*message_bus);
    imu_model->load_config(config);
    gps_model->load_config(config);

    // Będziemy zapisywać ostatnie pomiary do telemetrii logowanej
    ImuStateMessage latest_imu;
    GpsStateMessage latest_gps;
    std::memset(&latest_imu, 0, sizeof(ImuStateMessage));
    std::memset(&latest_gps, 0, sizeof(GpsStateMessage));
    
    message_bus->subscribe<ImuStateMessage>([&latest_imu](const ImuStateMessage& msg) {
        latest_imu = msg;
    });
    message_bus->subscribe<GpsStateMessage>([&latest_gps](const GpsStateMessage& msg) {
        latest_gps = msg;
    });


    if (config["rocket"].contains("engines")) {
        for (const auto& eng_cfg : config["rocket"]["engines"]) {
            uint32_t id = eng_cfg.value("engine_id", 0);
            std::string engine_type = eng_cfg.value("type", "solid");
            
            if (engine_type == "liquid") {
                double vac_thrust = eng_cfg.value("max_thrust_vac_n", 0.0);
                double sl_thrust = eng_cfg.value("max_thrust_sl_n", 0.0);
                double max_mass_flow = eng_cfg.value("max_mass_flow_kg_s", 0.0);
                
                auto liquid_eng = std::make_unique<LiquidEngineModel>(id, vac_thrust, sl_thrust, max_mass_flow, message_bus);
                
                engine_models.push_back(std::move(liquid_eng));
            } else {
                double prop_mass = eng_cfg.value("propellant_mass_kg", 0.0);
                
                auto curve = std::make_shared<ThrustCurve>();
                std::string profile = eng_cfg.value("thrust_profile", "constant");
                
                if (profile == "curve") {
                    std::string curve_file = eng_cfg.value("curve_file", "");
                    if (!curve->loadFromFile(curve_file)) {
                        std::cerr << "[Error] Blad wczytywania krzywej ciagu z pliku: " << curve_file << ". Plik nie istnieje lub jest uszkodzony." << std::endl;
                        return 1; // Przerwij symulację
                    }
                } else {
                    double burn_time = eng_cfg.value("burn_time_s", 0.0);
                    double thrust_n = eng_cfg.value("thrust_n", 0.0);
                    curve->addPoint(0.0, thrust_n);
                    curve->addPoint(burn_time, thrust_n);
                    curve->calculateProperties();
                }
                
                engine_models.push_back(std::make_unique<SolidMotorModel>(id, curve, prop_mass));
            }
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
    Eigen::Vector3d inertia_diag(
        config["rocket"]["mass"]["inertia_diagonal_kg_m2"][0].get<double>(),
        config["rocket"]["mass"]["inertia_diagonal_kg_m2"][1].get<double>(),
        config["rocket"]["mass"]["inertia_diagonal_kg_m2"][2].get<double>()
    );
    double dry_cg_z = config["rocket"]["mass"]["dry_cg_z_m"];
    
    std::string mass_model_type = config["rocket"]["mass"].value("type", "rigid");
    std::unique_ptr<IMassModel> mass_model;
    
    if (mass_model_type == "dynamic" && config["rocket"]["mass"].contains("tanks")) {
        std::vector<TankConfig> tanks;
        for (const auto& t_cfg : config["rocket"]["mass"]["tanks"]) {
            TankConfig tc;
            tc.z_bottom_m = t_cfg.value("z_bottom_m", 0.0);
            tc.radius_m = t_cfg.value("radius_m", 0.1);
            tc.max_height_m = t_cfg.value("max_height_m", 1.0);
            tc.propellant_density_kg_m3 = t_cfg.value("propellant_density_kg_m3", 1000.0);
            tc.mass_kg = t_cfg.value("mass_kg", 0.0);
            tanks.push_back(tc);
        }
        mass_model = std::make_unique<DynamicMassModel>(dry_mass, dry_cg_z, inertia_diag, tanks);
    } else {
        double init_prop_mass = config["rocket"]["mass"].value("initial_propellant_mass_kg", 0.0);
        double prop_cg_z = config["rocket"]["mass"].value("propellant_cg_z_m", 0.0);
        mass_model = std::make_unique<RigidBodyMassModel>(dry_mass, init_prop_mass, inertia_diag, dry_cg_z, prop_cg_z);
    }
    
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
    
    // Parse Engine Controllers
    if (config.contains("control") && config["control"].contains("engine_controllers")) {
        for (const auto& ctrl_cfg : config["control"]["engine_controllers"]) {
            uint32_t eng_id = ctrl_cfg.value("engine_id", 0);
            std::string type = ctrl_cfg.value("type", "profile");
            
            if (type == "profile") {
                if (ctrl_cfg.contains("csv_file")) {
                    std::string csv_file = ctrl_cfg["csv_file"];
                    std::ifstream f(csv_file.c_str());
                    if (!f.good()) {
                        std::cerr << "[Error] Plik profilu przepustnicy nie istnieje: " << csv_file << std::endl;
                        return 1; // Przerwij symulację
                    }
                    engine_controllers.push_back(std::make_unique<ProfileEngineController>(eng_id, csv_file, message_bus));
                } else if (ctrl_cfg.contains("throttle_profile")) {
                    auto curve = std::make_shared<ThrustCurve>();
                    for (const auto& pt : ctrl_cfg["throttle_profile"]) {
                        curve->addPoint(pt[0].get<double>(), pt[1].get<double>());
                    }
                    curve->calculateProperties(); // needed for interpolation setup
                    engine_controllers.push_back(std::make_unique<ProfileEngineController>(eng_id, curve, message_bus));
                }
            }
        }
    }

    // --- Physics Model Setup ---
    std::cout << "Initializing Rocket Dynamics (RK4) with Modular Physics..." << std::endl;
    std::unique_ptr<IIntegrator> integrator = std::make_unique<RK4Integrator>();
    
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
    
    // --- State Estimation (ES-EKF) ---
    std::cout << "Initializing Error-State Kalman Filter..." << std::endl;
    auto ekf = std::make_unique<ErrorStateKalmanFilter>(message_bus, Eigen::Vector3d(0, 0, 0), state.orientation, Eigen::Vector3d(0.0, 0.0, gravity_z), dt);
    ekf->setProcessNoise(0.5, 0.05, 0.001, 0.001); // Tuned for simulation
    ekf->setMeasurementNoise(1.0); // 1m GPS noise

    EstimatedStateMessage latest_est;
    std::memset(&latest_est, 0, sizeof(EstimatedStateMessage));
    latest_est.orientation = state.orientation;
    message_bus->subscribe<EstimatedStateMessage>([&latest_est](const EstimatedStateMessage& msg) {
        latest_est = msg;
    });

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
    
    double telemetry_rate_hz = 100.0;
    if (config.contains("telemetry") && config["telemetry"].contains("update_rate_hz")) {
        telemetry_rate_hz = config["telemetry"]["update_rate_hz"].get<double>();
    }
    uint64_t telemetry_interval_steps = std::max<uint64_t>(1, static_cast<uint64_t>(1000.0 / telemetry_rate_hz));

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
        imu_model->update(dt, state.time, state);
        gps_model->update(dt, state.time, state);

        // ---------------------------------------------------------
        // 4. GNC / Sterowanie (Guidance, Navigation, Control)
        // ---------------------------------------------------------
        tvc_controller->update(dt);
        
        for (auto& eng_ctrl : engine_controllers) {
            eng_ctrl->update(state.time);
        }

        // ---------------------------------------------------------
        // 5. Aktuatory (Actuators)
        // ---------------------------------------------------------
        // Apply servo delays, saturation limits, engine spool-up delays.
        // Convert commands to actual forces/torques applied in the NEXT frame.
        // force_body += ... (applied in next iteration's physics step)
        
        // ---------------------------------------------------------
        // 6. Wypychanie Telemetrii Fire-and-Forget (Telemetry)
        // ---------------------------------------------------------
        if (step_count % telemetry_interval_steps == 0) {
            nlohmann::json telemetry;
            telemetry["timestamp_us"] = static_cast<uint64_t>(state.time * 1e6);
            telemetry["time_s"] = state.time;
            
            // Ideal/True State
            telemetry["true"]["pos"] = {state.position.x(), state.position.y(), state.position.z()};
            telemetry["true"]["vel"] = {state.velocity.x(), state.velocity.y(), state.velocity.z()};
            telemetry["true"]["acc"] = {state.acceleration.x(), state.acceleration.y(), state.acceleration.z()};
            telemetry["true"]["quat"] = {state.orientation.w(), state.orientation.x(), state.orientation.y(), state.orientation.z()};
            telemetry["true"]["ang_vel"] = {state.angular_velocity.x(), state.angular_velocity.y(), state.angular_velocity.z()};

            RocketDiagnostics diag = dynamics_model->getDiagnostics();
            telemetry["dyn"]["mass_kg"] = diag.current_mass_kg;
            telemetry["dyn"]["cg_z"] = diag.current_cg_z_m;
            telemetry["dyn"]["thrust"] = {diag.thrust_body.x(), diag.thrust_body.y(), diag.thrust_body.z()};
            telemetry["dyn"]["aero"] = {diag.aero_force_body.x(), diag.aero_force_body.y(), diag.aero_force_body.z()};
            telemetry["dyn"]["inertia"] = {diag.inertia_diagonal_kg_m2.x(), diag.inertia_diagonal_kg_m2.y(), diag.inertia_diagonal_kg_m2.z()};
            telemetry["dyn"]["wind"] = {diag.wind_velocity_inertial.x(), diag.wind_velocity_inertial.y(), diag.wind_velocity_inertial.z()};
            
            // Control
            telemetry["ctrl"]["tvc_cmd"] = {tvc_controller->getCmdPitch(), tvc_controller->getCmdYaw()};
            telemetry["ctrl"]["tvc_err"] = {tvc_controller->getPitchError(), tvc_controller->getYawError()};
            
            // Sensors
            telemetry["sensors"]["imu_gyro"] = {latest_imu.angular_velocity.x(), latest_imu.angular_velocity.y(), latest_imu.angular_velocity.z()};
            telemetry["sensors"]["imu_acc"] = {latest_imu.linear_acceleration.x(), latest_imu.linear_acceleration.y(), latest_imu.linear_acceleration.z()};
            telemetry["sensors"]["gps"] = {latest_gps.latitude, latest_gps.longitude, latest_gps.altitude_m};
            
            // Estimated State
            telemetry["est"]["pos"] = {latest_est.position.x(), latest_est.position.y(), latest_est.position.z()};
            telemetry["est"]["vel"] = {latest_est.velocity.x(), latest_est.velocity.y(), latest_est.velocity.z()};
            telemetry["est"]["quat"] = {latest_est.orientation.w(), latest_est.orientation.x(), latest_est.orientation.y(), latest_est.orientation.z()};
            telemetry["est"]["bg"] = {latest_est.gyro_bias.x(), latest_est.gyro_bias.y(), latest_est.gyro_bias.z()};
            telemetry["est"]["ba"] = {latest_est.accel_bias.x(), latest_est.accel_bias.y(), latest_est.accel_bias.z()};
            
            uint32_t active_engines = config["rocket"].contains("engines") ? config["rocket"]["engines"].size() : 0;
            telemetry["engines"] = nlohmann::json::array();
            for (uint32_t i = 0; i < active_engines; ++i) {
                telemetry["engines"].push_back({
                    {"thrust", {diag.thrust_body.x() / active_engines, diag.thrust_body.y() / active_engines, diag.thrust_body.z() / active_engines}}
                });
            }

            std::vector<uint8_t> payload = nlohmann::json::to_msgpack(telemetry);
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
        
        if (state.position.z() <= 0.0) {
            // Uniemożliwienie zapadania się pod ziemię przed startem
            state.position.z() = 0.0;
            if (state.velocity.z() < 0.0) {
                state.velocity.z() = 0.0;
            }
            
            // Jeśli rakieta już wystartowała (była pow. 1 m) i wróciła na ziemię - koniec symulacji
            if (has_launched) {
                std::cout << "[simRocket] Rocket hit the ground (Z <= 0). Ending simulation." << std::endl;
                running = false;
            }
        }
    }

    nlohmann::json eof_packet;
    eof_packet["timestamp_us"] = 0xFFFFFFFFFFFFFFFF;
    std::vector<uint8_t> eof_payload = nlohmann::json::to_msgpack(eof_packet);
    sendto(udp_socket, eof_payload.data(), eof_payload.size(), 0, (struct sockaddr*)&telemetry_addr, sizeof(telemetry_addr));

    close(udp_socket);
    std::cout << "Simulation loop completed cleanly." << std::endl;
    return 0;
}
