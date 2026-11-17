#include "rocket_sil_framework/include/runtime/vehicle_factory.hpp"
#include "rocket_sil_framework/include/communication/publication_log.hpp"
#include "rocket_sil_framework/include/communication/preset_log_payloads.hpp"
#include "rocket_sil_framework/include/core/phased_scheduler.hpp"
#include <iostream>
#include "rocket_sil_framework/include/config/scenario_loader.hpp"
#include <set>
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
#include "rocket_sil_framework/include/core/fmu_component.hpp"
#include "rocket_sil_framework/include/plant_models/solid_motor_model.hpp"
#include "rocket_sil_framework/include/core/scheduler.hpp"
#include "rocket_sil_framework/include/plant_models/liquid_engine_model.hpp"
#include "rocket_sil_framework/include/plant_models/tvc_actuator_model.hpp"
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

#include "rocket_sil_framework/include/runtime/simulation.hpp"
#include "rocket_sil_framework/include/runtime/simulation_loop.hpp"

int simrocket::runConfiguredSystem(nlohmann::json config, const RunOptions& options) {
    const auto& config_path = options.config_path;
    const auto& bus_log_path = options.bus_log_path;
    const bool log_csv = options.log_csv;
    const bool component_mode = config.contains("component_runtime");
    if (!bus_log_path.empty() && !component_mode)
        throw std::invalid_argument("--log-bus requires a component scenario (YAML or resolved JSON)");
    const double base_hz = config.value("scheduler", nlohmann::json::object()).value("base_rate_hz", 1000.0);
    if (!std::isfinite(base_hz) || base_hz <= 0 || base_hz > 1000000) throw std::invalid_argument("Invalid base_rate_hz");
    const double dt = 1.0 / base_hz;
    const auto config_dir = std::filesystem::absolute(config_path).parent_path();
    auto resolve_path = [&](const std::string& path) {
        return (config_dir / path).lexically_normal().string();
    };
    if (config.contains("external_modules")) {
        for (auto& module : config["external_modules"].items())
            module.value()["path"] = resolve_path(module.value().at("path").get<std::string>());
    }
    if (config["rocket"].contains("engines"))
        for (auto& engine : config["rocket"]["engines"])
            if (engine.contains("curve_file")) engine["curve_file"] = resolve_path(engine["curve_file"]);
    if (config["control"].contains("engine_controllers"))
        for (auto& controller : config["control"]["engine_controllers"])
            if (controller.contains("csv_file")) controller["csv_file"] = resolve_path(controller["csv_file"]);


    std::ofstream component_log;
    if (component_mode) {
        const auto& record = config["component_runtime"]["record"];
        auto root = std::filesystem::path(record["directory"].get<std::string>());
        std::filesystem::create_directories(root);
        std::string pattern = (root / (std::filesystem::path(config_path).stem().string() + "-XXXXXX")).string();
        std::vector<char> writable(pattern.begin(), pattern.end()); writable.push_back('\0');
        const char* directory = mkdtemp(writable.data());
        if (!directory) throw std::runtime_error("Cannot create unique run directory");
        std::filesystem::path run_dir(directory);
        std::ofstream resolved(run_dir / "resolved.json");
        if (!resolved || !(resolved << config.dump(2))) throw std::runtime_error("Cannot save resolved configuration");
        component_log.open(run_dir / "samples.jsonl");
        if (!component_log) throw std::runtime_error("Cannot open component log");
        std::cout << "[simRocket] Run directory: " << run_dir.string() << '\n';
    }
    std::cout << "simRocket Lock-Step 6DoF Loop Initialized (" << base_hz << " Hz)." << std::endl;

    std::unique_ptr<CsvLogger> csv_logger = nullptr;
    const auto telemetry_config = config.value("telemetry", nlohmann::json::object());
    if (log_csv || telemetry_config.value("log_to_file", false)) {
        auto filename = resolve_path(telemetry_config.value("log_filename", std::string("logs/sim_log.csv")));
        std::filesystem::create_directories(std::filesystem::path(filename).parent_path());
        csv_logger = std::make_unique<CsvLogger>(filename);
    }

    // Odczyt parametrow
    std::vector<std::unique_ptr<IEngineController>> engine_controllers;

    // --- Message Bus ---
    std::shared_ptr<MessageBus> message_bus = std::make_shared<MessageBus>();

    std::unique_ptr<component::PublicationLog> bus_log;
    if (!bus_log_path.empty()) bus_log = std::make_unique<component::PublicationLog>(bus_log_path);
    // Local legacy APIs are isolated behind scheduled component adapters.
    auto estimator_bus = component_mode ? std::make_shared<MessageBus>() : message_bus;
    auto controller_bus = component_mode ? std::make_shared<MessageBus>() : message_bus;
    auto allocator_bus = component_mode ? std::make_shared<MessageBus>() : message_bus;
    using namespace component;
    BusTopic<ImuStateMessage> imu_topic("imu.measurement");
    BusTopic<GpsStateMessage> gps_topic("gps.measurement");
    BusTopic<EstimatedStateMessage> estimate_topic("estimator.state");
    BusTopic<TvcLogicalCommand> demand_topic("controller.demand");
    DirectPort<ActuatorCommandMessage> setpoint_port("allocator.setpoint");
    DirectPort<Transform3D> actuator_state_port("actuator.state");
    auto propulsion_actuator_input = actuator_state_port.connect("propulsion.actuator_state");
    DirectPort<RocketState> truth_port("dynamics.state");
    auto input_age = [&](const char* role, const char* port) -> Time {
        return component_mode ? timeFromSeconds(config["component_runtime"]["components"][role]["inputs"][port]["max_age_s"]) : unbounded;
    };
    auto imu_input = imu_topic.connect("estimator.imu", Policy::all_samples, input_age("estimator", "imu"));
    auto gps_input = gps_topic.connect("estimator.gps", Policy::all_samples, input_age("estimator", "gps"), false);
    auto estimate_input = estimate_topic.connect("controller.state", Policy::latest, input_age("controller", "state"));
    auto demand_input = demand_topic.connect("allocator.demand");
    auto setpoint_input = setpoint_port.connect("actuator.setpoint", Policy::latest, input_age("actuator", "setpoint"), false);
    auto imu_truth_input = truth_port.connect("imu.truth");
    auto gps_truth_input = truth_port.connect("gps.truth");
    if (bus_log) {
        auto encode = [](const auto& value) { return logPayload(value); };
        bus_log->attach(imu_topic, "bus", encode);
        bus_log->attach(gps_topic, "bus", encode);
        bus_log->attach(estimate_topic, "bus", encode);
        bus_log->attach(demand_topic, "bus", encode);
        bus_log->attach(setpoint_port, "direct", encode);
        bus_log->attach(actuator_state_port, "direct", encode);
        bus_log->attach(truth_port, "direct", encode);
        std::cout << "[simRocket] Bus log: " << std::filesystem::absolute(bus_log_path).string() << '\n';
    }
    double component_time = 0;

    // --- Sensors ---
    std::cout << "Initializing Sensor Models..." << std::endl;

    // Native IMU is optional: if sensors.imu.enabled is false (or external_modules has imu_fmu),
    // we skip instantiating the native model. GPS is always native for now.
    bool native_imu_enabled = config.value("sensors", nlohmann::json::object())
                                     .value("imu", nlohmann::json::object())
                                     .value("enabled", true);

    std::unique_ptr<ImuSensorModel> imu_model = nullptr;
    if (native_imu_enabled) {
        imu_model = std::make_unique<ImuSensorModel>(*message_bus);
        imu_model->load_config(config);
    }
    auto gps_model = std::make_unique<GpsSensorModel>(*message_bus);
    gps_model->load_config(config);

    // Będziemy zapisywać ostatnie pomiary do telemetrii logowanej
    ImuStateMessage latest_imu;
    GpsStateMessage latest_gps;


    auto imu_observer = message_bus->subscribeScoped<ImuStateMessage>([&](const ImuStateMessage& msg) {
        if (component_mode && msg.available_time_s - msg.timestamp_s > config["component_runtime"]["components"]["estimator"]["inputs"]["imu"]["max_age_s"].get<double>() + 1e-9)
            throw std::runtime_error("estimator.imu: stale sample");
        latest_imu = msg;
        if (component_mode) {
            latest_imu.timestamp_s = timeFromSeconds(msg.timestamp_s) / 1e9;
            latest_imu.available_time_s = timeFromSeconds(msg.available_time_s) / 1e9;
            imu_topic.publish(latest_imu, timeFromSeconds(msg.timestamp_s), timeFromSeconds(msg.available_time_s));
        }
    });
    auto gps_observer = message_bus->subscribeScoped<GpsStateMessage>([&](const GpsStateMessage& msg) {
        latest_gps = msg;
        if (component_mode) {
            latest_gps.timestamp_s = timeFromSeconds(msg.timestamp_s) / 1e9;
            latest_gps.available_time_s = timeFromSeconds(msg.available_time_s) / 1e9;
            gps_topic.publish(latest_gps, timeFromSeconds(msg.timestamp_s), timeFromSeconds(msg.available_time_s));
        }
    });


    auto vehicle = simrocket::buildVehicleModels(config, message_bus);
    auto& engine_models = vehicle.engines;
    auto& actuator_models = vehicle.actuators;
    auto& actuator_views = vehicle.actuator_views;
    auto& active_actuator_ids = vehicle.actuator_ids;

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
    if (!std::isfinite(simulation_time_s) || simulation_time_s <= 0 || simulation_time_s / dt > 1e12
        || std::abs(simulation_time_s / dt - std::round(simulation_time_s / dt)) > 1e-6)
        throw std::invalid_argument("simulation_time_s must be a positive multiple of the base step");
    if (!std::isfinite(real_time_factor) || real_time_factor < 0) throw std::invalid_argument("Invalid real_time_factor");
    uint64_t max_steps = static_cast<uint64_t>(std::llround(simulation_time_s / dt));

    // TVC Control Parameters
    double tvc_max_gimbal_deg = config["control"]["tvc"].value("max_gimbal_deg", 10.0);
    double tvc_kp = config["control"]["tvc"].value("pid_kp", 0.5);
    double tvc_kd = config["control"]["tvc"].value("pid_kd", 0.1);

    // --- Control Logic ---
    std::cout << "Initializing TVC Controller and Mixer..." << std::endl;
    double max_gimbal_rad = tvc_max_gimbal_deg * M_PI / 180.0;
    auto tvc_controller = std::make_unique<TvcController>(controller_bus, tvc_kp, tvc_kd, max_gimbal_rad);
    auto tvc_mixer = std::make_unique<TvcMixer>(allocator_bus, active_actuator_ids);

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

    if (dry_mass <= 0 || !inertia_diag.allFinite() || (inertia_diag.array() <= 0).any())
        throw std::invalid_argument("Mass and principal moments of inertia must be positive");
    std::set<uint32_t> engine_ids, actuator_ids;
    for (const auto& engine : config["rocket"]["engines"])
        if (!engine_ids.insert(engine.value("engine_id", 0u)).second) throw std::invalid_argument("Duplicate engine_id");
    for (const auto& actuator : config["rocket"]["actuators"])
        if (!actuator_ids.insert(actuator.value("actuator_id", 0u)).second || !engine_ids.contains(actuator.value("engine_id", 0u)))
            throw std::invalid_argument("Duplicate actuator_id or unknown engine_id");
    // --- Physics Model Setup ---
    std::cout << "Initializing Rocket Dynamics (RK4) with Modular Physics..." << std::endl;
    std::unique_ptr<IIntegrator> integrator = std::make_unique<RK4Integrator>();

    auto aero_model = std::make_unique<SimpleAerodynamicsModel>(drag_coeff, normal_force_coeff, ref_area, cop_z, pitch_yaw_damping, roll_damping);
    auto env_model = std::make_unique<SimpleEnvironmentModel>(Eigen::Vector3d(0, 0, gravity_z), Eigen::Vector3d(wind_x, wind_y, wind_z));

    auto dynamics_model = std::make_unique<NativeRocketDynamicsModel>(
        std::move(integrator), std::move(engine_models), std::move(actuator_models), std::move(mass_model), std::move(aero_model), std::move(env_model)
    );

    dynamics_model->setExternalActuatorScheduling(component_mode);

    RocketState state;
    state.position = Eigen::Vector3d(0, 0, 0); // Z=0 is ground level?

    // Initial rotation: yaw around X, then pitch around Y
    Eigen::AngleAxisd pitch_rot(initial_pitch_y_deg * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rot(initial_yaw_x_deg * M_PI / 180.0, Eigen::Vector3d::UnitX());
    state.orientation = yaw_rot * pitch_rot;

    auto vec3 = [](const nlohmann::json& value) { return Eigen::Vector3d(value.at(0), value.at(1), value.at(2)); };
    auto quat = [](const nlohmann::json& value) { return Eigen::Quaterniond(value.at(0), value.at(1), value.at(2), value.at(3)).normalized(); };
    if (config.contains("initial_state")) {
        const auto& initial = config["initial_state"];
        state.position = vec3(initial["position_world_m"]);
        state.velocity = vec3(initial["velocity_world_m_s"]);
        state.orientation = quat(initial["orientation_body_to_world_wxyz"]);
        state.angular_velocity = vec3(initial["angular_velocity_body_rad_s"]);
    }
    double last_setpoint_time = -1;
    MessageBus::Subscription actuator_adapter;
    if (component_mode) actuator_adapter = allocator_bus->subscribeScoped<ActuatorCommandMessage>([&](const auto& msg) {
        setpoint_port.publish(msg, timeFromSeconds(component_time), timeFromSeconds(component_time));
    });
    MessageBus::Subscription demand_adapter;
    if (component_mode) demand_adapter = controller_bus->subscribeScoped<TvcLogicalCommand>([&](const auto& msg) {
        demand_topic.publish(msg, timeFromSeconds(component_time), timeFromSeconds(component_time));
    });
    Eigen::Vector3d prior_position = Eigen::Vector3d::Zero(), prior_velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond prior_orientation = state.orientation;
    if (config.contains("estimator_prior")) {
        const auto& prior = config["estimator_prior"];
        prior_position = vec3(prior["position_world_m"]);
        prior_velocity = vec3(prior["velocity_world_m_s"]);
        prior_orientation = quat(prior["orientation_body_to_world_wxyz"]);
    }

    // --- State Estimation (ES-EKF) ---
    std::cout << "Initializing Error-State Kalman Filter..." << std::endl;
    auto ekf = std::make_unique<ErrorStateKalmanFilter>(estimator_bus, prior_position, prior_orientation, Eigen::Vector3d(0.0, 0.0, gravity_z), dt, prior_velocity);
    ekf->setProcessNoise(0.5, 0.05, 0.001, 0.001); // Tuned for simulation
    ekf->setMeasurementNoise(1.0); // 1m GPS noise

    EstimatedStateMessage latest_est;
    std::memset(&latest_est, 0, sizeof(EstimatedStateMessage));
    latest_est = ekf->getEstimatedState();
    if (component_mode) ekf->setGpsMaxAge(config["component_runtime"]["components"]["estimator"]["inputs"]["gps"]["max_age_s"]);
    auto est_observer = estimator_bus->subscribeScoped<EstimatedStateMessage>([&latest_est](const EstimatedStateMessage& msg) {
        latest_est = msg;
    });

    // Physics constants/properties (will be passed as inputs each frame)
    RocketInputs inputs;
    inputs.gravity_inertial = Vector3d(0.0, 0.0, gravity_z);

    // The signal registry is the boundary between native physics and external models.
    SignalRegistry signals;
    for (const auto& prefix : {"truth.position", "truth.velocity", "truth.gyro_body", "truth.specific_force_body", "imu.gyro_body", "imu.specific_force_body"})
        for (const auto& axis : {"x", "y", "z"})
            signals.declare(std::string(prefix) + "." + axis,
                std::string(prefix).find("gyro") != std::string::npos ? "rad/s" :
                std::string(prefix).find("force") != std::string::npos ? "m/s^2" :
                std::string(prefix).find("position") != std::string::npos ? "m" : "m/s");
    auto set_vector = [&](const std::string& prefix, const Eigen::Vector3d& value, double time) {
        for (int i = 0; i < 3; ++i) signals.set(prefix + "." + std::string(1, "xyz"[i]), value[i], time);
    };
    auto update_truth = [&] {
        set_vector("truth.position", state.position, state.time);
        set_vector("truth.velocity", state.velocity, state.time);
        set_vector("truth.gyro_body", state.angular_velocity, state.time);
        set_vector("truth.specific_force_body", state.orientation.conjugate() * (state.acceleration - inputs.gravity_inertial), state.time);
    };
    auto signal_observer = message_bus->subscribeScoped<ImuStateMessage>([&](const ImuStateMessage& msg) {
        set_vector("imu.gyro_body", msg.angular_velocity, msg.timestamp_s);
        set_vector("imu.specific_force_body", msg.linear_acceleration, msg.timestamp_s);
    });
    update_truth();
    std::vector<std::unique_ptr<FmuComponent>> fmu_models;
    if (config.contains("external_modules"))
        for (const auto& [name, module] : config["external_modules"].items())
            fmu_models.push_back(std::make_unique<FmuComponent>(name, module, signals));
    // Declare every output first, so forward references between FMUs can be validated.
    int imu_publishers = native_imu_enabled ? 1 : 0;
    int tvc_publishers = 0;
    std::set<uint32_t> fmu_engine_publishers;
    for (auto& fmu : fmu_models) {
        const auto publish = fmu->config().value("publish", nlohmann::json::object());
        const auto type = publish.value("type", std::string("signals"));
        if (type == "imu") ++imu_publishers;
        else if (type == "tvc" && config["control"]["tvc"].value("enabled", true))
            throw std::invalid_argument("Disable native TVC before connecting an FMU TVC controller");
        else if (type != "signals" && type != "tvc" && type != "engine")
            throw std::invalid_argument("Unknown FMU publish adapter: " + type);
        auto validate_signal = [&](const std::string& name) {
            signals.get(name);
            bool owned = false;
            for (const auto& output : fmu->config().at("outputs").items()) if (output.value() == name) owned = true;
            if (!owned) throw std::invalid_argument("Publish adapter must reference this FMU's output: " + name);
        };
        if (type == "imu") {
            for (const auto& key : {"gyro", "accel"}) {
                auto names = publish.at(key).get<std::vector<std::string>>();
                if (names.size() != 3) throw std::invalid_argument("IMU adapter requires three axes");
                for (const auto& name : names) validate_signal(name);
            }
        } else if (type == "tvc") {
            if (++tvc_publishers > 1) throw std::invalid_argument("Multiple FMU TVC publishers");
            validate_signal(publish.at("pitch")); validate_signal(publish.at("yaw"));
        } else if (type == "engine") {
            validate_signal(publish.at("throttle")); validate_signal(publish.at("active"));
            const auto id = publish.at("engine_id").get<uint32_t>();
            if (!fmu_engine_publishers.insert(id).second) throw std::invalid_argument("Multiple FMU publishers for one engine");
            bool exists = false;
            for (const auto& engine : config["rocket"]["engines"])
                if (engine.value("engine_id", 0u) == id && engine.value("type", "solid") == "liquid") exists = true;
            if (!exists) throw std::invalid_argument("FMU engine adapter requires an existing liquid engine");
            for (const auto& controller : config["control"].value("engine_controllers", nlohmann::json::array()))
                if (controller.value("engine_id", 0u) == id) throw std::invalid_argument("Remove native profile controller for FMU-controlled engine");
        }
        fmu->initialize(simulation_time_s);
    }
    if (imu_publishers > 1) throw std::invalid_argument("Multiple IMU publishers: disable native IMU or remove duplicate FMU adapter");
    auto publish_fmu = [&](FmuComponent& fmu, double time) {
        if (!fmu.publishReady(time)) return;
        const auto pub = fmu.config().value("publish", nlohmann::json::object());
        auto read = [&](const std::string& key) { return signals.get(pub.at(key).get<std::string>()).value; };
        auto vector = [&](const std::string& key) {
            auto names = pub.at(key).get<std::vector<std::string>>();
            if (names.size() != 3) throw std::invalid_argument("Expected three signals for " + key);
            return Eigen::Vector3d(signals.get(names[0]).value, signals.get(names[1]).value, signals.get(names[2]).value);
        };
        const auto type = pub.value("type", std::string("signals"));
        if (type == "imu") {
            ImuStateMessage msg;
            msg.timestamp_s = fmu.sampleTime();
            msg.available_time_s = fmu.availableTime();
            msg.angular_velocity = vector("gyro"); msg.linear_acceleration = vector("accel");
            message_bus->publish(msg);
        } else if (type == "tvc") {
            TvcLogicalCommand msg; msg.pitch_angle_rad = read("pitch"); msg.yaw_angle_rad = read("yaw");
            message_bus->publish(msg);
        } else if (type == "engine") {
            message_bus->publish(EngineCommandMsg(pub.at("engine_id").get<uint32_t>(), read("throttle"), read("active") > 0.5));
        }
    };

    // --- Telemetry UDP Socket Setup ---
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in telemetry_addr;
    std::memset(&telemetry_addr, 0, sizeof(telemetry_addr));
    telemetry_addr.sin_family = AF_INET;
    telemetry_addr.sin_port = htons(9876);
    inet_pton(AF_INET, "127.0.0.1", &telemetry_addr.sin_addr);
    double freq_sensors = config.value("sensors", nlohmann::json::object()).value("imu", nlohmann::json::object()).value("update_rate_hz", 1000.0);
    double freq_gnc = config.value("control", nlohmann::json::object()).value("update_rate_hz", 100.0);
    double freq_telemetry = config.value("telemetry", nlohmann::json::object()).value("update_rate_hz", 100.0);

    Scheduler scheduler;
    PhasedScheduler phased;
    auto add_task = [&](const std::string& name, double hz, uint32_t priority,
                        std::function<void(double,double)> callback) {
        if (!component_mode) { scheduler.addTask(name, hz, priority, std::move(callback)); return; }
        Phase phase = Phase::compute;
        std::vector<std::string> after;
        if (name == "imu" || name == "gps") after = {"truth"};
        else if (name == "control") after = {"estimator"};
        else if (name == "dynamics") { phase = Phase::advance; after = {"actuator"}; }
        else if (name == "telemetry") phase = Phase::observe;
        else if (name != "propulsion_control") { phase = Phase::advance; after = {"fmu_inputs"}; }
        phased.addTask(name, hz, phase, after, std::move(callback));
    };

    std::vector<std::string> default_order = {"sensors", "control", "dynamics", "telemetry"};
    std::vector<std::string> exec_order = config.value("scheduler", nlohmann::json::object()).value("execution_order", default_order);

    std::set<std::string> seen_tasks;
    for (const auto& name : exec_order) {
        bool known = name == "sensors" || name == "control" || name == "dynamics" || name == "telemetry" || (component_mode && name == "propulsion_control");
        for (const auto& fmu : fmu_models) known = known || name == fmu->name();
        if (!known || !seen_tasks.insert(name).second) throw std::invalid_argument("Unknown or duplicate execution_order task: " + name);
    }
    for (const auto& name : {"sensors", "control", "dynamics", "telemetry"})
        if (!seen_tasks.contains(name)) throw std::invalid_argument(std::string("Missing execution_order task: ") + name);
    auto get_priority = [&](const std::string& name) -> uint32_t {
        for (size_t i = 0; i < exec_order.size(); ++i) {
            if (exec_order[i] == name) return static_cast<uint32_t>(i + 1);
        }
        return 999;
    };

    // Observe the state at t, then advance native dynamics to t + base_dt.
    // FMU results for t are delivered before this schedule; new results wait until t + period.
    for (auto& fmu : fmu_models) {
        const auto name = fmu->name();
        const double hz = fmu->config().value("update_rate_hz", base_hz);
        if (!std::isfinite(hz) || hz <= 0 || std::abs(simulation_time_s * hz - std::round(simulation_time_s * hz)) > 1e-7)
            throw std::invalid_argument("FMU frequency must be positive and divide the simulation duration");
        auto* component = fmu.get();
        add_task(name, hz, get_priority(name), [component](double time, double dt_s) { component->step(time, dt_s); });
    }
    add_task("imu", freq_sensors, get_priority("sensors"), [&](double time, double dt_s) {
        if (imu_model) imu_model->update(time == 0 ? 0 : dt_s, time,
            component_mode ? imu_truth_input->latest(timeFromSeconds(time)).sample->value : state);
    });
    // GPS delivery queue is serviced every base tick; acquisition has its own rate.
    add_task("gps", 0, get_priority("sensors"), [&](double time, double dt_s) {
        gps_model->update(dt_s, time,
            component_mode ? gps_truth_input->latest(timeFromSeconds(time)).sample->value : state);
    });

    if (component_mode) add_task("propulsion_control", base_hz, get_priority("propulsion_control"), [&](double time, double) {
        for (auto& controller : engine_controllers) controller->update(time);
    });
    add_task("control", freq_gnc, get_priority("control"), [&](double time, double dt_s) {
        if (component_mode) controller_bus->publish(estimate_input->latest(timeFromSeconds(time)).sample->value);
        if (component_mode && time - latest_est.timestamp_s > config["component_runtime"]["components"]["controller"]["inputs"]["state"]["max_age_s"].get<double>() + 1e-9)
            throw std::runtime_error("controller.state: stale estimate");
        if (config["control"]["tvc"].value("enabled", true)) tvc_controller->update(dt_s);
        if (!component_mode) for (auto& eng_ctrl : engine_controllers) {
            eng_ctrl->update(time);
        }
    });

    add_task("dynamics", 0.0, get_priority("dynamics"), [&](double time, double dt_s) {
        if (component_mode) {
            const auto& act = config["actuator_initial"];
            if (last_setpoint_time < 0 && time >= act["initial_setpoint"]["startup_deadline_s"].get<double>())
                throw std::runtime_error("actuator.setpoint: startup deadline exceeded");
            if (last_setpoint_time >= 0 && time - last_setpoint_time > act["inputs"]["setpoint"]["max_age_s"].get<double>() + 1e-9)
                throw std::runtime_error("actuator.setpoint: stale command");
        }
        // Dynamics model internally sums forces from engines, actuators, and aero
        inputs.force_body = Vector3d(0.0, 0.0, 0.0);
        inputs.torque_body = Vector3d(0.0, 0.0, 0.0);
        dynamics_model->update(dt_s, inputs, state);
    });

    add_task("telemetry", freq_telemetry, get_priority("telemetry"), [&](double time, double dt_s) {
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

        telemetry["engines"] = nlohmann::json::array();
        for (const auto& engine : config["rocket"]["engines"]) {
            const auto id = engine.value("engine_id", 0u);
            Eigen::Vector3d thrust = Eigen::Vector3d::Zero();
            if (diag.engine_thrust_body.contains(id)) thrust = diag.engine_thrust_body.at(id);
            telemetry["engines"].push_back({{"engine_id", id}, {"thrust", {thrust.x(), thrust.y(), thrust.z()}}});
        }

        telemetry["external"] = nlohmann::json::object();
        for (const auto& [name, sample] : signals.samples())
            telemetry["external"][name] = {{"value", sample.value}, {"time_s", sample.time}, {"unit", sample.unit}};
        if (component_log.is_open()) {
            nlohmann::json row = {{"time_s", state.time}};
            auto encode_vector = [](const Eigen::Vector3d& v) { return nlohmann::json::array({v.x(), v.y(), v.z()}); };
            for (const auto& selection : config["component_runtime"]["record"]["signals"]) {
                const std::string name = selection;
                if (name == "truth") row[name] = telemetry["true"];
                else if (name == "imu.measurement") row[name] = {
                    {"sample_time_s", latest_imu.timestamp_s}, {"available_time_s", latest_imu.available_time_s},
                    {"angular_velocity_body", encode_vector(latest_imu.angular_velocity)},
                    {"specific_force_body", encode_vector(latest_imu.linear_acceleration)}};
                else if (name == "estimator.state") {
                    row[name] = telemetry["est"]; row[name]["sample_time_s"] = latest_est.timestamp_s;
                } else if (name == "actuator.state") {
                    row[name] = nlohmann::json::array();
                    for (auto* actuator : actuator_views) {
                        const auto actual = actuator->actualState();
                        row[name].push_back({{"actuator_id", actuator->getActuatorId()}, {"sample_time_s", state.time},
                            {"angle_x_rad", actual[0]}, {"angle_y_rad", actual[1]}, {"rate_x_rad_s", actual[2]}, {"rate_y_rad_s", actual[3]}});
                    }
                }
            }
            if (!(component_log << row.dump() << '\n')) throw std::runtime_error("Component log write failed");
        }
        if (csv_logger) csv_logger->log(telemetry);
        std::vector<uint8_t> payload = nlohmann::json::to_msgpack(telemetry);
        sendto(udp_socket, payload.data(), payload.size(), 0, (struct sockaddr*)&telemetry_addr, sizeof(telemetry_addr));
    });

    if (component_mode) {
        phased.addTask("truth", 0, Phase::reveal, {}, [&](double time, double) {
            component_time = timeFromSeconds(time) / 1e9;
            state.time = component_time;
            truth_port.publish(state, timeFromSeconds(time), timeFromSeconds(time));
            update_truth();
            for (auto& fmu : fmu_models) publish_fmu(*fmu, time);
        });
        phased.addTask("estimator", config["component_runtime"]["components"]["estimator"]["rate_hz"],
            Phase::compute, {"imu", "gps"}, [&](double time, double) {
                for (const auto& sample : imu_input->take(timeFromSeconds(time))) ekf->onImuMessage(sample.sample->value);
                for (const auto& sample : gps_input->take(timeFromSeconds(time)))
                    if (sample.status == Status::valid) ekf->onGpsMessage(sample.sample->value);
                latest_est = ekf->getEstimatedState();
                estimate_topic.publish(latest_est, timeFromSeconds(latest_est.timestamp_s), timeFromSeconds(time));
            });
        phased.addTask("allocator", freq_gnc, Phase::compute, {"control"}, [&](double time, double) {
            allocator_bus->publish(demand_input->latest(timeFromSeconds(time)).sample->value);
        });
        phased.addTask("actuator", 0, Phase::prepare, {"allocator"}, [&](double time, double step) {
            const auto command = setpoint_input->latest(timeFromSeconds(time));
            if (command.status == Status::valid) {
                const auto& msg = command.sample->value;
                for (auto* actuator : actuator_views) if (actuator->getActuatorId() == msg.actuator_id)
                    actuator->setTarget(msg.yaw_angle_rad, msg.pitch_angle_rad);
                last_setpoint_time = command.sample->envelope.sample_time * 1e-9;
            } else if (command.status != Status::missing) throw std::runtime_error("actuator.setpoint: stale or invalid");
            // This preset explicitly uses an ideal feedthrough actuator.
            for (auto* actuator : actuator_views) {
                actuator->update(step);
                actuator_state_port.publish(actuator->getTransform(), timeFromSeconds(time), timeFromSeconds(time));
                dynamics_model->setActuatorTransform(actuator->getActuatorId(),
                    propulsion_actuator_input->latest(timeFromSeconds(time)).sample->value);
            }
            // Refresh diagnostics for t before observation, without advancing physics.
            dynamics_model->refreshDiagnostics(state, inputs);
        });
        phased.addTask("fmu_inputs", 0, Phase::prepare, {}, [&](double time, double) {
            if (!imu_model) imu_truth_input->latest(timeFromSeconds(time));
        });
        phased.compile(dt);
    }
    scheduler.validate(dt);
    Scheduler gps_rate_check;
    gps_rate_check.addTask("GPS acquisition", config.value("sensors", nlohmann::json::object()).value("gps", nlohmann::json::object()).value("update_rate_hz", 10.0), 0, [](double, double) {});
    gps_rate_check.validate(dt);
    // All sampling/control/FMUs must precede dynamics; telemetry observes the completed native step.
    if (get_priority("sensors") >= get_priority("dynamics") || get_priority("control") >= get_priority("dynamics")
        || get_priority("telemetry") <= get_priority("dynamics")) throw std::invalid_argument("execution_order requires sensors/control before dynamics and telemetry after dynamics");
    for (const auto& fmu : fmu_models)
        if (get_priority(fmu->name()) >= get_priority("dynamics")) throw std::invalid_argument("Place every FMU before dynamics in execution_order");
    simrocket::runSimulationLoop(state, {dt, max_steps, simulation_time_s, real_time_factor},
        [&](double time, double step) {
            if (component_mode) phased.update(time, step);
            else {
                update_truth();
                for (auto& fmu : fmu_models) publish_fmu(*fmu, state.time);
                scheduler.update(time, step);
            }
        });

    nlohmann::json eof_packet;
    eof_packet["timestamp_us"] = 0xFFFFFFFFFFFFFFFF;
    std::vector<uint8_t> eof_payload = nlohmann::json::to_msgpack(eof_packet);
    sendto(udp_socket, eof_payload.data(), eof_payload.size(), 0, (struct sockaddr*)&telemetry_addr, sizeof(telemetry_addr));

    if (bus_log) bus_log->finish();
    close(udp_socket);
    std::cout << "Simulation loop completed cleanly." << std::endl;
    return 0;
}
