#include "rocket_sil_framework/include/physics/solid_motor_model.hpp"
#include <algorithm>

SolidMotorModel::SolidMotorModel(double burn_time_s, double constant_thrust_n, double total_propellant_mass_kg, double engine_position_z_m, std::shared_ptr<MessageBus> bus)
    : burn_time_s_(burn_time_s), constant_thrust_n_(constant_thrust_n), engine_position_z_m_(engine_position_z_m) {
    if (burn_time_s_ > 0.0) {
        mass_flow_rate_ = total_propellant_mass_kg / burn_time_s_;
    } else {
        mass_flow_rate_ = 0.0;
    }
    
    if (bus) {
        bus->subscribe<TvcCommandMessage>([this](const TvcCommandMessage& msg) {
            // For now, instantaneous servo response.
            // In the future, we can add a first-order low-pass filter here for servo delay.
            this->current_pitch_rad_ = msg.pitch_angle_rad;
            this->current_yaw_rad_ = msg.yaw_angle_rad;
        });
    }
}

EngineOutput SolidMotorModel::compute(double time_s, const MassProperties& mass_props) {
    EngineOutput out;
    out.thrust_body.setZero();
    out.torque_body.setZero();
    out.mass_flow_rate = 0.0;
    
    // Engine ignites at t=0 and burns until t=burn_time_s_
    if (time_s >= 0.0 && time_s <= burn_time_s_) {
        // Nozzle Frame Definition
        // Gimbal applies rotation around X (yaw) and Y (pitch)
        Eigen::AngleAxisd pitch_rot(current_pitch_rad_, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_rot(current_yaw_rad_, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond nozzle_rotation = yaw_rot * pitch_rot;
        
        Transform3D nozzle_frame(Eigen::Vector3d(0, 0, engine_position_z_m_), nozzle_rotation);
        
        // Thrust is purely along +Z in the Nozzle Frame
        Eigen::Vector3d thrust_nozzle(0.0, 0.0, constant_thrust_n_);
        
        // Transform thrust to Body Frame
        out.thrust_body = nozzle_frame.transformVectorToParent(thrust_nozzle);
        
        // Calculate torque: tau = r x F
        Eigen::Vector3d r_cg_to_nozzle = nozzle_frame.getOriginInParent() - mass_props.center_of_gravity;
        out.torque_body = r_cg_to_nozzle.cross(out.thrust_body);
        
        out.mass_flow_rate = mass_flow_rate_;
    }
    
    return out;
}
