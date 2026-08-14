#pragma once

#include <cstdint>

// Ensure no padding is added so the struct size is deterministic over the network
#pragma pack(push, 1)

struct EngineTelemetry {
    double thrust_x, thrust_y, thrust_z;
} __attribute__((packed));

struct TelemetryPacket {
    uint64_t timestamp_us;
    
    // Translation
    double pos_x, pos_y, pos_z;
    double vel_x, vel_y, vel_z;
    double acc_x, acc_y, acc_z;

    // Rotation
    double quat_w, quat_x, quat_y, quat_z;
    double ang_vel_x, ang_vel_y, ang_vel_z;

    // Diagnostics (Mass and Forces)
    double mass_kg;
    double cg_z; // Center of Gravity wzdłuż osi Z
    double total_thrust_x, total_thrust_y, total_thrust_z;
    double aero_force_x, aero_force_y, aero_force_z;
    double inertia_x, inertia_y, inertia_z;
    double wind_x, wind_y, wind_z;
    
    // TVC Diagnostics
    double tvc_cmd_pitch;
    double tvc_cmd_yaw;
    double tvc_error_pitch;
    double tvc_error_yaw;
    
    uint32_t num_engines;
    // Note: the packet will be followed by num_engines * EngineTelemetry
} __attribute__((packed));

#pragma pack(pop)
