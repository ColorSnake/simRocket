#include "rocket_sil_framework/include/telemetry/csv_logger.hpp"
#include <iostream>

CsvLogger::CsvLogger(const std::string& filename) : is_open_(false) {
    file_.open(filename, std::ios::out | std::ios::trunc);
    if (file_.is_open()) {
        is_open_ = true;
        // Zapis nagłówka (Header)
        file_ << "time_s,"
              << "pos_x,pos_y,pos_z,"
              << "vel_x,vel_y,vel_z,"
              << "acc_x,acc_y,acc_z,"
              << "quat_w,quat_x,quat_y,quat_z,"
              << "ang_vel_x,ang_vel_y,ang_vel_z,"
              << "mass_kg,cg_z,"
              << "thrust_x,thrust_y,thrust_z,"
              << "aero_x,aero_y,aero_z,"
              << "inertia_x,inertia_y,inertia_z,"
              << "wind_x,wind_y,wind_z,"
              << "tvc_cmd_pitch,tvc_cmd_yaw,tvc_err_pitch,tvc_err_yaw,"
              << "num_engines\n";
    } else {
        std::cerr << "[CsvLogger] Failed to open file for logging: " << filename << std::endl;
    }
}

CsvLogger::~CsvLogger() {
    if (is_open_) {
        file_.close();
    }
}

void CsvLogger::log(const TelemetryPacket& packet) {
    if (!is_open_) return;

    std::lock_guard<std::mutex> lock(mutex_);
    
    file_ << packet.timestamp_us / 1e6 << ","
          << packet.pos_x << "," << packet.pos_y << "," << packet.pos_z << ","
          << packet.vel_x << "," << packet.vel_y << "," << packet.vel_z << ","
          << packet.acc_x << "," << packet.acc_y << "," << packet.acc_z << ","
          << packet.quat_w << "," << packet.quat_x << "," << packet.quat_y << "," << packet.quat_z << ","
          << packet.ang_vel_x << "," << packet.ang_vel_y << "," << packet.ang_vel_z << ","
          << packet.mass_kg << "," << packet.cg_z << ","
          << packet.total_thrust_x << "," << packet.total_thrust_y << "," << packet.total_thrust_z << ","
          << packet.aero_force_x << "," << packet.aero_force_y << "," << packet.aero_force_z << ","
          << packet.inertia_x << "," << packet.inertia_y << "," << packet.inertia_z << ","
          << packet.wind_x << "," << packet.wind_y << "," << packet.wind_z << ","
          << packet.tvc_cmd_pitch << "," << packet.tvc_cmd_yaw << ","
          << packet.tvc_error_pitch << "," << packet.tvc_error_yaw << ","
          << packet.num_engines << "\n";
}
