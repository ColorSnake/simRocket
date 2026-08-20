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
              << "imu_gyro_x,imu_gyro_y,imu_gyro_z,"
              << "imu_acc_x,imu_acc_y,imu_acc_z,"
              << "gps_lat,gps_lon,gps_alt,"
              << "est_pos_x,est_pos_y,est_pos_z,"
              << "est_vel_x,est_vel_y,est_vel_z,"
              << "est_quat_w,est_quat_x,est_quat_y,est_quat_z,"
              << "est_bg_x,est_bg_y,est_bg_z,"
              << "est_ba_x,est_ba_y,est_ba_z,"
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

void CsvLogger::log(const nlohmann::json& packet) {
    if (!is_open_) return;

    std::lock_guard<std::mutex> lock(mutex_);
        file_ << packet["time_s"].get<double>() << ","
          << packet["true"]["pos"][0].get<double>() << "," << packet["true"]["pos"][1].get<double>() << "," << packet["true"]["pos"][2].get<double>() << ","
          << packet["true"]["vel"][0].get<double>() << "," << packet["true"]["vel"][1].get<double>() << "," << packet["true"]["vel"][2].get<double>() << ","
          << packet["true"]["acc"][0].get<double>() << "," << packet["true"]["acc"][1].get<double>() << "," << packet["true"]["acc"][2].get<double>() << ","
          << packet["true"]["quat"][0].get<double>() << "," << packet["true"]["quat"][1].get<double>() << "," << packet["true"]["quat"][2].get<double>() << "," << packet["true"]["quat"][3].get<double>() << ","
          << packet["true"]["ang_vel"][0].get<double>() << "," << packet["true"]["ang_vel"][1].get<double>() << "," << packet["true"]["ang_vel"][2].get<double>() << ","
          << packet["dyn"]["mass_kg"].get<double>() << "," << packet["dyn"]["cg_z"].get<double>() << ","
          << packet["dyn"]["thrust"][0].get<double>() << "," << packet["dyn"]["thrust"][1].get<double>() << "," << packet["dyn"]["thrust"][2].get<double>() << ","
          << packet["dyn"]["aero"][0].get<double>() << "," << packet["dyn"]["aero"][1].get<double>() << "," << packet["dyn"]["aero"][2].get<double>() << ","
          << packet["dyn"]["inertia"][0].get<double>() << "," << packet["dyn"]["inertia"][1].get<double>() << "," << packet["dyn"]["inertia"][2].get<double>() << ","
          << packet["dyn"]["wind"][0].get<double>() << "," << packet["dyn"]["wind"][1].get<double>() << "," << packet["dyn"]["wind"][2].get<double>() << ","
          << packet["ctrl"]["tvc_cmd"][0].get<double>() << "," << packet["ctrl"]["tvc_cmd"][1].get<double>() << ","
          << packet["ctrl"]["tvc_err"][0].get<double>() << "," << packet["ctrl"]["tvc_err"][1].get<double>() << ","
          << packet["sensors"]["imu_gyro"][0].get<double>() << "," << packet["sensors"]["imu_gyro"][1].get<double>() << "," << packet["sensors"]["imu_gyro"][2].get<double>() << ","
          << packet["sensors"]["imu_acc"][0].get<double>() << "," << packet["sensors"]["imu_acc"][1].get<double>() << "," << packet["sensors"]["imu_acc"][2].get<double>() << ","
          << packet["sensors"]["gps"][0].get<double>() << "," << packet["sensors"]["gps"][1].get<double>() << "," << packet["sensors"]["gps"][2].get<double>() << ","
          << packet["est"]["pos"][0].get<double>() << "," << packet["est"]["pos"][1].get<double>() << "," << packet["est"]["pos"][2].get<double>() << ","
          << packet["est"]["vel"][0].get<double>() << "," << packet["est"]["vel"][1].get<double>() << "," << packet["est"]["vel"][2].get<double>() << ","
          << packet["est"]["quat"][0].get<double>() << "," << packet["est"]["quat"][1].get<double>() << "," << packet["est"]["quat"][2].get<double>() << "," << packet["est"]["quat"][3].get<double>() << ","
          << packet["est"]["bg"][0].get<double>() << "," << packet["est"]["bg"][1].get<double>() << "," << packet["est"]["bg"][2].get<double>() << ","
          << packet["est"]["ba"][0].get<double>() << "," << packet["est"]["ba"][1].get<double>() << "," << packet["est"]["ba"][2].get<double>() << ","
          << (packet.contains("engines") ? packet["engines"].size() : 0) << "\n";
}
