#include "rocket_sil_framework/include/communication/preset_log_payloads.hpp"
namespace component {
using Json = nlohmann::json;
static Json vector(const Eigen::Vector3d& v) { return Json::array({v.x(), v.y(), v.z()}); }
static Json quaternion(const Eigen::Quaterniond& q) { return Json::array({q.w(), q.x(), q.y(), q.z()}); }
Json logPayload(const ImuStateMessage& v) {
    return {{"angular_velocity_body_rad_s",vector(v.angular_velocity)}, {"specific_force_body_m_s2",vector(v.linear_acceleration)}};
}
Json logPayload(const GpsStateMessage& v) {
    return {{"position_world_m",vector(v.position_local)}, {"velocity_ned_m_s",vector(v.velocity_ned)},
        {"latitude_deg",v.latitude}, {"longitude_deg",v.longitude}, {"altitude_m",v.altitude_m}};
}
Json logPayload(const EstimatedStateMessage& v) {
    return {{"position_world_m",vector(v.position)}, {"velocity_world_m_s",vector(v.velocity)},
        {"orientation_body_to_world_wxyz",quaternion(v.orientation)}, {"angular_velocity_body_rad_s",vector(v.angular_velocity)},
        {"gyro_bias_rad_s",vector(v.gyro_bias)}, {"accel_bias_m_s2",vector(v.accel_bias)}};
}
Json logPayload(const TvcLogicalCommand& v) { return {{"angle_x_rad",v.yaw_angle_rad},{"angle_y_rad",v.pitch_angle_rad}}; }
Json logPayload(const ActuatorCommandMessage& v) { return {{"actuator_id",v.actuator_id},{"angle_x_rad",v.yaw_angle_rad},{"angle_y_rad",v.pitch_angle_rad}}; }
Json logPayload(const RocketState& v) {
    return {{"position_world_m",vector(v.position)}, {"velocity_world_m_s",vector(v.velocity)},
        {"acceleration_world_m_s2",vector(v.acceleration)}, {"orientation_body_to_world_wxyz",quaternion(v.orientation)},
        {"angular_velocity_body_rad_s",vector(v.angular_velocity)}};
}
Json logPayload(const Transform3D& v) {
    return {{"origin_body_m",vector(v.getOriginInParent())},
        {"axis_x_body",vector(v.transformVectorToParent(Eigen::Vector3d::UnitX()))},
        {"axis_y_body",vector(v.transformVectorToParent(Eigen::Vector3d::UnitY()))},
        {"axis_z_body",vector(v.transformVectorToParent(Eigen::Vector3d::UnitZ()))}};
}
}
