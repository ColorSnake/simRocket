#pragma once
#include <nlohmann/json.hpp>
#include "rocket_sil_framework/include/messages/sensor_messages.hpp"
#include "rocket_sil_framework/include/messages/estimated_state_message.hpp"
#include "rocket_sil_framework/include/control/tvc_mixer.hpp"
#include "rocket_sil_framework/include/core/rocket_state.hpp"
#include "rocket_sil_framework/include/math/transform3d.hpp"

namespace component {
nlohmann::json logPayload(const ImuStateMessage&);
nlohmann::json logPayload(const GpsStateMessage&);
nlohmann::json logPayload(const EstimatedStateMessage&);
nlohmann::json logPayload(const TvcLogicalCommand&);
nlohmann::json logPayload(const ActuatorCommandMessage&);
nlohmann::json logPayload(const RocketState&);
nlohmann::json logPayload(const Transform3D&);
}
