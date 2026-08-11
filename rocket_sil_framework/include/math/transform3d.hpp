#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

class Transform3D {
public:
    Transform3D() {
        transform_.setIdentity();
    }

    Transform3D(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation) {
        transform_ = Eigen::Translation3d(translation) * rotation;
    }

    // Setters
    void setTranslation(const Eigen::Vector3d& translation) {
        transform_.translation() = translation;
    }

    void setRotation(const Eigen::Quaterniond& rotation) {
        transform_.linear() = rotation.toRotationMatrix();
    }

    // Transforms a vector from local frame to parent frame
    Eigen::Vector3d transformVectorToParent(const Eigen::Vector3d& vector_local) const {
        // Only rotation applies to directional vectors, not translation
        return transform_.linear() * vector_local;
    }

    // Transforms a point from local frame to parent frame
    Eigen::Vector3d transformPointToParent(const Eigen::Vector3d& point_local) const {
        return transform_ * point_local;
    }

    // Get the origin of this local frame in the parent frame
    Eigen::Vector3d getOriginInParent() const {
        return transform_.translation();
    }

private:
    Eigen::Isometry3d transform_;
};
