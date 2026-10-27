// robot.cpp — Panda model + kinematics helpers (Person B)
#include "robot.hpp"

#include <cmath>
#include <pinocchio/parsers/urdf.hpp>

Robot loadPanda(const std::string& urdfPath) {
    Robot r;
    pinocchio::urdf::buildModel(urdfPath, r.model);
    r.data = pinocchio::Data(r.model);
    r.eeFrame = r.model.getFrameId("panda_link7");
    r.elbowFrame = r.model.getFrameId("panda_link4");

    const int n = r.model.nv;
    r.qReady.resize(n);
    r.qReady << 0, -M_PI / 4, 0, -3 * M_PI / 4, 0, M_PI / 2, M_PI / 4;   // same as Q_READY in panda_kin.py
    r.qLower = r.model.lowerPositionLimit;
    r.qUpper = r.model.upperPositionLimit;
    r.qdotMax.resize(n);
    r.qdotMax << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61;            // same as QDOT_MAX in sot_panda.py
    return r;
}

void updateKinematics(Robot& robot, const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(robot.model, robot.data, q);
    pinocchio::computeJointJacobians(robot.model, robot.data, q);
    pinocchio::updateFramePlacements(robot.model, robot.data);
}

Eigen::MatrixXd frameJacobian(Robot& robot, pinocchio::FrameIndex frame) {
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot.model.nv);
    pinocchio::getFrameJacobian(robot.model, robot.data, frame, pinocchio::LOCAL_WORLD_ALIGNED, J);
    return J;
}
