// robot.hpp — Panda model + kinematics helpers (Person B)
#pragma once

#include <string>
#include <Eigen/Dense>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

struct Robot {
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex eeFrame;      // panda_link7
    pinocchio::FrameIndex elbowFrame;   // panda_link4
    Eigen::VectorXd qReady;             // start pose
    Eigen::VectorXd qLower, qUpper;     // joint limits [rad]
    Eigen::VectorXd qdotMax;            // velocity limits [rad/s]
};

// Load the URDF and fill in frames and limits.
Robot loadPanda(const std::string& urdfPath);

// Run forward kinematics + Jacobians for configuration q.
// Call this ONCE per time step before building tasks.
void updateKinematics(Robot& robot, const Eigen::VectorXd& q);

// 6 x n Jacobian of a frame (rows 0-2 linear, 3-5 angular), world-aligned.
Eigen::MatrixXd frameJacobian(Robot& robot, pinocchio::FrameIndex frame);
