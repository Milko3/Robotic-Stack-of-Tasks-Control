// sot_types.hpp — SHARED CONTRACT between all three parts.
// Do not change without telling the group first.
#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

// One task in the stack: "make J * qdot equal xdot".
struct Task {
    std::string name;        // e.g. "pos", "ori", "elb", "post"
    Eigen::MatrixXd J;       // task Jacobian, size m x n (n = 7 for Panda)
    Eigen::VectorXd xdot;    // desired task velocity, size m (feedforward + K * error)
    Eigen::VectorXd error;   // current task error, size m (only used for logging/plots)
};

// One experiment, same as SCEN in sot_panda.py.
struct Scenario {
    std::string name;                 // "A", "B", "B_flat"
    double zElbow;                    // elbow height target [m]
    std::vector<std::string> order;   // task names, highest priority first
    bool flat;                        // true = all tasks in one pseudoinverse (no priorities)
};
