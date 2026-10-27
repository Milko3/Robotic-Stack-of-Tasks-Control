// sot_core.hpp — the stack-of-tasks solver (Person A)
#pragma once

#include "sot_types.hpp"

// Damped pseudoinverse (adaptive damping near singularities), as dpinv() in sot_panda.py.
Eigen::MatrixXd dpinv(const Eigen::MatrixXd& J);

// Null-space projector P = I - J^+ J.
Eigen::MatrixXd nullspaceProjector(const Eigen::MatrixXd& J);

// Prioritized solution: each task only uses the null space of the ones above it.
// n = number of joints. If leaks != nullptr, fill with |J_above * dq_k| per task (for evaluation).
Eigen::VectorXd sotStep(const std::vector<Task>& tasks, int n, std::vector<double>* leaks = nullptr);

// No priorities: stack all tasks into one big J and solve once.
Eigen::VectorXd flatStep(const std::vector<Task>& tasks, int n);
