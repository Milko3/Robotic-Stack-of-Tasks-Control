// tasks.hpp — the individual tasks (Person B)
// Each function returns a Task (J, xdot, error). updateKinematics() must have been called first.
#pragma once

#include "sot_types.hpp"
#include "robot.hpp"

// T1: end-effector position follows a horizontal circle (3 rows).
Task eePositionTask(Robot& robot, double t);

// T2: end-effector orientation, keep flange pointing straight down (3 rows).
Task eeOrientationTask(Robot& robot);

// T3: elbow (panda_link4) height -> zElbow (1 row).
Task elbowHeightTask(Robot& robot, double zElbow);

// T4: posture, q -> qReady (n rows).
Task postureTask(Robot& robot, const Eigen::VectorXd& q);

// Old example from the first sot.cpp: rotate EE around world z until yaw = 0 (1 row).
Task eeYawTask(Robot& robot);

// Build the task list for a scenario, in priority order.
std::vector<Task> makeTasks(Robot& robot, const Eigen::VectorXd& q, double t, const Scenario& sc);
