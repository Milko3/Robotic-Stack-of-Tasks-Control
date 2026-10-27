// main.cpp — simulation loop, scenarios and logging (Person C)
// Python reference: simulate() and __main__ in sot_panda.py
#include <iostream>
#include <iomanip>

#include "robot.hpp"
#include "tasks.hpp"
#include "sot_core.hpp"

// Scale the whole qdot down if any joint is over its limit (keeps the direction).
Eigen::VectorXd saturate(const Eigen::VectorXd& qdot, const Eigen::VectorXd& qdotMax) {
    const double s = (qdot.cwiseAbs().array() / qdotMax.array()).maxCoeff();
    return s > 1.0 ? Eigen::VectorXd(qdot / s) : qdot;
}

int main(int argc, char** argv) {
    const std::string urdf = argc > 1 ? argv[1] : "/sot/models/panda.urdf";
    Robot robot = loadPanda(urdf);
    std::cout << "Loaded Panda: nq = " << robot.model.nq
              << ", ee frame = " << robot.eeFrame
              << ", elbow frame = " << robot.elbowFrame << "\n";

    // Same scenarios as SCEN in sot_panda.py
    std::vector<Scenario> scenarios = {
        {"A",      0.58, {"pos", "ori", "elb", "post"}, false},
        {"B",      0.90, {"pos", "ori", "elb", "post"}, false},
        {"B_flat", 0.90, {"pos", "ori", "elb"},         true},
    };

    const double T = 2.0, dt = 0.002;   // TODO (C): T = 14.0 like in Python once tasks work
    for (const Scenario& sc : scenarios) {
        Eigen::VectorXd q = robot.qReady;
        std::cout << "\n=== Scenario " << sc.name << " ===\n";
        for (int k = 0; k * dt < T; ++k) {
            const double t = k * dt;
            updateKinematics(robot, q);
            std::vector<Task> tasks = makeTasks(robot, q, t, sc);

            Eigen::VectorXd qdot = sc.flat ? flatStep(tasks, robot.model.nv)
                                           : sotStep(tasks, robot.model.nv);
            qdot = saturate(qdot, robot.qdotMax);

            if (k % 250 == 0) {   // print every 0.5 s
                std::cout << std::fixed << std::setprecision(4) << "t = " << t;
                for (const Task& task : tasks) std::cout << "  |e_" << task.name << "| = " << task.error.norm();
                std::cout << "\n";
            }
            // TODO (C): log t, q, qdot, task errors to CSV for plotting
            q += qdot * dt;
        }
    }
    return 0;
}
