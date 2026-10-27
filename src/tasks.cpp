// tasks.cpp — the individual tasks (Person B)
// Python reference: make_tasks() in sot_panda.py and panda_kin.py
#include "tasks.hpp"

#include <cmath>
#include <stdexcept>

Task eePositionTask(Robot& robot, double t) {
    Task task;
    task.name = "pos";
    task.J = frameJacobian(robot, robot.eeFrame).topRows(3);
    // TODO (B): circle reference pd(t), pd_dot(t) as in circle() in sot_panda.py
    //           error = pd - p,  xdot = pd_dot + K_pos * error
    (void)t;
    task.error = Eigen::VectorXd::Zero(3);
    task.xdot = Eigen::VectorXd::Zero(3);
    return task;
}

Task eeOrientationTask(Robot& robot) {
    Task task;
    task.name = "ori";
    task.J = frameJacobian(robot, robot.eeFrame).bottomRows(3);
    // TODO (B): error = orientation_error(R_DOWN, R) as in panda_kin.py
    //           (R = robot.data.oMf[robot.eeFrame].rotation()),  xdot = K_ori * error
    task.error = Eigen::VectorXd::Zero(3);
    task.xdot = Eigen::VectorXd::Zero(3);
    return task;
}

Task elbowHeightTask(Robot& robot, double zElbow) {
    Task task;
    task.name = "elb";
    task.J = frameJacobian(robot, robot.elbowFrame).row(2);     // only the z row
    // TODO (B): error = zElbow - z of robot.data.oMf[robot.elbowFrame],  xdot = K_elb * error
    (void)zElbow;
    task.error = Eigen::VectorXd::Zero(1);
    task.xdot = Eigen::VectorXd::Zero(1);
    return task;
}

Task postureTask(Robot& robot, const Eigen::VectorXd& q) {
    Task task;
    task.name = "post";
    const int n = robot.model.nv;
    task.J = Eigen::MatrixXd::Identity(n, n);
    // TODO (B): error = qReady - q,  xdot = K_post * error
    (void)q;
    task.error = Eigen::VectorXd::Zero(n);
    task.xdot = Eigen::VectorXd::Zero(n);
    return task;
}

Task eeYawTask(Robot& robot) {
    // Moved from the first sot.cpp (get_dTheta + row 5 of the Jacobian).
    Task task;
    task.name = "yaw";
    task.J = frameJacobian(robot, robot.eeFrame).row(5);        // angular velocity around world z
    const Eigen::Matrix3d R = robot.data.oMf[robot.eeFrame].rotation();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    task.error = Eigen::VectorXd::Constant(1, -yaw);
    task.xdot = task.error;                                     // gain 1
    return task;
}

std::vector<Task> makeTasks(Robot& robot, const Eigen::VectorXd& q, double t, const Scenario& sc) {
    std::vector<Task> tasks;
    for (const std::string& name : sc.order) {
        if (name == "pos")       tasks.push_back(eePositionTask(robot, t));
        else if (name == "ori")  tasks.push_back(eeOrientationTask(robot));
        else if (name == "elb")  tasks.push_back(elbowHeightTask(robot, sc.zElbow));
        else if (name == "post") tasks.push_back(postureTask(robot, q));
        else if (name == "yaw")  tasks.push_back(eeYawTask(robot));
        else throw std::runtime_error("Unknown task: " + name);
    }
    return tasks;
}
