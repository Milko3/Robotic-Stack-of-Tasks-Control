// sot_core.cpp — the stack-of-tasks solver (Person A)
// Python reference: dpinv(), sot_step(), flat_step() in sot_panda.py
#include "sot_core.hpp"

Eigen::MatrixXd dpinv(const Eigen::MatrixXd& J) {
    // TODO (A): adaptive damping with SVD (Eigen::JacobiSVD), EPS = 0.05, LAM_MAX = 0.05.
    // For now: plain (undamped) pseudoinverse, moved from the first sot.cpp.
    return J.completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::MatrixXd nullspaceProjector(const Eigen::MatrixXd& J) {
    // Moved from calculateProjection() in the first sot.cpp.
    const int n = J.cols();
    return Eigen::MatrixXd::Identity(n, n) - dpinv(J) * J;
}

Eigen::VectorXd sotStep(const std::vector<Task>& tasks, int n, std::vector<double>* leaks) {
    // TODO (A): loop over tasks in priority order:
    //   dq    = dpinv(J_k * P) * (xdot_k - J_k * qdot)
    //   qdot += dq
    //   JA    = [JA; J_k],  P = I - pinv(JA) * JA
    (void)tasks;
    if (leaks) leaks->clear();
    return Eigen::VectorXd::Zero(n);
}

Eigen::VectorXd flatStep(const std::vector<Task>& tasks, int n) {
    // TODO (A): stack all J and xdot, return dpinv(J) * xdot
    (void)tasks;
    return Eigen::VectorXd::Zero(n);
}
