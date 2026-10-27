// Stack of Tasks för Franka Panda – fristående version (utan ROS)
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <cmath>

class matrix_operations {
public:
    Eigen::MatrixXd jacobian;  // 6 x nv: rad 0-2 linjär, rad 3-5 vinkel

    void set_jacobian(const pinocchio::Model& model, pinocchio::Data& data,
                      const Eigen::VectorXd& q, pinocchio::FrameIndex frameID) {
        pinocchio::computeJointJacobians(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        jacobian.setZero(6, model.nv);
        pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    }
    static Eigen::MatrixXd identity(int n) { return Eigen::MatrixXd::Identity(n, n); }
    static Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& A) {
        return A.completeOrthogonalDecomposition().pseudoInverse();
    }
};

class standard_method {
public:
    matrix_operations mo;
    Eigen::MatrixXd J1() const { return mo.jacobian.row(5); }      // yaw
    Eigen::MatrixXd J2() const { return mo.jacobian.topRows(3); }  // position

    double get_dTheta(const pinocchio::Data& data, pinocchio::FrameIndex f) const {
        const Eigen::Matrix3d& R = data.oMf[f].rotation();
        return -std::atan2(R(1, 0), R(0, 0));
    }
    Eigen::MatrixXd calculateProjection() const {
        Eigen::MatrixXd J = J1();
        return matrix_operations::identity(J.cols()) - matrix_operations::pseudoInverse(J) * J;
    }
    Eigen::VectorXd calculateDq1(double dTheta) const {
        Eigen::MatrixXd J = J1();
        Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(J.cols());
        Eigen::VectorXd e1(1); e1 << dTheta;
        return dq0 + matrix_operations::pseudoInverse(J) * (e1 - J * dq0);
    }
    Eigen::VectorXd calculateDq2(const Eigen::VectorXd& dq1, const Eigen::Vector3d& e2) const {
        Eigen::MatrixXd J = J2();
        Eigen::MatrixXd N1 = calculateProjection();
        return dq1 + matrix_operations::pseudoInverse(J * N1) * (e2 - J * dq1);
    }
};

int main(int argc, char** argv) {
    std::string urdf_path = (argc > 1) ? argv[1] : "panda.urdf";
    pinocchio::Model model;
    try { pinocchio::urdf::buildModel(urdf_path, model); }
    catch (const std::exception& e) {
        std::cerr << "Kunde inte läsa URDF: " << urdf_path << "\n" << e.what() << "\n";
        return 1;
    }
    pinocchio::Data data(model);

    std::string eeName = model.existFrame("panda_hand") ? "panda_hand" : "panda_link8";
    pinocchio::FrameIndex eeID = model.getFrameId(eeName);
    std::cout << "nq = " << model.nq << ", end-effector: " << eeName << "\n";

    Eigen::VectorXd q = pinocchio::neutral(model);
    const double home[7] = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    for (int i = 0; i < 7 && i < model.nq; i++) q[i] = home[i];

    standard_method sm;
    sm.mo.set_jacobian(model, data, q, eeID);
    Eigen::Vector3d target = data.oMf[eeID].translation() + Eigen::Vector3d(0.10, 0.0, 0.0);

    const double dt = 0.05;
    for (int step = 0; step <= 100; step++) {
        sm.mo.set_jacobian(model, data, q, eeID);
        double dTheta = sm.get_dTheta(data, eeID);
        Eigen::Vector3d e2 = target - data.oMf[eeID].translation();
        Eigen::VectorXd dq1 = sm.calculateDq1(dTheta);
        Eigen::VectorXd dq2 = sm.calculateDq2(dq1, e2);
        if (step % 10 == 0)
            std::cout << "steg " << step << "  yaw-fel = " << dTheta
                      << "  positionsfel = " << e2.norm() << " m\n";
        q = pinocchio::integrate(model, q, dq2 * dt);
    }
    std::cout << "Slutlig q: " << q.transpose() << "\n";
    return 0;
}
