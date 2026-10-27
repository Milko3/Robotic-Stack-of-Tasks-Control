#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

class matrix_operations {
public:
    Eigen::MatrixXd jacobian;

    void set_jacobian(
        const pinocchio::Model& model,
        pinocchio::Data& data,
        const Eigen::VectorXd& q,
        pinocchio::FrameIndex frameID)
    {
        pinocchio::computeJointJacobians(model, data, q);
        pinocchio::updateFramePlacements(model, data);

        jacobian = Eigen::MatrixXd::Zero(6, model.nv);

        pinocchio::getFrameJacobian(
            model, data, frameID,
            pinocchio::LOCAL_WORLD_ALIGNED,
            jacobian);
    }

    Eigen::MatrixXd findPsuedoInverse(
        const Eigen::MatrixXd& matrix) const
    {
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(matrix);
        cod.setThreshold(1e-10);
        return cod.pseudoInverse();
    }
};

class standard_method {
    matrix_operations mo;
    Eigen::VectorXd dq;
    Eigen::MatrixXd P;

public:
    // Call once at the beginning of every control cycle.
    void reset(int nv) {
        dq = Eigen::VectorXd::Zero(nv);
        P = Eigen::MatrixXd::Identity(nv, nv);
    }

    // Add one task, in highest-to-lowest priority order.
    void addTask(
        const Eigen::MatrixXd& J,
        const Eigen::VectorXd& desiredVelocity)
    {
        if (J.cols() != dq.size() ||
            J.rows() != desiredVelocity.size())
        {
            throw std::runtime_error("Invalid task dimensions");
        }

        const Eigen::MatrixXd projectedJ = J * P;
        const Eigen::MatrixXd inverse =
            mo.findPsuedoInverse(projectedJ);

        // Equation (4).
        const Eigen::VectorXd correction =
            inverse * (desiredVelocity - J * dq);
        dq += correction;

        // Equation (6), using the previous projector.
        P -= inverse * projectedJ;
    }

    double get_dTheta(
        const pinocchio::Data& data,
        pinocchio::FrameIndex frameID) const
    {
        const Eigen::Matrix3d& R = data.oMf[frameID].rotation();
        return -std::atan2(R(1, 0), R(0, 0));
    }

    Eigen::VectorXd calculateVelocity(
        const pinocchio::Model& model,
        pinocchio::Data& data,
        const Eigen::VectorXd& q,
        pinocchio::FrameIndex frameID,
        const Eigen::VectorXd& preferredPosture)
    {
        mo.set_jacobian(model, data, q, frameID);
        reset(model.nv);

        // Task 1: drive yaw toward zero.
        //
        // yaw = atan2(R(1,0), R(0,0)).
        // Convert world angular velocity into yaw rate.
        const Eigen::Vector3d axis =
            data.oMf[frameID].rotation().col(0);

        const double denominator =
            axis.x() * axis.x() + axis.y() * axis.y();

        if (denominator < 1e-8) {
            throw std::runtime_error(
                "Yaw is undefined near this orientation");
        }

        Eigen::RowVector3d yawRateMap;
        yawRateMap <<
            -axis.x() * axis.z() / denominator,
            -axis.y() * axis.z() / denominator,
             1.0;

        const Eigen::MatrixXd J1 =
            yawRateMap * mo.jacobian.bottomRows(3);

        Eigen::VectorXd velocity1(1);
        velocity1[0] = 2.0 * get_dTheta(data, frameID);

        addTask(J1, velocity1);

        // Task 2: move toward a preferred joint posture,
        // without disturbing task 1.
        const Eigen::MatrixXd J2 =
            Eigen::MatrixXd::Identity(model.nv, model.nv);

        const Eigen::VectorXd velocity2 =
            0.2 * pinocchio::difference(
                model, q, preferredPosture);

        addTask(J2, velocity2);

        return dq;
    }
};

int main(int argc, char* argv[]) {
    try {
        const std::string urdfPath =
            argc > 1 ? argv[1] : "panda.urdf";

        const std::string frameName =
            argc > 2 ? argv[2] : "panda_hand";

        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdfPath, model);

        if (!model.existFrame(frameName)) {
            throw std::runtime_error(
                "Unknown frame: " + frameName);
        }

        const auto frameID = model.getFrameId(frameName);
        pinocchio::Data data(model);

        Eigen::VectorXd q = pinocchio::neutral(model);

        const double initial[] = {
            0.0, -0.4, 0.0, -1.8, 0.0, 1.4, 0.7
        };

        for (int i = 0; i < 7; ++i) {
            const std::string jointName =
                "panda_joint" + std::to_string(i + 1);

            if (!model.existJointName(jointName)) {
                throw std::runtime_error(
                    "Missing joint: " + jointName);
            }

            const auto jointID = model.getJointId(jointName);
            q[model.joints[jointID].idx_q()] = initial[i];
        }

        const Eigen::VectorXd preferredPosture = q;
        standard_method controller;

        const double dt = 0.002;

        for (int step = 0; step <= 2500; ++step) {
            const Eigen::VectorXd dq =
                controller.calculateVelocity(
                    model, data, q, frameID,
                    preferredPosture);

            if (!dq.allFinite()) {
                throw std::runtime_error(
                    "Non-finite joint velocity");
            }

            if (step % 500 == 0) {
                std::cout
                    << "t = " << step * dt
                    << " s, yaw error = "
                    << controller.get_dTheta(data, frameID)
                    << " rad, |dq| = " << dq.norm()
                    << '\n';
            }

            if (step < 2500) {
                q = pinocchio::integrate(
                    model, q, Eigen::VectorXd(dt * dq));
            }
        }
    }
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}