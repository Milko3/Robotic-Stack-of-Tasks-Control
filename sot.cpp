#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <vector>
#include <cmath>






class matrix_operations{

    public:
    Eigen::MatrixXd jacobian;
    void set_jacobian(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, int endEffectorFrameID){
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    
    jacobian.resize(6, model.nv);
    jacobian.setZero();
    
    pinocchio::getFrameJacobian(model, data, endEffectorFrameID, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
}
    std::vector<std::vector<double>> createIdentityMatrix(int size){
        std::vector<std::vector<double>> I(size, std::vector<double>(size, 0.0));
        for (int i =0; i<7; i++){
            I[i][i] = 1.0;
        }
        return I;
    }
    Eigen::MatrixXd findPsuedoInverse(const Eigen::MatrixXd& matrix){
        return matrix.completeOrthogonalDecomposition().pseudoInverse();
    }
};


class standard_method{
    matrix_operations mo;
    Eigen::VectorXd dQ1;
    Eigen::VectorXd dQ2;
public:
    Eigen::MatrixXd calculateProjection(const Eigen::MatrixXd& J){
        int n = J.cols();                           
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
        Eigen::MatrixXd Projection = I - mo.findPsuedoInverse(J) * J;
        return Projection;
    }
    
    Eigen::VectorXd calculateDq1(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, int endEffectorFrameID){
        mo.set_jacobian(model, data, q, endEffectorFrameID);
        Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(mo.jacobian.cols());
        // Task 1: keep the plate level (tilt about world x and y -> 0).
        Eigen::MatrixXd Jlevel = mo.jacobian.middleRows(3, 2);   // rows 3, 4 = rotation about x, y
        Eigen::Vector2d tilt = get_tilt_error(data, endEffectorFrameID);

        const double K1 = 1.0;   // gain for task 1 (plate level)
        dQ1 = dq0 + mo.findPsuedoInverse(Jlevel) * (K1 * tilt - Jlevel * dq0);
        return dQ1;
    }
    
    Eigen::VectorXd calculateDq2(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, int endEffectorFrameID, const Eigen::Vector3d& goalXYZ, const Eigen::VectorXd& dq1){
        mo.set_jacobian(model, data, q, endEffectorFrameID);
        Eigen::MatrixXd Jxyz = mo.jacobian.block(0, 0, 3, mo.jacobian.cols()); 
        Eigen::Vector3d dxyz = get_d_xyz(data, endEffectorFrameID, goalXYZ);    
        Eigen::MatrixXd Jlevel = mo.jacobian.middleRows(3, 2);   // task 1's Jacobian (plate level)

        Eigen::MatrixXd P1 = calculateProjection(Jlevel);
        Eigen::MatrixXd Jxyz_proj = Jxyz * P1;
        
        const double K2 = 1.0;   // gain for task 2 (position)
        dQ2 = dq1 + mo.findPsuedoInverse(Jxyz_proj) * (K2 * dxyz - (Jxyz * dq1));
        return dQ2;
    }
    
    double get_dTheta(pinocchio::Data& data, int frameID){
        const Eigen::Matrix3d R = data.oMf[frameID].rotation();
        double thetaCurrent = std::atan2(R(1, 0), R(0, 0));
        double dTheta = -thetaCurrent;
        return dTheta;
    }
    
    // How much the plate is tilted, as the rotation (about world x and y) that would level it.
    // z = the flange's z axis (plate normal), target = straight down (0, 0, -1).
    Eigen::Vector2d get_tilt_error(pinocchio::Data& data, int frameID){
        Eigen::Vector3d z = data.oMf[frameID].rotation().col(2);
        Eigen::Vector3d target(0.0, 0.0, -1.0);
        Eigen::Vector3d error = z.cross(target);
        return error.head<2>();
    }

    // Tilt angle of the plate in degrees (0 = perfectly level).
    double get_tilt_degrees(pinocchio::Data& data, int frameID){
        Eigen::Vector3d z = data.oMf[frameID].rotation().col(2);
        double c = std::max(-1.0, std::min(1.0, -z.z()));
        return std::acos(c) * 180.0 / M_PI;
    }

    Eigen::Vector3d get_d_xyz(pinocchio::Data& data, int frameId, const Eigen::Vector3d& goalXYZ){ // Ändrat till Vector3d
        Eigen::Vector3d xyz = data.oMf[frameId].translation();
        Eigen::Vector3d d_xyz = goalXYZ - xyz;
        return d_xyz;
    }
};




// main() is left out when the ROS 2 controller (ros2/sot_ros) includes this file.
#ifndef SOT_NO_MAIN
int main(){

    // ---------------------------------------------------------------- setup
    std::string urdf_path = "/sot/models/panda.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);
    std::cout << "Model loaded, nq = " << model.nq << std::endl;

    const int endEffectorFrameID = model.getFrameId("panda_link7");

    // ---------------------------------------------------------------- test case
    // Waiter: the hand carries a plate and must not spill.
    // Start pose: the plate (flange) is level.
    // Task 1 (priority 1): keep the plate level (parallel to the floor).
    // Task 2 (priority 2): hand moves 20 cm forward (x), 10 cm sideways (y) and 20 cm down (z).
    // Pass if after T seconds both errors are small.
    Eigen::VectorXd q(7);
    q << 0.0, 0.2, 0.0, -1.6, 0.0, 1.8, 0.7;

    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    const Eigen::Vector3d startXYZ = data.oMf[endEffectorFrameID].translation();
    //const Eigen::Vector3d goalXYZ = startXYZ + Eigen::Vector3d(-0.624, 0.624, 0.2); // scenario long reach
    //const Eigen::Vector3d goalXYZ = startXYZ + Eigen::Vector3d(-0.15, 0.2, 0.1); // scenario short move
    const Eigen::Vector3d goalXYZ = startXYZ + Eigen::Vector3d(-0.35, -0.45, -0.35); // scenario low reach

    standard_method sot;
    const double dt = 0.002;   // time step [s]
    const double T = 12.0;      // test length [s]
    const int steps = static_cast<int>(T / dt);

    std::cout << "Test: plate level (priority 1), turn 90 degrees around the base and 20 cm up (priority 2)" << std::endl;
    for (int step = 0; step <= steps; ++step) {
        Eigen::VectorXd dq1 = sot.calculateDq1(model, data, q, endEffectorFrameID);
        Eigen::VectorXd dq2 = sot.calculateDq2(model, data, q, endEffectorFrameID, goalXYZ, dq1);

        if (step % 500 == 0) {   // print every 1 s
            double tilt = sot.get_tilt_degrees(data, endEffectorFrameID);
            double posError = sot.get_d_xyz(data, endEffectorFrameID, goalXYZ).norm();
            std::cout << "t = " << step * dt << " s"
                      << "   plate tilt = " << tilt << " deg"
                      << "   position error = " << posError << " m" << std::endl;
        }
        const double maxJointSpeed = 2.0;   // [rad/s]
        const double fastest = dq2.cwiseAbs().maxCoeff();
        if (fastest > maxJointSpeed) {
            dq2 *= maxJointSpeed / fastest;
        }

        if (step < steps) {
            q += dq2 * dt;        // move the joints for one time step
        }
    }

    // ---------------------------------------------------------------- result
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    double finalTilt = sot.get_tilt_degrees(data, endEffectorFrameID);
    double finalPos = sot.get_d_xyz(data, endEffectorFrameID, goalXYZ).norm();
    bool pass = finalTilt < 0.1 && finalPos < 1e-2;   // 0.1 degree and 1 cm
    std::cout << (pass ? "PASS" : "FAIL")
              << ": final plate tilt = " << finalTilt << " deg"
              << ", final position error = " << finalPos << " m" << std::endl;
    return pass ? 0 : 1;
}
#endif  // SOT_NO_MAIN
