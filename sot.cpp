#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
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
    public:
    Eigen::MatrixXd calculateProjection(){
        int n = mo.jacobian.cols();                              // 7 för Panda
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
        Eigen::MatrixXd Projection = I - mo.findPsuedoInverse(mo.jacobian) * mo.jacobian;
        return Projection;
    }
    Eigen::VectorXd calculateDq2(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, int endEffectorFrameID){
        mo.set_jacobian(model, data, q, endEffectorFrameID);
        Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(mo.jacobian.cols());
        Eigen::MatrixXd Jz = mo.jacobian.row(5);                 // rad 5 = vinkelhastighet kring z (wz)
        double dTheta = get_dTheta(data, endEffectorFrameID);

        Eigen::VectorXd dQ2 = dq0 + mo.findPsuedoInverse(Jz) * (dTheta - (Jz * dq0)(0));
        return dQ2;
    }
    Eigen::VectorXd calculateDq1(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, int endEffectorFrameID){
        mo.set_jacobian(model, data, q, endEffectorFrameID);
        Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(mo.jacobian.cols());
        Eigen::MatrixXd Jz = mo.jacobian.row(5);                 // rad 5 = vinkelhastighet kring z (wz)
        double dTheta = get_dTheta(data, endEffectorFrameID);

        Eigen::VectorXd dQ1 = dq0 + mo.findPsuedoInverse(Jz) * (dTheta - (Jz * dq0)(0));
        return dQ1;
    }
    double get_dTheta(pinocchio::Data& data, int frameID){
        const Eigen::Matrix3d R = data.oMf[frameID].rotation();
        double thetaCurrent = std::atan2(R(1, 0), R(0, 0));
        double dTheta = -thetaCurrent;
        return dTheta;
    }
};




int main(){  
   


std::string urdf_path = "/sot/models/panda.urdf";
pinocchio::Model model;
pinocchio::urdf::buildModel(urdf_path, model);

std::cout << "Model loaded, nq = " << model.nq << std::endl;


}