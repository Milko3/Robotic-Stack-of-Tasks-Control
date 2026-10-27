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
    Eigen::VectorXd dQ1;
    Eigen::VectorXd dQ2;
public:
    Eigen::MatrixXd calculateProjection(){
        int n = mo.jacobian.cols();                           
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
        Eigen::MatrixXd Projection = I - mo.findPsuedoInverse(mo.jacobian) * mo.jacobian;
        return Projection;
    }
    
    Eigen::VectorXd calculateDq1(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, int endEffectorFrameID){
        mo.set_jacobian(model, data, q, endEffectorFrameID);
        Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(mo.jacobian.cols());
        Eigen::MatrixXd Jz = mo.jacobian.row(3);              
        double dTheta = get_dTheta(data, endEffectorFrameID);

        dQ1 = dq0 + mo.findPsuedoInverse(Jz) * (dTheta - (Jz * dq0)(0));
        return dQ1; 
    }
    
    Eigen::VectorXd calculateDq2(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, int endEffectorFrameID, const Eigen::Vector3d& goalXYZ, const Eigen::VectorXd& dq1){
        mo.set_jacobian(model, data, q, endEffectorFrameID);
        Eigen::MatrixXd Jxyz = mo.jacobian.block(0, 0, 3, mo.jacobian.cols()); 
        Eigen::Vector3d dxyz = get_d_xyz(data, endEffectorFrameID, goalXYZ);    

        
        Eigen::MatrixXd P1 = calculateProjection();
        Eigen::MatrixXd Jxyz_proj = Jxyz * P1;
        
        dQ2 = dq1 + mo.findPsuedoInverse(Jxyz_proj) * (dxyz - (Jxyz * dq1));
        return dQ2;
    }
    
    double get_dTheta(pinocchio::Data& data, int frameID){
        const Eigen::Matrix3d R = data.oMf[frameID].rotation();
        double thetaCurrent = std::atan2(R(1, 0), R(0, 0));
        double dTheta = -thetaCurrent;
        return dTheta;
    }
    
    Eigen::Vector3d get_d_xyz(pinocchio::Data& data, int frameId, const Eigen::Vector3d& goalXYZ){ // Ändrat till Vector3d
        Eigen::Vector3d xyz = data.oMf[frameId].translation();
        Eigen::Vector3d d_xyz = goalXYZ - xyz;
        return d_xyz;
    }
};




int main(){  
   


std::string urdf_path = "/sot/models/panda.urdf";
pinocchio::Model model;
pinocchio::urdf::buildModel(urdf_path, model);

std::cout << "Model loaded, nq = " << model.nq << std::endl;


}
