#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
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
        for (int i =0; i<7; i++;){
            I[i][i] = 1.0;
        }
        return I;
    }
    Eigen::MatrixXd findPsuedoInverse(Eigen::MatrixXd matrix){
        Eigen::MatrixXd pseudoInverse1;
        pinocchio::pseudoInverse(matrix, pseudoInverse1);
        return pseudoInverse1;
    }
};


class standard_method{
    matrix_operations mo;
    public:
    std::vector calculateProjection(){
        std::vector<std::vector<double>> I = mo.createIdentityMatrix;
        std::vector<7, double> Projection = I - mo.findPsuedoInverse(mo.jacobian*I)*(mo.jacobian*I);
        return Projection
    }
    std::vector calculateDq2(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, int endEffectorFrameID){
        std::vector<double> dq0(7, 0.0);
        std::vector<std::vector<double>> I = mo.createIdentityMatrix;
        mo.set_jacobian(model, data, q,endEffectorFrameID);
        double dTheta = get_dTheta(data);
        
        std::vector<7, double> dQ2 = dq0 + mo.findPsuedoInverse(mo.jacobian*I)*(dTheta-mo.jacobian*dq0);
        return dQ2;
        

    }
    std::vector calculateDq1(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, int endEffectorFrameID){
        std::vector<double> dq0(7, 0.0);
        std::vector<std::vector<double>> I = mo.createIdentityMatrix;
        mo.set_jacobian(model, data, q,endEffectorFrameID);
        double dTheta = get_dTheta(data);
        
        std::vector<7, double> dQ1 = dq0 + mo.findPsuedoInverse(mo.jacobian*I)*(dTheta-mo.jacobian*dq0);
        return dQ1;
        

    }
    double get_dTheta(pinocchio::Data& data){
        const Eigen::Matrix3d& R = data.omf[7].rotation();
        double thetaCurrent = td::atan2(R(1, 0), R(0, 0));
        double dTheta = -thetaCurrent;
        return dTheta;



    }


}




int main(){  
   


    std::string pkg_share = ament_index_cpp::get_package_share_directory("franka_description");
std::string urdf_path = pkg_share + "/urdf/panda.urdf"; 
pinocchio::Model model;
pinocchio::urdf::buildModel(urdf_path, model);


}