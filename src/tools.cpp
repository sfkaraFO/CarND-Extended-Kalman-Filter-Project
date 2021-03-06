#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
   /**
   * TODO: Calculate the RMSE here.
   */

   Eigen::VectorXd RMSE_Error(4);
   RMSE_Error << -1,-1,-1,-1;
   // Check the validity of the following conditions respectively:
   //  - estimations and ground_truth vector sizes shouldn't be zero 
   //  - estimations and ground_truth vector sizes should be equal 
   //  - estimation and ground_truth matrices(also vector) sizes shouldn't be zero 
   //  - estimation and ground_truth matrices(also vector) sizes should be equal 
   if (estimations.empty() || ground_truth.empty()){
      std::cerr << "RMSE calculation error: vector is empty!" << std::endl;
      return RMSE_Error;
   }
   else if (estimations.size() != ground_truth.size()){
      std::cerr << "RMSE calculation error: vector sizes are inconsistent!" << std::endl;
      return RMSE_Error;
   }
   else if (estimations[0].size()==0 || estimations[0].size()==0){
      std::cerr << "RMSE calculation error: matrix size is zero!" << std::endl;
      return RMSE_Error;
   }
   else if (estimations[0].size() != estimations[0].size()){
      std::cerr << "RMSE calculation error: matrix sizes are inconsistent!" << std::endl;
      return RMSE_Error;   
   }
   // All is well
   else{
      uint32_t dim = estimations[0].size();
      Eigen::VectorXd RMSE = VectorXd::Zero(dim);
      VectorXd residual = VectorXd::Zero(dim);
      // Integrate the square of resudials
      for (uint32_t i = 0; i < estimations.size(); ++i){
         residual = estimations[i] - ground_truth[i];
         residual = residual.array().pow(2);
         RMSE = RMSE + residual;
      }
      // Calculate the mean
      RMSE = RMSE / estimations.size();

      // Calculate the root
      RMSE = RMSE.array().sqrt();

      return RMSE;
   }
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   /**
   * TODO:
   * Calculate a Jacobian here.
   */
   
   MatrixXd Hj(3,4);
   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // pre-compute a set of terms to avoid repeated calculation
   float c1 = px*px+py*py;
   float c2 = sqrt(c1);
   float c3 = (c1*c2);

   // check division by zero
   if (fabs(c1) < 0.0001) {
      std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
      return Hj;
   }

   // compute the Jacobian matrix
   Hj << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
         py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

   return Hj;   

}
