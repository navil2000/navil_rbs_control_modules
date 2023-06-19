/*
* Author:   Navil Abdeselam Abdel-lah
* Date:     19/06/2023
* Org:      Robesafe
*
* @brief Generic library for LQR controllers. 
* The Rbs2_lqr class must be provided with the 
* corresponding matrices for the calculation. 
* This implementation makes use of the Eigen 
* library, specially designed for matrix 
* computation in C++.
*
*/

#pragma once

#include <eigen3/Eigen/Dense>

class rbs_lqr
{
    public:
        // Constructor matrix by matrix 
        rbs_lqr(const Eigen::MatrixXd& A_,
                 const Eigen::MatrixXd& B_,
                 const Eigen::MatrixXd& Q_,
                 const Eigen::MatrixXd& R_
        );
        // Constructor for state space variable systems with A as nxn matrix
        rbs_lqr(int n, int m);
        // Iterative computation to obtain the solution to the LQR problem
        // 1st -> New matrices
        Eigen::MatrixXd dlqr( const Eigen::MatrixXd& A_,
                              const Eigen::MatrixXd& B_,
                              const Eigen::MatrixXd& Q_,
                              const Eigen::MatrixXd& R_,
                              int n_it = 30
        );
        // 2nd -> No actualization of matrices
        Eigen::MatrixXd dlqr(int n_it = 30);
        // Update matrices
        void update_matrices(const Eigen::MatrixXd& A_,
                             const Eigen::MatrixXd& B_,
                             const Eigen::MatrixXd& Q_,
                             const Eigen::MatrixXd& R_
        );
    private:
        // Check matrices type and dimensions
        bool check_matrices(const Eigen::MatrixXd& A_,
                            const Eigen::MatrixXd& B_,
                            const Eigen::MatrixXd& Q_,
                            const Eigen::MatrixXd& R_
        );
        // Matrices
        Eigen::MatrixXd     A;
        Eigen::MatrixXd     B;
        Eigen::MatrixXd     Q;
        Eigen::MatrixXd     R;
};