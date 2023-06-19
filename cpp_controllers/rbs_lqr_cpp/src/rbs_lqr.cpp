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

#include "rbs_lqr_cpp/rbs_lqr.hpp"

rbs_lqr::rbs_lqr(     const Eigen::MatrixXd& A_,
                        const Eigen::MatrixXd& B_,
                        const Eigen::MatrixXd& Q_,
                        const Eigen::MatrixXd& R_):
    A{A_},
    B{B_},
    Q{Q_},
    R{R_}
{
    check_matrices(A,B,Q,R);
    return;
}

rbs_lqr::rbs_lqr(int n, int m):
A{n,n}, B{n,m}, Q{n,n}, R{m,m}
{
    check_matrices(A,B,Q,R);
    return;
}

bool rbs_lqr::check_matrices(const Eigen::MatrixXd& A_,
                              const Eigen::MatrixXd& B_,
                              const Eigen::MatrixXd& Q_,
                              const Eigen::MatrixXd& R_)
{
/*
    This function checks that the dimensions
    of the matrices are coherent with the 
    operations to be performed...

    @TODO: check that this is the condition to verify that the calculation can be done. 
*/

    return A_.cols() == Q_.cols() && B_.cols() == R_.cols(); 
}

void rbs_lqr::update_matrices(const Eigen::MatrixXd& A_,
                               const Eigen::MatrixXd& B_,
                               const Eigen::MatrixXd& Q_,
                               const Eigen::MatrixXd& R_)
{
    if(check_matrices(A_, B_,Q_,R_))
    {
        A = A_;
        B = B_;
        Q = Q_;
        R = R_;
    }
}

Eigen::MatrixXd  rbs_lqr::dlqr(const Eigen::MatrixXd& A_,
                                const Eigen::MatrixXd& B_,
                                const Eigen::MatrixXd& Q_,
                                const Eigen::MatrixXd& R_,
                                int n_it)
{
/*
    In this method, the iterative calculation
    necessary to solve the LQR problem is carried out.
*/

    // Check matrices
    if(check_matrices(A_,B_,Q_,R_))
    {
        // Update matrices
        update_matrices(A_,B_,Q_,R_);
        // Calculation
        // 1. Initial values for P and K
        Eigen::MatrixXd P_inf(int(Q.rows()), int(Q.cols()));
        Eigen::MatrixXd K_inf(int(R.rows()), int(R.cols())); 
        
        P_inf.setIdentity();
        K_inf.setIdentity();

        for(int i=0; i<n_it; i++)
        {
            K_inf = -(B.transpose()*P_inf*B + R).inverse()*B.transpose()*P_inf*A;
            P_inf = A.transpose()*P_inf*A + A.transpose()*P_inf*B*K_inf + Q;
        }
        
        return K_inf;
    }
    else
    {
        Eigen::MatrixXd no_valid_matrices;
        no_valid_matrices << 0;
        return no_valid_matrices;
    }
}

Eigen::MatrixXd  rbs_lqr::dlqr(int n_it)
{
/*
    In this method, the iterative calculation
    necessary to solve the LQR problem is carried out.
*/

    // Check matrices
    if(check_matrices(A,B,Q,R))
    {
        // Calculation
        // 1. Initial values for P and K
        Eigen::MatrixXd P_inf(int(Q.rows()), int(Q.cols()));
        Eigen::MatrixXd K_inf(int(R.rows()), int(R.cols())); 

        P_inf.setIdentity();
        K_inf.setIdentity();

        for(int i=0; i<n_it; i++)
        {
            K_inf = -(B.transpose()*P_inf*B + R).inverse()*B.transpose()*P_inf*A;
            P_inf = A.transpose()*P_inf*A + A.transpose()*P_inf*B*K_inf;
        }

        return K_inf;
    }
    else
    {
        Eigen::MatrixXd no_valid_matrices;
        no_valid_matrices << 0;
        return no_valid_matrices;
    }
}