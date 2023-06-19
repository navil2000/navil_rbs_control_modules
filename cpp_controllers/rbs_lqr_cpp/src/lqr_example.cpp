/**
* Author:   Navil Abdeselam Abdel-lah
* Date:     19/06/2023
* Org:      Robesafe
*
* Simple example of how to use the rbs_lqr class.
* In this example, a simple system is simulated in order
* to show the use of the lqr.
*
*/

#include <iostream>
#include "rbs_lqr_cpp/rbs_lqr.hpp"
#include <matplot/matplot.h>
#include <eigen3/Eigen/Dense>
using namespace matplot;

inline void ezPrint(std::string msg)
{
    std::cout << msg << std::endl;
}

int main()
{
    ezPrint("\n - Starting LQR example - \n");
    
    // Plotting variables
    std::vector<double> y_plot;
    std::vector<double> t_plot;

    // Simulation parameters
    int nsim = 500;
    double dt = 0.1;

    // Vectors involved in the simulation
    ezPrint(" - Creating vectors - ");
    Eigen::VectorXd     x(2);
    Eigen::VectorXd     x_dot(2);
    Eigen::VectorXd     u(1);
    Eigen::VectorXd     y(1);
    Eigen::MatrixXd     K_lqr(1,2);

    u << 1;

    x << 0,
         0;

    x_dot << 20,
             3;

    y << 0;

    // LQR matrices
    ezPrint(" - Creating matrices - ");
    Eigen::MatrixXd     A(2,2);
    Eigen::MatrixXd     B(2,1);
    Eigen::MatrixXd     C(1,2);
    Eigen::MatrixXd     Q(2,2);
    Eigen::MatrixXd     R(1,1);

    A << 1, dt,
         0, 1;
    
    B << 0.5*dt*dt,
         dt;

    C << 1, 0;

    Q << 1000, 0,
         0, 1000;

    R << 0.1;


    // Initial state and control vector
    // Create the lqr object
    rbs_lqr lqr(A,B,Q,R);

    ezPrint(" - Calculating LQR Gain - ");
    K_lqr = lqr.dlqr();
    std::cout << "Gain value: " << K_lqr << std::endl;
    
    // Simulation
    ezPrint(" - Starting simulation - ");
    for(int i=0;i<nsim;++i)
    {
        // Dynamic of the system
        x = A*x_dot + B*K_lqr*x_dot;
        y = C*x;
        x_dot = x;

        // Store the values for plotting
        y_plot.push_back(y(0));
        t_plot.push_back(i*dt);
    }
    ezPrint(" - Simulation finished - ");

    // Plotting
    ezPrint(" - Plotting results - ");
    auto fig = figure(true);
    plot(t_plot,y_plot);
    show();
    save("img/rbs_lqr_cpp_test.png");
    
    return 0;
}