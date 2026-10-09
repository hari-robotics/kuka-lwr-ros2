// 5th order polynomial interpolator to smooth the reference trajectory

#ifndef SMOOTH_TRAJ_H
#define SMOOTH_TRAJ_H

#include <cmath>

using namespace std;

double SmoothTraj(double Xi, double Xf, double T, int SubSteps, int i){
	double t0 = 0;
	double H = (Xf - Xi);
	int A[] = { 0, 0, 0, 10, -15, 6 };
	double t;
	double Rho;
	double Sigma;
	double Xt;
	
	t = (double) i/SubSteps;
	Rho = (t - t0)/T;
	Sigma = A[0] + A[1]*Rho + A[2]*pow(Rho, 2) + A[3]*pow(Rho, 3) + A[4]*pow(Rho, 4) + A[5]*pow(Rho, 5);
	Xt = Xi + H*Sigma;

//	RCLCPP_INFO(get_logger(), "t: %f", t);
	
	return Xt;
}

#endif
