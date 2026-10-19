//	Multiplication between Matrix M_a(r1, c1) and M_b(r2, c2)
#ifndef MATRIX_MULTIPLICATION_H
#define MATRIX_MULTIPLICATION_H

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

Eigen::MatrixXd MatrixMultiplication(const Eigen::MatrixXd &M_a, const Eigen::MatrixXd &M_b, const int &r1, const int &c1, const int &c2){
	
	Eigen::MatrixXd M_p =  Eigen::MatrixXd(r1,c2);	//	product result matrix
/*
//	this part of code is wrong 9.9.2022
	for(int k=0; k<r1; k++){
		for(int l=0; l<c2; l++){
			M_p(k,l) = 0.0;
		}
	}

	for (int k=0; k<r1; k++){
		for (int l=0; l<c2; l++){
			for (int h=0; h<c1; h++){
				M_p(k,l) += M_a.coeff(k,l) * M_b.coeff(h,l);
			}
		}
	}
*/
	for(int i=0; i<r1; i++){
		for(int j=0; j<c2; j++){
			M_p(i,j) = 0.0;
		}
	}

	for (int i=0; i<r1; i++){
		for (int j=0; j<c2; j++){
			for (int k=0; k<c1; k++){
				M_p(i,j) += M_a.coeff(i,k) * M_b.coeff(k,j);
			}
		}
	}


	return M_p;
}

#endif