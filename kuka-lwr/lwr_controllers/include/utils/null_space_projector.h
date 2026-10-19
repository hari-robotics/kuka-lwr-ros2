//	Computation of the Null Space Projector matrix N

#ifndef NULL_SPACE_PROJECTOR_H
#define NULL_SPACE_PROJECTOR_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <utils/pseudo_inverse.h>
#include <utils/matrix_multiplication.h>

using namespace std;
using namespace Eigen;

 Eigen::MatrixXd NullSpaceProjector(const Eigen::MatrixXd &J_, const Eigen::MatrixXd &J_transpose, const Eigen::MatrixXd &M_pinv){
 
	Eigen::MatrixXd J_M_pinv =  Eigen::MatrixXd(6,7);
	Eigen::MatrixXd J_M_pinv_J_transpose =  Eigen::MatrixXd(6,6);
	Eigen::MatrixXd J_M_pinv_J_transpose_pinv =  Eigen::MatrixXd(6,6);
	
	Eigen::MatrixXd M_pinv_J_transpose =  Eigen::MatrixXd(7,6);
	
	Eigen::MatrixXd J_plus =  Eigen::MatrixXd(7,6);
	Eigen::MatrixXd J_plus_transpose =  Eigen::MatrixXd(6,7);

	Eigen::MatrixXd J_transpose_J_plus_transpose =  Eigen::MatrixXd(7,7);

	MatrixXd I = MatrixXd::Identity(7, 7);

	Eigen::MatrixXd N =  Eigen::MatrixXd(7,7);

//	J_M_pinv (6,7)*(7,7)=(6,7)
	J_M_pinv = MatrixMultiplication(J_, M_pinv, 6, 7, 7);

//	J_M_pinv_J_transpose (6,7)*(7,6)=(6,6)
	J_M_pinv_J_transpose = MatrixMultiplication(J_M_pinv, J_transpose, 6, 7, 6);

//	J_M_pinv_J_transpose_pinv inv(6,6)=(6,6)
	PseudoInverse(J_M_pinv_J_transpose, J_M_pinv_J_transpose_pinv, true);
    	//PseudoInverse(J_M_pinv_J_transpose, J_M_pinv_J_transpose_pinv);

//	M_pinv_J_transpose (7,7)*(7,6)=(7,6)
	M_pinv_J_transpose = MatrixMultiplication(M_pinv, J_transpose, 7, 7, 6);

//	J_plus (7,6)*(6,6)=(7,6)
	J_plus = MatrixMultiplication(M_pinv_J_transpose, J_M_pinv_J_transpose_pinv, 7, 6, 6);

//	J_plus_transpose transp(7,6)=(6,7)
	J_plus_transpose = J_plus.transpose();

//	J_transpose_J_plus_transpose (7,6)*(6,7)=(7,7)
	J_transpose_J_plus_transpose = MatrixMultiplication(J_transpose, J_plus_transpose, 7, 6, 7);

//	N (7,7)-(7,7)=(7,7)
	N = I - J_transpose_J_plus_transpose;

/////// Alternative N_ = J# * J;
/*	Eigen::MatrixXd J_J_plus =  Eigen::MatrixXd(7,7);

	J_J_plus = MatrixMultiplication(J_plus, J_, 7, 6, 7);	

	N = I - J_J_plus;*/
/////// Alternative N_

//		PRINT Null Space Projector MATRIX
/*				
		for (int k=0; k<7; k++){
			for (int l=0; l<7; l++){
				cout << N.coeff(k,l) << "|";
			}
			cout<<"\n";
		}
*/		

	return N;
}

#endif
