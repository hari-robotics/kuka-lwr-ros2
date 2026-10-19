//	Moore-Penrose pseudo-inverse matrix with Singular Value Decomposition by Eigen Library
//	NOTE: input matrix should be const type

//  #include "stdafx.h"
#ifndef PSEUDO_INVERSE_H
#define PSEUDO_INVERSE_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
using namespace Eigen;

inline void PseudoInverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_,bool damped = true)
{	
	double lambda_ = damped?0.2:0.0;

	JacobiSVD<MatrixXd> svd(M_, ComputeFullU | ComputeFullV);
	JacobiSVD<MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
	MatrixXd S_ = M_;	// copying the dimensions of M_, its content is not needed.
	S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

    M_pinv_ = MatrixXd(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
}

#endif

/*
#include<iostream> 
#include<Eigen/Core> 
#include<Eigen/SVD>   

// method for calculating the pseudo-Inverse as recommended by Eigen developers
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    //  For a square matrix
	//  Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
    // For a non-square matrix
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}
*/
