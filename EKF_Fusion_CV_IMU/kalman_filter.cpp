#include "kalman_filter.h"

KalmanFilter::KalmanFilter(){}

KalmanFilter::KalmanFilter(
		const Eigen::MatrixXd &A,
		const Eigen::MatrixXd &C,
		const Eigen::MatrixXd &Q,
		const Eigen::MatrixXd &R,
		const Eigen::MatrixXd &P
		)
:mA(A), mC(C), mQ(Q), mR(R), mP(P)
{
}

void KalmanFilter::predict()
{
	mX = mA * mX;
	mP = mA * mP * mA.transpose() + mQ;
}

void KalmanFilter::update(const Eigen::VectorXd& z)
{
	Eigen::VectorXd residual = z - mC * mX;
	KF(residual);
}

/*
private
*/
void KalmanFilter::KF(const Eigen::VectorXd& residual)
{
	Eigen::MatrixXd S = mC * mP * mC.transpose() + mR;
	mK = mP * mC.transpose() * S.inverse();

	mX = mX + (mK * residual);
  	int size = mX.size();
  	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
  	mP = (I - mK * mC) * mP;
}