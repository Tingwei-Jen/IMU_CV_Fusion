#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{

}

KalmanFilter::~KalmanFilter()
{

}


void KalmanFilter::predict()
{
	mX = mA * mX;                               //X(k|k-1)
	mP = mA * mP * mA.transpose() + mQ;         //P(k|k-1)
}

void KalmanFilter::update(const Eigen::VectorXd& z)
{
	Eigen::VectorXd residual = z - mC * mX;
	KF(residual);
}

void KalmanFilter::update_fuzzy(const Eigen::VectorXd& z)
{
	Eigen::VectorXd residual = z - mC * mX;
	KF_Fuzzy(residual);
}


/*private*/
void KalmanFilter::KF(const Eigen::VectorXd& residual)
{
	Eigen::MatrixXd S = mC * mP * mC.transpose() + mR;
	Eigen::MatrixXd K = mP * mC.transpose() * S.inverse();

	mX = mX + ( K * residual );
  	int size = mX.size();
  	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
  	mP = (I - K * mC) * mP;
}

void KalmanFilter::KF_Fuzzy(const Eigen::VectorXd& residual)
{
	Eigen::MatrixXd S = mC * mP * mC.transpose() + mR;
	double q = residual.transpose() * S.inverse() * residual;
	double beta_ob = Fuzzy(q);     //    0<beta_ob<1
	double beta_pred = 1.0 - beta_ob;

	Eigen::MatrixXd P_ob_inv = mP.inverse() + mC.transpose() * mR.inverse() * mC;
	Eigen::MatrixXd K = P_ob_inv.inverse() * mC.transpose() * mR.inverse();

	Eigen::VectorXd X_ob = mX + K * residual;

	//update state
	mX = beta_pred * mX + beta_ob * X_ob;   

	//update P matrix
	Eigen::VectorXd X_error = mX - X_ob;
	mP = beta_pred * mP + beta_ob * ( P_ob_inv.inverse() + X_error*X_error.transpose() );

}

double KalmanFilter::Fuzzy(double q)
{
	//y = -0.196x + 2.225    6.25 <x< 11.35

	double mu;
	if ( q < 6.25 )
		mu = 1.0;
	else if ( q > 11.35 )
		mu = 0.0;
	else
		mu = -0.196 * q + 2.225;

	return mu;
}

