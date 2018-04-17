#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include <Eigen/Dense>

class KalmanFilter
{
public:
	KalmanFilter();
	KalmanFilter(
		const Eigen::MatrixXd &A,
		const Eigen::MatrixXd &C,
		const Eigen::MatrixXd &Q,
		const Eigen::MatrixXd &R,
		const Eigen::MatrixXd &P
		);
	
	void init_state(const Eigen::VectorXd& x0){ mX = x0; }
	void init_P_matrix(const Eigen::MatrixXd &P0){ mP = P0; }

	void set_A_matrix(const Eigen::MatrixXd &A){ mA = A; }
	void set_C_matrix(const Eigen::MatrixXd &C){ mC = C; }
	void set_Q_matrix(const Eigen::MatrixXd &Q){ mQ = Q; }
	void set_R_matrix(const Eigen::MatrixXd &R){ mR = R; }


	void predict();
	void update(const Eigen::VectorXd& z);

	Eigen::VectorXd getState(){ return mX;}
	Eigen::MatrixXd getPmatrix(){ return mP;}
	Eigen::MatrixXd getKmatrix(){ return mK;}
	
private:
	void KF(const Eigen::VectorXd& residual);

	Eigen::MatrixXd mA, mC, mQ, mR, mP, mK;
	Eigen::VectorXd mX;  //mX: x, y, vx, vy
};

#endif