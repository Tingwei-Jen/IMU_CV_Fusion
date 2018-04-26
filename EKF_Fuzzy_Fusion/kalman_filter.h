#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include <Eigen/Dense>

class KalmanFilter
{
public:
    KalmanFilter();
    ~KalmanFilter();

    void predict();
    void update(const Eigen::VectorXd& z);
    void update_fuzzy(const Eigen::VectorXd& z);

public:
	void set_state(const Eigen::VectorXd& x0){ mX = x0; }
	void set_P_matrix(const Eigen::MatrixXd &P0){ mP = P0; }

	void set_A_matrix(const Eigen::MatrixXd &A){ mA = A; }
	void set_Q_matrix(const Eigen::MatrixXd &Q){ mQ = Q; }
	void set_C_matrix(const Eigen::MatrixXd &C){ mC = C; }
	void set_R_matrix(const Eigen::MatrixXd &R){ mR = R; }

    Eigen::VectorXd getState(){ return mX;}

private:
    void KF(const Eigen::VectorXd& residual);
	void KF_Fuzzy(const Eigen::VectorXd& residual);

    double Fuzzy(double q);                    //membership function

private:
    Eigen::MatrixXd mA, mQ, mP;
    Eigen::MatrixXd mC, mR;
    Eigen::VectorXd mX;
};
#endif //KALMAN_FILTER_H_