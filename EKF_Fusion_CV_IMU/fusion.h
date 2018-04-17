#ifndef FUSION_H_
#define FUSION_H_

#include "kalman_filter.h"
#include "measurement.h"
#include <Eigen/Dense>

class Fusion{
public:
    Fusion();
    ~Fusion();

    void ProcessMeasurement(const Measurement &measurement);
    Eigen::VectorXd getState(){ return ekf->getState();}
    Eigen::MatrixXd getPmatrix(){ return ekf->getPmatrix();}
    Eigen::MatrixXd getKmatrix(){ return ekf->getKmatrix();}

private:
    KalmanFilter *ekf;
    bool initialized;
    double previous_timestamp_;

    Eigen::MatrixXd R_CV_;
    Eigen::MatrixXd R_IMU_;
    Eigen::MatrixXd C_CV_;
    Eigen::MatrixXd C_IMU_;
};
#endif