#ifndef FUSION_H_
#define FUSION_H_

#include "kalman_filter.h"
#include "measurement.h"
#include <Eigen/Dense>

class Fusion{
public:
    Fusion();
    ~Fusion();

    void ProcessMeasurement(const Measurement &measurement);  //imu in update
    void ProcessMeasurement_imu(const Measurement &measurement); //imu in predict
    Eigen::VectorXd getState(){ return ekf->getState();}

private:
    KalmanFilter *ekf;
    bool initialized;
    double previous_timestamp_;

    Eigen::MatrixXd R_cv;  //cv
    Eigen::MatrixXd C_cv;  //cv
    Eigen::MatrixXd R_imu;  //imu
    Eigen::MatrixXd C_imu;  //imu
};
#endif