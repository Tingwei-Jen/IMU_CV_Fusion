#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <Eigen/Dense>

struct Measurement{

    double timestamp_;
    enum SensorType{
        CV,
        IMU,
        KF
    }sensor_type_;
    Eigen::VectorXd measurement_data;

};
#endif