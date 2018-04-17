#include "fusion.h"
#include <iostream>

Fusion::Fusion()
{
    initialized = false;
    ekf = new KalmanFilter();

    //state: [px, py, pz, vx, vy , vz, ax, ay, az]

    R_CV_ = Eigen::MatrixXd(3,3);
    R_IMU_ = Eigen::MatrixXd(3,3);
    C_CV_ = Eigen::MatrixXd(3,9);
    C_IMU_ = Eigen::MatrixXd(3,9);

    R_CV_<<0.0001, 0, 0,
           0, 0.0001, 0,
           0, 0, 0.0001;

    R_IMU_<<0.00097, 0, 0,
            0, 0.00221, 0,
            0, 0, 0.001175;
    
    C_CV_<<1, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0, 0, 0;

    C_IMU_<<0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1;

}
Fusion::~Fusion()
{
    delete ekf;
}

void Fusion::ProcessMeasurement(const Measurement &measurement)
{ 
    if(!initialized)
    {
        if(measurement.sensor_type_ == Measurement::CV)
        {
            float px = measurement.measurement_data(0);
            float py = measurement.measurement_data(1);
            float pz = measurement.measurement_data(2);

            if(abs(px) < 0.00001 && abs(py) < 0.00001 && abs(pz) < 0.00001)
            {
                px = 0.00001;
                py = 0.00001;
                pz = 0.00001;
            }

            Eigen::VectorXd x0(9);
            x0<<px, py, pz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            ekf->init_state(x0);
        } 
        else if (measurement.sensor_type_ == Measurement::IMU) 
        {
            float accx = measurement.measurement_data(0);
            float accy = measurement.measurement_data(1);
            float accz = measurement.measurement_data(2);

            if(abs(accx) < 0.00001 && abs(accy) < 0.00001 && abs(accz) < 0.00001)
            {
                accx = 0.00001;
                accy = 0.00001;
                accz = 0.00001;
            }

            Eigen::VectorXd x0(9);
            x0<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, accx, accy, accz;
            ekf->init_state(x0);
        }

        Eigen::MatrixXd P0(9,9);
        P0<<0.05, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0.05, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0.05, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0.05, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0.05, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.05, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0.05, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0.05, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0.05;


        ekf->init_P_matrix(P0);
        std::cout<<"Kalman Filter Init !!"<<std::endl;
        previous_timestamp_ = measurement.timestamp_;

        initialized = true;
        return;
    }
    else
    {
        //prediction
        double dt = measurement.timestamp_ - previous_timestamp_;
        previous_timestamp_ = measurement.timestamp_;
        Eigen::MatrixXd A_(9,9);
        A_<<1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0,
            0, 1, 0, 0, dt, 0, 0, dt*dt/2, 0,
            0, 0, 1, 0, 0, dt, 0, 0, dt*dt/2,
            0, 0, 0, 1, 0, 0, dt, 0, 0,
            0, 0, 0, 0, 1, 0, 0, dt, 0,
            0, 0, 0, 0, 0, 1, 0, 0, dt,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1;

        ekf->set_A_matrix(A_);

        Eigen::MatrixXd Q_(9,9);
        Q_<<0.01, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0.01, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0.01, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0.01, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0.01, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0.01;
        
        ekf->set_Q_matrix(Q_);
        ekf->predict();

        //update

        if(measurement.sensor_type_ == Measurement::CV)
        {
            ekf->set_C_matrix(C_CV_);
            ekf->set_R_matrix(R_CV_);
            ekf->update(measurement.measurement_data);
        }
        else if (measurement.sensor_type_ == Measurement::IMU)
        {
            ekf->set_C_matrix(C_IMU_);
            ekf->set_R_matrix(R_IMU_);
            ekf->update(measurement.measurement_data);
        }
    }
}