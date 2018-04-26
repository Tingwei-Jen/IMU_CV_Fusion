#include "fusion.h"
#include <iostream>

Fusion::Fusion()
{
    initialized = false;
    ekf = new KalmanFilter();

    //state: [px, py, pz, vx, vy , vz, ax, ay, az]

    R_cv = Eigen::MatrixXd(3,3);
    C_cv = Eigen::MatrixXd(3,9);
    R_imu = Eigen::MatrixXd(3,3);
    C_imu = Eigen::MatrixXd(3,9);

    R_cv << 0.00001, 0, 0,
            0, 0.00001, 0,
            0, 0, 0.00001;

    R_imu << 0.01, 0, 0,
             0, 0.01, 0,
             0, 0, 0.01;

    C_cv << 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0;
    
    C_imu << 0, 0, 0, 0, 0, 0, 1, 0, 0,
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

            if (abs(px)<0.00001 && abs(py)<0.00001 && abs(pz)<0.00001)
            {
                px = 0.00001;
                py = 0.00001;
                pz = 0.00001;
            }

            Eigen::VectorXd x0(9);
            x0<<px, py, pz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            ekf->set_state(x0);
        }
        else if (measurement.sensor_type_ == Measurement::IMU)
        {
            float accx = measurement.measurement_data(0);
            float accy = measurement.measurement_data(1);
            float accz = measurement.measurement_data(2);

            if (abs(accx)<0.00001 && abs(accy)<0.00001 && abs(accz)<0.00001)
            {
                accx = 0.00001;
                accy = 0.00001;
                accz = 0.00001;
            }
            Eigen::VectorXd x0(9);
            x0<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, accx, accy, accz;
            ekf->set_state(x0);
        }

        Eigen::MatrixXd P0(9,9);
        P0<<0.05, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0.05, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0.05, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0.05, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0.05, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.05; 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0.05, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0.05, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0.05;

        ekf->set_P_matrix(P0);
        std::cout<<"Kalman Filter Init: x & p!!"<<std::endl;
        previous_timestamp_ = measurement.timestamp_;

        initialized = true;
        return;
    }
    else
    {
        double dt = measurement.timestamp_ - previous_timestamp_;
        double dt_3 = dt*dt*dt;  //dt^3
        double dt_2 = dt*dt; //dt^2
        
        previous_timestamp_ = measurement.timestamp_;
        
        Eigen::MatrixXd A_(9,9);
        A_<<1, 0, 0, dt, 0, 0, dt_2/2, 0, 0,
            0, 1, 0, 0, dt, 0, 0, dt_2/2, 0,
            0, 0, 1, 0, 0, dt, 0, 0, dt_2/2,
            0, 0, 0, 1, 0, 0, dt, 0, 0,
            0, 0, 0, 0, 1, 0, 0, dt, 0,
            0, 0, 0, 0, 0, 1, 0, 0, dt,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1;

        ekf->set_A_matrix(A_);

        double noisy = 9.0;
        Eigen::VectorXd w(9);
        w<< noisy * dt_3/6, 
            noisy * dt_3/6, 
            noisy * dt_3/6, 
            noisy * dt_2/2, 
            noisy * dt_2/2, 
            noisy * dt_2/2, 
            noisy * dt, 
            noisy * dt, 
            noisy * dt;

        Eigen::MatrixXd Q_(9,9); 
        Q_<<w(0)*w(0), 0, 0, 0, 0, 0, 0, 0, 0,
            0, w(1)*w(1), 0, 0, 0, 0, 0, 0, 0,
            0, 0, w(2)*w(2), 0, 0, 0, 0, 0, 0,
            0, 0, 0, w(3)*w(3), 0, 0, 0, 0, 0,
            0, 0, 0, 0, w(4)*w(4), 0, 0, 0, 0,
            0, 0, 0, 0, 0, w(5)*w(5), 0, 0, 0,
            0, 0, 0, 0, 0, 0, w(6)*w(6), 0, 0,
            0, 0, 0, 0, 0, 0, 0, w(7)*w(7), 0,
            0, 0, 0, 0, 0, 0, 0, 0, w(8)*w(8);

        ekf->set_Q_matrix(Q_);

        ekf->predict();

        //update IMU
        if (measurement.sensor_type_ == Measurement::IMU)
        {
            ekf->set_C_matrix(C_imu);
            ekf->set_R_matrix(R_imu);
            ekf->update(measurement.measurement_data);
        } 
        else   //update CV
        {
            ekf->set_C_matrix(C_cv);
            ekf->set_R_matrix(R_cv);
            ekf->update(measurement.measurement_data);
        }
    }
}

 void Fusion::ProcessMeasurement_imu(const Measurement &measurement)
 {
    if(!initialized)
    {
        if(measurement.sensor_type_ == Measurement::CV)
        {   
            float px = measurement.measurement_data(0);
            float py = measurement.measurement_data(1);
            float pz = measurement.measurement_data(2);

            if (abs(px)<0.00001 && abs(py)<0.00001 && abs(pz)<0.00001)
            {
                px = 0.00001;
                py = 0.00001;
                pz = 0.00001;
            }

            Eigen::VectorXd x0(9);
            x0<<px, py, pz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            ekf->set_state(x0);
        }
        else if (measurement.sensor_type_ == Measurement::IMU)
        {
            float accx = measurement.measurement_data(0);
            float accy = measurement.measurement_data(1);
            float accz = measurement.measurement_data(2);

            if (abs(accx)<0.00001 && abs(accy)<0.00001 && abs(accz)<0.00001)
            {
                accx = 0.00001;
                accy = 0.00001;
                accz = 0.00001;
            }
            Eigen::VectorXd x0(9);
            x0<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, accx, accy, accz;
            ekf->set_state(x0);
        }

        Eigen::MatrixXd P0(9,9);
        P0<<0.05, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0.05, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0.05, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0.05, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0.05, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.05; 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0.05, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0.05, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0.05;

        ekf->set_P_matrix(P0);
        std::cout<<"Kalman Filter Init: x & p!!"<<std::endl;
        previous_timestamp_ = measurement.timestamp_;

        initialized = true;
        return;
    }
    else
    {

        double dt = measurement.timestamp_ - previous_timestamp_;
        double dt_3 = dt*dt*dt;  //dt^3
        double dt_2 = dt*dt; //dt^2
        
        previous_timestamp_ = measurement.timestamp_;
        
        Eigen::MatrixXd A_(9,9);
        A_<<1, 0, 0, dt, 0, 0, dt_2/2, 0, 0,
            0, 1, 0, 0, dt, 0, 0, dt_2/2, 0,
            0, 0, 1, 0, 0, dt, 0, 0, dt_2/2,
            0, 0, 0, 1, 0, 0, dt, 0, 0,
            0, 0, 0, 0, 1, 0, 0, dt, 0,
            0, 0, 0, 0, 0, 1, 0, 0, dt,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1;

        ekf->set_A_matrix(A_);

        double noisy = 9.0;
        Eigen::VectorXd w(9);
        w<< noisy * dt_3/6, 
            noisy * dt_3/6, 
            noisy * dt_3/6, 
            noisy * dt_2/2, 
            noisy * dt_2/2, 
            noisy * dt_2/2, 
            noisy * dt, 
            noisy * dt, 
            noisy * dt;

        Eigen::MatrixXd Q_(9,9); 
        Q_<<w(0)*w(0), 0, 0, 0, 0, 0, 0, 0, 0,
            0, w(1)*w(1), 0, 0, 0, 0, 0, 0, 0,
            0, 0, w(2)*w(2), 0, 0, 0, 0, 0, 0,
            0, 0, 0, w(3)*w(3), 0, 0, 0, 0, 0,
            0, 0, 0, 0, w(4)*w(4), 0, 0, 0, 0,
            0, 0, 0, 0, 0, w(5)*w(5), 0, 0, 0,
            0, 0, 0, 0, 0, 0, w(6)*w(6), 0, 0,
            0, 0, 0, 0, 0, 0, 0, w(7)*w(7), 0,
            0, 0, 0, 0, 0, 0, 0, 0, w(8)*w(8);

        ekf->set_Q_matrix(Q_);

        if (measurement.sensor_type_ == Measurement::IMU)
        {
            float accx = measurement.measurement_data(0);
            float accy = measurement.measurement_data(1);
            float accz = measurement.measurement_data(2);

            Eigen::VectorXd X = ekf->getState();
                
            X(6) = accx;
            X(7) = accy;
            X(8) = accz;

            ekf->set_state(X);
        }
        
        ekf->predict();
        
        if(measurement.sensor_type_ == Measurement::CV)
        {
            ekf->set_C_matrix(C_cv);
            ekf->set_R_matrix(R_cv);
            ekf->update(measurement.measurement_data);
        }
    }

 }