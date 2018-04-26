#ifndef CLEAN_IMU_H_
#define CLEAN_IMU_H_

#include <Eigen/Dense>
#include "measurement.h"

struct Config
{
    double apha_acc_highpass;
    double apha_acc_lowpass;
};

class CleanImu
{
public:
    CleanImu(Config _config);
    ~CleanImu(){}
    void BandPass(Measurement& measurement);

private:
    void high_pass(double apha, double input, double pre_input, double pre_output, double &output);
    void low_pass(double apha, double input, double pre_output, double &output);

private:
    double m_apha_acc_highpass;
    double m_apha_acc_lowpass;
    bool initialization;

    std::vector<double> pre_input_acc;    //accx, accy, accz 
    std::vector<double> pre_output_acc;

};
#endif  //CLEAN_IMU_H_