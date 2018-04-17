#ifndef HL_PASS_FILTER_H_
#define HL_PASS_FILTER_H_

#include <Eigen/Dense>
#include "measurement.h"

using Eigen::VectorXd;

class HLPassFilter
{
public:
    HLPassFilter();
    ~HLPassFilter();
    void HP_Filter_IMU(double apha, std::vector<Measurement>& measurement_list);
    void LP_Filter_IMU(double apha, std::vector<Measurement>& measurement_list);

private:
    void low_pass(VectorXd input, VectorXd pre_output, float apha, VectorXd &output);
    void high_pass(VectorXd input, VectorXd pre_input, float apha, VectorXd pre_output, VectorXd &output);

private:
    bool initialization;
    VectorXd pre_input; 
    VectorXd pre_output;
    
};
#endif  //HL_PASS_FILTER_H_