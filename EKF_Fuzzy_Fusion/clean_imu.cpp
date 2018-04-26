#include "clean_imu.h"
#include <iostream>

using namespace std;

CleanImu::CleanImu(Config _config)
{
    m_apha_acc_highpass = _config.apha_acc_highpass; 
    m_apha_acc_lowpass = _config.apha_acc_lowpass;

    initialization = false;

    for(int i=0; i<3; i++)
    {
        pre_input_acc.push_back(0.0);
        pre_output_acc.push_back(0.0);
    }
}

void CleanImu::BandPass(Measurement& measurement)
{
    if (measurement.sensor_type_ == Measurement::IMU)
    {
        if(!initialization)
        {
            for(int j=0; j<3; j++)
            {   
                pre_input_acc.push_back(measurement.measurement_data(j));
                pre_output_acc.push_back(measurement.measurement_data(j));
            }      
            initialization = true;
        }
        else
        {
            vector<double> acc_input;
            for(int j=0; j<3; j++)
            {
                acc_input.push_back(measurement.measurement_data(j));
            }
            
            vector<double> acc_bp;
            for(int j=0; j<3; j++)
            {
                double output_hp;
                double output_lp;
                high_pass(m_apha_acc_highpass, acc_input[j], pre_input_acc[j], pre_output_acc[j], output_hp);
                low_pass(m_apha_acc_lowpass, output_hp, pre_output_acc[j], output_lp);
                acc_bp.push_back(output_lp);
                
                pre_input_acc[j] = acc_input[j];
                pre_output_acc[j] = output_lp;
            }
            
            for(int j=0; j<3; j++)
            {
                measurement.measurement_data(j) = acc_bp[j];
            }   
        }
    }
}


/*
private
*/
void CleanImu::high_pass(double apha, double input, double pre_input, double pre_output, double &output)
{
    output = apha * pre_output + apha * ( input - pre_input );
}

void CleanImu::low_pass(double apha, double input, double pre_output, double &output)
{
    output = apha * input + (1-apha) * pre_output;
}