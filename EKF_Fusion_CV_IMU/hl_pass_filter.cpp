#include "hl_pass_filter.h"

HLPassFilter::HLPassFilter()
{
    initialization = false;
}
HLPassFilter::~HLPassFilter()
{

}
    
void HLPassFilter::HP_Filter_IMU(double apha, std::vector<Measurement>& measurement_list)
{
    for(int i=0; i<measurement_list.size();i++)
    {
        if(measurement_list[i].sensor_type_ == Measurement::IMU)
        {
            if(!initialization)
            {
                pre_input = measurement_list[i].measurement_data;
                pre_output = measurement_list[i].measurement_data;
                initialization = true;
            }
            else 
            {
                VectorXd input = measurement_list[i].measurement_data;
                VectorXd output;
                high_pass(input, pre_input, apha, pre_output, output);
                
                measurement_list[i].measurement_data = output;
                pre_input = input;
                pre_output = output;
            }
        }
    }
}

void HLPassFilter::LP_Filter_IMU(double apha, std::vector<Measurement>& measurement_list)
{
    for(int i=0; i<measurement_list.size();i++)
    {
        if(measurement_list[i].sensor_type_ == Measurement::IMU)
        {
            if(!initialization)
            {
                pre_output = measurement_list[i].measurement_data;
                initialization = true;
            }
            else 
            {
                VectorXd input = measurement_list[i].measurement_data;
                VectorXd output;
                low_pass(input, pre_output, apha,  output);
                
                measurement_list[i].measurement_data = output;
                pre_output = output;
            }
        }
    }
}


void HLPassFilter::low_pass(VectorXd input, VectorXd pre_output, float apha, VectorXd &output)
{
    output = apha * input + (1-apha) * pre_output;
}

void HLPassFilter::high_pass(VectorXd input, VectorXd pre_input, float apha, VectorXd pre_output, VectorXd &output)
{
    output = apha * pre_output + apha * ( input - pre_input );
}
