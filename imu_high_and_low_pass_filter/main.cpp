#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>

using namespace std;

struct IMU_data
{
    float accx;
    float accy;
    float accz;
};

void low_pass(float input, float pre_output, float apha, float &output)
{
    output = apha * input + (1-apha) * pre_output;
}

void high_pass(float input, float pre_input, float apha, float pre_output, float &output)
{
    output = apha * pre_output + apha * ( input - pre_input );
}



int main(int argc, char* argv[])
{
    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string out_file_name_ = argv[2];
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    string line;
    vector<IMU_data> IMU_data_list;

    float pre_input_x;
    float pre_output_x;
    float pre_input_y;
    float pre_output_y;
    float pre_input_z;
    float pre_output_z;        

    bool inititalization = false;

    float apha = 0.8;


    while (getline(in_file_, line)) 
    {
        istringstream iss(line);
        IMU_data imu_data;

        if(!inititalization)
        {
            float accx;
            float accy;
            float accz;

            iss >> accx;
            iss >> accy;
            iss >> accz;
            
            pre_input_x = accx;
            pre_input_y = accy;
            pre_input_z = accz;

            pre_output_x = accx;      
            pre_output_y = accy;  
            pre_output_z = accz;  
            
            imu_data.accx = accx;
            imu_data.accy = accy;
            imu_data.accz = accz;

            IMU_data_list.push_back(imu_data);

            inititalization = true;
        }
        else 
        {
            float accx;
            float accy;
            float accz;

            iss >> accx;
            iss >> accy;
            iss >> accz;

            float output_x;
            float output_y;
            float output_z;
            high_pass(accx, pre_input_x, apha, pre_output_x, output_x);
            high_pass(accy, pre_input_y, apha, pre_output_y, output_y);
            high_pass(accz, pre_input_z, apha, pre_output_z, output_z);

            imu_data.accx = output_x;
            imu_data.accy = output_y;
            imu_data.accz = output_z;

            IMU_data_list.push_back(imu_data);

            pre_input_x = accx;
            pre_input_y = accy;
            pre_input_z = accz;

            pre_output_x = output_x;
            pre_output_y = output_y;
            pre_output_z = output_z;

        }
    }

    int N = IMU_data_list.size();
    for(int i=0; i<N; i++)
    {
        out_file_ << IMU_data_list[i].accx << "\t";
        out_file_ << IMU_data_list[i].accy << "\t";
        out_file_ << IMU_data_list[i].accz << "\n";
    }

    return 0;
}