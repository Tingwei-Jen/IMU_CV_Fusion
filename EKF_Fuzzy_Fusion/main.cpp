#include <vector>
#include <Eigen/Dense>

#include "fusion.h"
#include "measurement.h"
#include "readwritedata.h"
#include "clean_imu.h"

using namespace std;

void RecordOutput(Measurement meas, std::vector<double>& out_data);

int main(int argc, char* argv[]){

    ReadWriteData readwritedata;
    Fusion fusion;
   
    string in_file_name_ = argv[1];
    string out_file_name_ = argv[2];
    
    readwritedata.init_infile(in_file_name_);
    readwritedata.init_outfile(out_file_name_);
    
    std::vector<Measurement> measurement_list;
    readwritedata.getMeasurementList(measurement_list);

    int N = measurement_list.size();


    // Config config;
    // config.apha_acc_highpass = 0.99;
    // config.apha_acc_lowpass = 0.99;
    // CleanImu cleanimu(config);
    // for (int i=0; i<N; i++)
    // {
    //     cleanimu.BandPass(measurement_list[i]);
    // }

    std::vector<std::vector<double>> output_data_list;
	for (int i=0; i<N; i++)
	{
          
	    fusion.ProcessMeasurement(measurement_list[i]);
        //fusion.ProcessMeasurement_imu(measurement_list[i]);

        std::vector<double> out_data;

        for(int i=0; i<6; i++)
        {
          //out_data.push_back(fusion.getState()(i));
        }

        RecordOutput(measurement_list[i], out_data);

        output_data_list.push_back(out_data);
	}
    
    readwritedata.WriteData(output_data_list);

	return 0;
}


void RecordOutput(Measurement meas, std::vector<double>& out_data)
{

    // output the measurements
    if (meas.sensor_type_ == Measurement::CV) 
    {
        //記得補0
        out_data.push_back(meas.measurement_data(0));
        out_data.push_back(meas.measurement_data(1));
        out_data.push_back(meas.measurement_data(2));
        out_data.push_back(0.0);
        out_data.push_back(0.0);
        out_data.push_back(0.0);
        out_data.push_back(0.0);
        out_data.push_back(0.0);
        out_data.push_back(0.0);             
    } 
	else if (meas.sensor_type_ == Measurement::IMU) 
	{
        out_data.push_back(0.0);
        out_data.push_back(0.0);
        out_data.push_back(0.0);   
        out_data.push_back(meas.measurement_data(0));
        out_data.push_back(meas.measurement_data(1));
        out_data.push_back(meas.measurement_data(2));
        out_data.push_back(0.0);
        out_data.push_back(0.0);
        out_data.push_back(0.0);  
   	}
	else if (meas.sensor_type_ == Measurement::KF) 
	{
        out_data.push_back(0.0);
        out_data.push_back(0.0);
        out_data.push_back(0.0);            
        out_data.push_back(0.0);
        out_data.push_back(0.0);
        out_data.push_back(0.0);   
        out_data.push_back(meas.measurement_data(0));
        out_data.push_back(meas.measurement_data(1));
        out_data.push_back(meas.measurement_data(2));
   	}
}
