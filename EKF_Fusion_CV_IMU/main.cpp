#include <vector>
#include <Eigen/Dense>

#include "fusion.h"
#include "measurement.h"
#include "readwritedata.h"
#include "hl_pass_filter.h"

using namespace std;

int main(int argc, char* argv[]){

    ReadWriteData readwritedata;
    Fusion fusion;
    HLPassFilter hlpassfilter;

    string in_file_name_ = argv[1];
    string out_file_name_ = argv[2];
    
    readwritedata.init_infile(in_file_name_);
    readwritedata.init_outfile(out_file_name_);

    std::vector<Measurement> measurement_list;
    readwritedata.GetMeasurementList(measurement_list);

    //double apha = 0.3;
    //hlpassfilter.HP_Filter_IMU(apha, measurement_list);
    //hlpassfilter.LP_Filter_IMU(apha, measurement_list);

    std::vector<std::vector<double>> output_data_list;

	  int N = measurement_list.size();
	  for (int i=0; i<N; i++)
	  {
	      fusion.ProcessMeasurement(measurement_list[i]);

        std::vector<double> out_data;

        for(int i=0; i<9; i++)
        {
          out_data.push_back(fusion.getState()(i));
        }

	      // output the measurements
        if (measurement_list[i].sensor_type_ == Measurement::IMU) 
        {
      	    //補0
      	    out_data.push_back(0.0);
            out_data.push_back(0.0);
            out_data.push_back(0.0);
            out_data.push_back(measurement_list[i].measurement_data(0));
            out_data.push_back(measurement_list[i].measurement_data(1));
            out_data.push_back(measurement_list[i].measurement_data(2));           
        } 
		    else if (measurement_list[i].sensor_type_ == Measurement::CV) 
		    {
            out_data.push_back(measurement_list[i].measurement_data(0));
            out_data.push_back(measurement_list[i].measurement_data(1));
            out_data.push_back(measurement_list[i].measurement_data(2));
            out_data.push_back(0.0);
            out_data.push_back(0.0);
            out_data.push_back(0.0);         
   	    }
        output_data_list.push_back(out_data);
	  }
    readwritedata.WriteData(output_data_list);

	  return 0;
}
