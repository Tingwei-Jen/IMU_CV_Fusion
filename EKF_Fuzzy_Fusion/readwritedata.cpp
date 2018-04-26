#include "readwritedata.h"

using namespace std;

ReadWriteData::ReadWriteData()
{

}

ReadWriteData::~ReadWriteData()
{

}

void ReadWriteData::getMeasurementList(std::vector<Measurement>& measurement_list)
{

    ifstream in_file_(m_in_file_name.c_str(), ifstream::in);
    measurement_list.clear();
    string line;

    while (getline(in_file_, line)) 
    {
		string sensor_type;
        Measurement meas_package;
        double timestamp;

		istringstream iss(line);
        iss >> sensor_type;
		if (sensor_type.compare("IMU") == 0)
		{		
			meas_package.sensor_type_ = Measurement::IMU;
            float accx;
            float accy;
            float accz;
            iss >> timestamp;
            iss >> accx;
            iss >> accy;
            iss >> accz;
			meas_package.measurement_data = Eigen::VectorXd(3);
			meas_package.measurement_data<< accx, accy, accz;
			meas_package.timestamp_ = timestamp - time_bias;
			measurement_list.push_back(meas_package);
		}
		else if (sensor_type.compare("CV") == 0)
		{
			meas_package.sensor_type_ = Measurement::CV;
            float px;
            float py;
            float pz;
            iss >> timestamp;
            iss >> px;
            iss >> py;
            iss >> pz;
			meas_package.measurement_data = Eigen::VectorXd(3);
			meas_package.measurement_data<< px, py, pz;
			meas_package.timestamp_ = timestamp - time_bias;
			measurement_list.push_back(meas_package);
        }
        else if (sensor_type.compare("KF") == 0) 
        {
            meas_package.sensor_type_ = Measurement::KF;
            float px;
            float py;
            float pz;
            iss >> timestamp;
            iss >> px;
            iss >> py;
            iss >> pz;
			meas_package.measurement_data = Eigen::VectorXd(3);
			meas_package.measurement_data<< px, py, pz;
			meas_package.timestamp_ = timestamp - time_bias;
			measurement_list.push_back(meas_package);
        }
    }
}

void ReadWriteData::WriteData(std::vector<std::vector<double>> output_data_list)
{
    ofstream out_file_(m_out_file_name.c_str(), ofstream::out);

    int N = output_data_list.size();
    for(int i=0; i<N; i++)
    {
        for(int j=0; j<output_data_list[i].size();j++)
        {
            out_file_ << output_data_list[i][j] << "\t";
        }
        out_file_<< "\n";
    }
}