#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

struct CV_data
{
    float px;
    float py;
    float pz;
};
struct IMU_data
{
    double timestamp;
    double accx;
    double accy;
    double accz;
};


int main(int argc, char* argv[]) 
{
    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string out_file_name_IMU = argv[2];
    ofstream out_file_IMU(out_file_name_IMU.c_str(), ofstream::out);
    
    string line;
    vector<IMU_data> IMU_data_list;

    while (getline(in_file_, line)) 
    {
        string sensor_type;
        IMU_data imu_data;

        istringstream iss(line);
        iss >> sensor_type;
        if (sensor_type.compare("IMU") == 0) 
        {
            double timestamp;
            double w;
            double x;
            double y;
            double z;
            double accx;
            double accy;
            double accz;
            iss >> timestamp;
            iss >> w;
            iss >> x;
            iss >> y;
            iss >> z;
            iss >> accx;
            iss >> accy;
            iss >> accz;

            imu_data.timestamp = timestamp - 1523340678.828200;
            imu_data.accx = accx;
            imu_data.accy = accy;
            imu_data.accz = accz;
            IMU_data_list.push_back(imu_data);

        }
    }

    int IMU_SIZE = IMU_data_list.size();

    for(int i=0; i<IMU_SIZE; i++)
    {
        out_file_IMU << IMU_data_list[i].timestamp << "\t";
        out_file_IMU << IMU_data_list[i].accx << "\t";
        out_file_IMU << IMU_data_list[i].accy << "\t";
        out_file_IMU << IMU_data_list[i].accz << "\n";
    }
 
    // close files
    if (out_file_IMU.is_open()) {
        out_file_IMU.close();
    }

    if (in_file_.is_open()) {
        in_file_.close();
    }

    return 0;
}