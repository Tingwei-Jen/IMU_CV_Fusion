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
    float accx;
    float accy;
    float accz;
};



int main(int argc, char* argv[]) 
{
    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string out_file_name_IMU = argv[2];
    ofstream out_file_IMU(out_file_name_IMU.c_str(), ofstream::out);

    string out_file_name_CV = argv[3];
    ofstream out_file_CV(out_file_name_CV.c_str(), ofstream::out);

    string line;

    vector<CV_data> CV_data_list;
    vector<IMU_data> IMU_data_list;


    while (getline(in_file_, line)) 
    {
        string sensor_type;
        
        CV_data cv_data;
        IMU_data imu_data;

        istringstream iss(line);
        iss >> sensor_type;
        if (sensor_type.compare("IMU") == 0) 
        {
            float timestamp;
            float w;
            float x;
            float y;
            float z;
            float accx;
            float accy;
            float accz;
            iss >> timestamp;
            iss >> w;
            iss >> x;
            iss >> y;
            iss >> z;
            iss >> accx;
            iss >> accy;
            iss >> accz;

            imu_data.accx = accx;
            imu_data.accy = accy;
            imu_data.accz = accz;
            IMU_data_list.push_back(imu_data);

        }
        else if (sensor_type.compare("CV") == 0)
        {
            float timestamp;
            float px;
            float py;
            float pz;
            float w;
            float x;
            float y;
            float z;

            iss >> timestamp;
            iss >> px;
            iss >> py;
            iss >> pz;
            iss >> w;
            iss >> x;
            iss >> y;
            iss >> z;

            cv_data.px = px;
            cv_data.py = py;
            cv_data.pz = pz;

            CV_data_list.push_back(cv_data);

        }
    }

    int IMU_SIZE = IMU_data_list.size();
    int CV_SIZE = CV_data_list.size();

    for(int i=0; i<IMU_SIZE; i++)
    {
        out_file_IMU << IMU_data_list[i].accx << "\t";
        out_file_IMU << IMU_data_list[i].accy << "\t";
        out_file_IMU << IMU_data_list[i].accz << "\n";
    }
    for(int i=0; i<CV_SIZE; i++)
    {
        out_file_CV << CV_data_list[i].px << "\t";
        out_file_CV << CV_data_list[i].py << "\t";
        out_file_CV << CV_data_list[i].pz << "\n";
    }
    
    // close files
    if (out_file_IMU.is_open()) {
        out_file_IMU.close();
    }
    if (out_file_CV.is_open()) {
        out_file_CV.close();
    }

    if (in_file_.is_open()) {
        in_file_.close();
    }

    return 0;
}