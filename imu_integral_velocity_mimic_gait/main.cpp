#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h> 

using namespace std;

struct IMU_data
{
    double timestamp;
    double accx;
    double accy;
    double accz;
};
void BandPass(vector<double> data, double apha_hp, double apha_lp, vector<double>& data_after);
void High_Pass(vector<double> data, double apha, vector<double>& data_after_hp);
void Low_Pass(vector<double> data, double apha, vector<double>& data_after_lp);
void high_pass(double apha, double input, double pre_input, double pre_output, double &output);
void low_pass(double apha, double input, double pre_output, double &output);

vector<IMU_data> ReadData(int argc, char* argv[]);
void WriteData(vector<double> data);

void GetMagnitude(vector<double> x, vector<double> y, vector<double> z, vector<double>& magnitude);
void GetStationary(double stationary_value, vector<double> magnitude, vector<bool>& stationary);
void GetVelocity(vector<double> timestamp, vector<bool> stationary, vector<double> acc, vector<double>& vel);


//low pass: bigger apha, bigger cutoff freq 
//high pass: bigger apha, smaller cutoff freq
int main(int argc, char* argv[])
{
    vector<IMU_data> IMU_list = ReadData(argc, argv);

    int data_size = IMU_list.size();

    vector<double> timestamp;
    vector<double> accx;
    vector<double> accy;
    vector<double> accz;
    vector<double> accx_hp;
    vector<double> accy_hp;
    vector<double> accz_hp;
    vector<double> accx_hp_lp;
    vector<double> accy_hp_lp;
    vector<double> accz_hp_lp;

    vector<double> accx_bp;
    vector<double> accy_bp;
    vector<double> accz_bp;


    for(int i=0; i<data_size;i++)
    {
        timestamp.push_back(IMU_list[i].timestamp);
        accx.push_back(IMU_list[i].accx);
        accy.push_back(IMU_list[i].accy);
        accz.push_back(IMU_list[i].accz);
    }


    //detect stationary period
    double apha_hp = 1.0;
    double apha_lp = 1.0;

    BandPass(accx, apha_hp, apha_lp, accx_bp);
    BandPass(accy, apha_hp, apha_lp, accy_bp);
    BandPass(accz, apha_hp, apha_lp, accz_bp);

    // High_Pass(accx, apha_hp, accx_hp);
    // High_Pass(accy, apha_hp, accy_hp);
    // High_Pass(accz, apha_hp, accz_hp);

    // Low_Pass(accx_hp, apha_lp, accx_hp_lp);
    // Low_Pass(accy_hp, apha_lp, accy_hp_lp);
    // Low_Pass(accz_hp, apha_lp, accz_hp_lp);

    vector<double> magnitude;
    //GetMagnitude(accx_hp_lp, accy_hp_lp, accz_hp_lp, magnitude);
    GetMagnitude(accx_bp, accy_bp, accz_bp, magnitude);


    double stationary_value = 0.05;
    vector<bool> stationary;
    GetStationary(stationary_value, magnitude, stationary);

    //compute translational velocity
    vector<double> vely;
    GetVelocity(timestamp, stationary, accy, vely);


    WriteData(vely);

    return 0;
}

void BandPass(vector<double> data, double apha_hp, double apha_lp, vector<double>& data_after)
{
    double pre_input = data[0];
    double pre_output = data[0];

    data_after.clear();
    data_after.push_back(data[0]);

    for(int i=1; i<data.size(); i++)
    {
        double output_hp;
        double output_lp;
        high_pass(apha_hp, data[i], pre_input, pre_output, output_hp);
        low_pass(apha_lp, output_hp, pre_output, output_lp);

        data_after.push_back(output_lp);
        pre_input = data[i];
        pre_output = output_lp;

    }
}



void High_Pass(vector<double> data, double apha, vector<double>& data_after_hp)
{
    double pre_input = data[0];
    double pre_output = data[0];
    data_after_hp.clear();
    data_after_hp.push_back(data[0]);

    for(int i=1; i<data.size(); i++)
    {
        double output;
        high_pass(apha, data[i], pre_input, pre_output, output);
        data_after_hp.push_back(output);
        pre_input = data[i];
        pre_output = output;
    }
}

void Low_Pass(vector<double> data, double apha, vector<double>& data_after_lp)
{
    double pre_output = data[0];
    data_after_lp.clear();
    data_after_lp.push_back(data[0]);

    for(int i=1; i<data.size(); i++)
    {
        double output;
        low_pass(apha, data[i], pre_output, output);
        data_after_lp.push_back(output);
        pre_output = output;
    }
}

void high_pass(double apha, double input, double pre_input, double pre_output, double &output)
{
    output = apha * pre_output + apha * ( input - pre_input );
}

void low_pass(double apha, double input, double pre_output, double &output)
{
    output = apha * input + (1-apha) * pre_output;
}



vector<IMU_data> ReadData(int argc, char* argv[])
{
    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string line;
    vector<IMU_data> IMU_data_list;

    while (getline(in_file_, line)) 
    {
        istringstream iss(line);
        IMU_data imu_data;
        
        double timestamp;
        double accx;
        double accy;
        double accz;

        iss >> timestamp;
        iss >> accx;
        iss >> accy;
        iss >> accz;

        imu_data.timestamp = timestamp;
        imu_data.accx = accx;
        imu_data.accy = accy;
        imu_data.accz = accz;

        IMU_data_list.push_back(imu_data);
    }

    return IMU_data_list;
}

void WriteData(vector<double> data)
{
    string out_file_name_ = "output.txt";
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    for(int i=0; i<data.size();i++)
    {
        out_file_<< data[i] << "\n";
    }
}

void GetMagnitude(vector<double> x, vector<double> y, vector<double> z, vector<double>& magnitude)
{
    magnitude.clear();
    int size = x.size();

    for(int i=0; i<size; i++)
    {
        double mag;
        mag = sqrt(x[i]*x[i] + y[i]*y[i] + z[i]*z[i]);
        magnitude.push_back(mag);
    }
}

void GetStationary(double stationary_value, vector<double> magnitude, vector<bool>& stationary)
{
    stationary.clear();
    for(int i=0; i<magnitude.size(); i++)
    {
        if(magnitude[i] < stationary_value)
            stationary.push_back(true);
        else 
            stationary.push_back(false);
    }
}

void GetVelocity(vector<double> timestamp, vector<bool> stationary, vector<double> acc, vector<double>& vel)
{
    int N = timestamp.size();

    double pre_time = timestamp[0];
    double pre_output = 0.0;
    double pre_input = 0.0;

    vel.clear();
    vel.push_back(pre_output);

    for(int i=1; i<N; i++)
    {
        double velocity;
        double time_now = timestamp[i];
        double time_diff = time_now - pre_time;

        velocity = pre_output + acc[i] * time_diff;

        if (stationary[i] == true)
            velocity = 0.0;

        double apha = 1.0;
        double output;
        high_pass(apha, velocity, pre_input, pre_output, output);


        pre_time = time_now;
        pre_output = output;
        pre_input = velocity;

        vel.push_back(output);
    }
}

