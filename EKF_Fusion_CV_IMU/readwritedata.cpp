#include "readwritedata.h"

using namespace std;

ReadWriteData::ReadWriteData()
{

}

ReadWriteData::~ReadWriteData()
{

}

void ReadWriteData::GetMeasurementList(std::vector<Measurement>& measurement_list)
{

    ifstream in_file_(m_in_file_name.c_str(), ifstream::in);
    measurement_list.clear();

    string line;

	Eigen::MatrixXd R_CV_W = Eigen::MatrixXd::Identity(3,3);
	Eigen::MatrixXd R_W_IMU = Eigen::MatrixXd::Identity(3,3);
	Eigen::MatrixXd R_CV_IMU = Eigen::MatrixXd::Identity(3,3);

    float pre_cvpx = 0.0;


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
			
			R_W_IMU = QuaternionRotationMatrix(w,x,y,z);
			R_CV_W = R_CV_IMU * R_W_IMU.inverse();

			Eigen::VectorXd acc_cv(3);
			Eigen::VectorXd acc_imu(3);
			acc_imu<< accx, accy, accz;

			acc_cv = R_CV_W * R_W_IMU * acc_imu;

			meas_package.measurement_data = Eigen::VectorXd(3);
			meas_package.measurement_data<< acc_cv(0), acc_cv(1), acc_cv(2);
			meas_package.timestamp_ = timestamp - time_bias;
			measurement_list.push_back(meas_package);
		}
		else if (sensor_type.compare("CV") == 0)
		{
			meas_package.sensor_type_ = Measurement::CV;
			
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
			
            if(px != pre_cvpx) //non lose tracking
            {
                R_CV_IMU = QuaternionRotationMatrix(w,x,y,z);

			    meas_package.measurement_data = Eigen::VectorXd(3);
			    meas_package.measurement_data<< px, py, pz;
			    meas_package.timestamp_ = timestamp - time_bias;
			    measurement_list.push_back(meas_package);

            }

            pre_cvpx = px;
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

Eigen::MatrixXd ReadWriteData::QuaternionRotationMatrix(float w, float x, float y, float z) {
    double m00, m01, m02, m10, m11, m12, m20, m21, m22;
    double sqw = pow(w, 2);
    double sqx = pow(x, 2);
    double sqy = pow(y, 2);
    double sqz = pow(z, 2);

    // invs (inverse square length) is only required if quaternion is not already normalised
    double invs = 1 / (sqx + sqy + sqz + sqw);
    m00 = ( sqx - sqy - sqz + sqw) * invs ;
    m11 = (-sqx + sqy - sqz + sqw) * invs ;
    m22 = (-sqx - sqy + sqz + sqw) * invs ;

    double tmp1 = x * y;
    double tmp2 = z * w;
    m10 = 2.0 * (tmp1 + tmp2) * invs ;
    m01 = 2.0 * (tmp1 - tmp2) * invs ;

    tmp1 = x * z;
    tmp2 = y * w;
    m20 = 2.0 * (tmp1 - tmp2) * invs ;
    m02 = 2.0 * (tmp1 + tmp2) * invs ;

    tmp1 = y * z;
    tmp2 = x * w;
    m21 = 2.0 * (tmp1 + tmp2) * invs ;
    m12 = 2.0 * (tmp1 - tmp2) * invs ;

		Eigen::MatrixXd R_(3,3);
		R_<< m00, m01, m02, m10, m11, m12, m20, m21, m22;
    return R_;
}