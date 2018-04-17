#ifndef READWRITEDATA_H_
#define READWRITEDATA_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include "measurement.h"

class ReadWriteData
{
public:
    ReadWriteData();
    ~ReadWriteData();    

    void init_infile(std::string in_file_name){ this->m_in_file_name = in_file_name; }
    void init_outfile(std::string out_file_name){ this->m_out_file_name = out_file_name; }
    void GetMeasurementList(std::vector<Measurement>& measurement_list);
    void WriteData(std::vector<std::vector<double>> output_data_list);

private:
    Eigen::MatrixXd QuaternionRotationMatrix(float w, float x, float y, float z);

private:
    std::string m_in_file_name;
    std::string m_out_file_name;
    const double time_bias = 1523340494.333508;
};

#endif //READWRITEDATA_H_