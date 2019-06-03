#ifndef TRACE_H
#define TRACE_H
#include <vector>
#include <string>
#include <fstream>
#include "DBHelper.hpp"
using namespace std;

class Trace
{
public:
	Trace();
	~Trace();

public:
	bool print_menu();
    bool push(DATA::FlightData data);              // 把数据 fd 存入向量 flight_data
public:
    bool clear();
    string get_trace();
	bool save(std::string trace_nick_name, std::string describe);                                   // 把数据从向量 flight_data 存入数据库    
    bool get(std::string trace_nick_name,
             vector<DATA::FlightData> &vec_data);  // 从数据库读取数据到向量 flight_data
    
    bool save_delta(std::string nick_name);
    void get_and_save(std::string trace_nick_name, std::string nick_name);

private:
    vector<DATA::FlightData> flight_data;
	DBHelper                 db_helper;
};
#endif
