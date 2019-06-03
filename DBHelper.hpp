#ifndef DBHELPER_H
#define DBHELPER_H

#include "data.hpp"

#include<iostream>
#include<string>
#include<vector>
#include<mysql/mysql.h>
using namespace std; 

class DBHelper
{
public:
	DBHelper();
	~DBHelper();

	// menu 表：增加，查询，修改，删除记录。
	// bool update_menu(string id,string nick_name, string describe);
	// bool delete_menu(string id);
    bool select_menu(string nick_name, vector<DATA::FlightData> &data);
	bool select_menu_and_print();
    bool insert_menu(string nick_name, string describe, vector<DATA::FlightData> data);
	bool delete_menu(string nick_name);
    string get_traces();

private:
	bool   initDB();
	bool   exeSelectSQL(string sql);
	bool   exeUpdateSQL(string sql);

	// 各个trace 表: 创建表、批量插入，查询所有、删除表
    bool select_trace(vector<DATA::FlightData> &data);
	bool create_trace_table();
    bool insert_trace(vector<DATA::FlightData> data);
	bool drop_trace();

	// 辅助
	string  get_table_name(string nick_name);
	string  selectId(string nick_name);

	template <typename Type> Type stringToNum(const string& str);
	template <typename Type> string numToString(const Type& num);
	
	MYSQL      *con;
	MYSQL_RES  *res;
	MYSQL_ROW   row;

	string      menu_table;
	string      trace_table;
	string      sql;

    DATA::FlightData  fd;
};

#endif
