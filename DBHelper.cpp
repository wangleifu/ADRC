#include "DBHelper.hpp"
#include <iomanip>
#include <sstream>
/*
mysql_init()	      获取或初始化MYSQL结构
mysql_real_connect()  连接到MySQL服务器。
mysql_query()	      执行指定为“以Null终结的字符串”的SQL查询。
mysql_use_result()	  初始化逐行的结果集检索。
mysql_field_count()	  返回上次执行语句的结果集的列数。
mysql_fetch_row()	  从结果集中获取下一行
mysql_num_fields()	  返回结果集中的字段数
*/
using namespace std;

DBHelper::DBHelper()
{
	con = mysql_init(NULL);
	if(con == NULL)
	{
		std::cout << "Struct Error:" << mysql_error(con) << std::endl;
		exit(1);
	}

	DBHelper::initDB();
}

DBHelper::~DBHelper()
{
	if(con != NULL)
	{
		mysql_close(con);
	}
}

bool
DBHelper::initDB()
{
	std::string  host    = "localhost";
	std::string  user    = "root";
	std::string	pwd      = "root";
	std::string  db_name = "dji";

	con = mysql_real_connect(con, host.c_str(), user.c_str(),
		                     pwd.c_str(), db_name.c_str(), 3306, NULL, 0);
	if(con == NULL)
	{
		std::cout << "Initial Error" << mysql_error(con) << std::endl;
		exit(1);
	}

	menu_table = "flight_traces";
	return true;
}

// menu 表的数据库操作
bool
DBHelper::select_menu_and_print()
{
	sql = "select * from " + menu_table;
	if(!DBHelper::exeSelectSQL(sql))
	{
		return false;
	}
	return true;
}
bool
DBHelper::select_menu(string nick_name, vector<DATA::FlightData> &data)
{
	trace_table = DBHelper::get_table_name(nick_name);
	DBHelper::select_trace(data);
	return true;
}
bool
DBHelper::insert_menu(string nick_name, string describe, vector<DATA::FlightData> data)
{
	sql = "insert into "+ menu_table +
	      "(trace_nick_name, trace_table_name, trace_create_date, trace_describe)"+
	      " values('"+ nick_name +"','trace',now(),'"+describe+"')";
	if(!DBHelper::exeUpdateSQL(sql)){
		return false;
	}

	// 创建traceId表
	trace_table = DBHelper::get_table_name(nick_name);
	DBHelper::create_trace_table();

	// 插入数据
	DBHelper::insert_trace(data);
	return true;
}
bool
DBHelper::delete_menu(string nick_name)
{
	// drop掉traceId表
	trace_table = DBHelper::get_table_name(nick_name);
	DBHelper::drop_trace();

	// 删除对应的menu表中的记录
	sql = "delete from "+ menu_table +
		  " where trace_nick_name='"+nick_name+"'";
	DBHelper::exeUpdateSQL(sql);
	return true;
}
/*
bool
DBHelper::update_menu(string id, string nick_name, string describe)
{
	sql = "update "+ menu_table +" set trace_nick_name='"+ nick_name +
	      "' trace_describe='"+ describe +"'' where trace_id="+ id;
	DBHelper::exeUpdateSQL(sql);
	return true;
}
*/
/*
bool
DBHelper::delete_menu(string id)
{
	// drop掉traceId表
	trace_table = get_table_name(id);
	DBHelper::drop_trace();

	// 删除对应的menu表中的记录
	sql = "delete from "+ menu_table +" where trace_id="+ id;
	DBHelper::exeUpdateSQL(sql);
	return true;
}
*/




// 各个trace 表: 创建表、批量插入，查询所有、删除表
bool
DBHelper::create_trace_table()
{
	sql = "create table " + trace_table +"("+
	      "x float not null, y float not null, z float not null,"+
	      "Vx float not null, Vy float not null, Vz float not null,"+
	      "roll float not null, pitch float not null, yaw float not null,"+
	      "roll_rate float not null, pitch_rate float not null, yaw_rate float not null"+
	      ");";
	DBHelper::exeUpdateSQL(sql);
	return true;
}
bool
DBHelper::insert_trace(vector<DATA::FlightData> data)
{
	if(data.empty())
	{
		return false;
	}
	int size = data.size();
	string x,y,z,Vx,Vy,Vz,roll, pitch, yaw, rollRate, pitchRate, yawRate;

	for(int i= 0; i < size; ++i)
	{
		fd = data[i];

		x         = numToString(fd.pos.x);
		y         = numToString(fd.pos.y);
		z         = numToString(fd.pos.z);
		Vx        = numToString(fd.vel.Vx);
		Vy        = numToString(fd.vel.Vy);
		Vz        = numToString(fd.vel.Vz);
		roll      = numToString(fd.att.roll);
		pitch     = numToString(fd.att.pitch);
		yaw       = numToString(fd.att.yaw);
		rollRate  = numToString(fd.ang.rollRate);
		pitchRate = numToString(fd.ang.pitchRate);
		yawRate   = numToString(fd.ang.yawRate);

		sql = "insert into "+trace_table+" values("+
		  x+","+ y +","+ z +","+ Vx +","+ Vy +","+ Vz +","+
		  roll+","+pitch+","+yaw+","+rollRate+","+pitchRate+","+yawRate+
		  ")";
		DBHelper::exeUpdateSQL(sql);
	}
	return true;
}
bool
DBHelper::select_trace(vector<DATA::FlightData> &data)
{
	sql = "select * from " + trace_table;
	DBHelper::exeUpdateSQL(sql);

	res = mysql_store_result(con);

	int rowCount   = mysql_num_rows(res);

	// 每行12列数据：x,y,z,Vx,Vy.Vz,roll,pitch,yaw,rollRate,pitchRate,yawRate
	for(int i = 0; i < rowCount; ++i)
	{
		row = mysql_fetch_row(res);
		if(row <= 0)
		{
			std::cout << "Failed while getting data from database." << endl;
			return false;
		}
        fd.pos.x         = DBHelper::stringToNum<DATA::float32_t>(row[0]);
        fd.pos.y         = DBHelper::stringToNum<DATA::float32_t>(row[1]);
        fd.pos.z         = DBHelper::stringToNum<DATA::float32_t>(row[2]);
        fd.vel.Vx        = DBHelper::stringToNum<DATA::float32_t>(row[3]);
        fd.vel.Vy        = DBHelper::stringToNum<DATA::float32_t>(row[4]);
        fd.vel.Vz        = DBHelper::stringToNum<DATA::float32_t>(row[5]);
        fd.att.roll      = DBHelper::stringToNum<DATA::float32_t>(row[6]);
        fd.att.pitch     = DBHelper::stringToNum<DATA::float32_t>(row[7]);
        fd.att.yaw       = DBHelper::stringToNum<DATA::float32_t>(row[8]);
        fd.ang.rollRate  = DBHelper::stringToNum<DATA::float32_t>(row[9]);
        fd.ang.pitchRate = DBHelper::stringToNum<DATA::float32_t>(row[10]);
        fd.ang.yawRate   = DBHelper::stringToNum<DATA::float32_t>(row[11]);
		data.push_back(fd);
	}

	mysql_free_result(res);
	return true;
}
bool
DBHelper::drop_trace()
{
	sql = "drop table " + trace_table;
	if(!DBHelper::exeUpdateSQL(sql))
	{
		return false;
	}
	return true;
}





// sql的具体执行
bool
DBHelper::exeSelectSQL(string s)
{
	// mysql_query(): 执行成功返回： 0， 失败返回： 非0。
	if(mysql_query(con, s.c_str()))
	{
		std::cout << "Query Error: " << mysql_error(con) << std::endl;
		exit(1);
	}
	else
	{
		res = mysql_store_result(con);

		int rowCount   = mysql_num_rows(res);
		int fieldCount = mysql_num_fields(res);

		MYSQL_FIELD* field = NULL;
		for(int k = 0; k < fieldCount; ++k)
		{
			field = mysql_fetch_field_direct(res, k);
			std::cout << std::setw(21) << std::setfill(' ') << std::left << field->name;
			//std::cout << field->name << "\t";
		}
		std::cout << std::endl;

		for(int i = 0; i < rowCount; ++i)
		{
			row = mysql_fetch_row(res);
			if(row <= 0)
			{
				break;
			}
			for(int j = 0; j < fieldCount; ++j)
			{
				std::cout << std::setw(21) << std::setfill(' ') << std::left << row[j];
			}
			std::cout << endl;
		}

		mysql_free_result(res);
	}
	return true;
}
bool
DBHelper::exeUpdateSQL(string s)
{
	// mysql_query(con, s.c_str()), mysql_real_query(con, s.c_str(), strlen(s))
	if(mysql_query(con, s.c_str()))
	{
		cout <<"Insert or update error:" << mysql_error(con) <<endl;
		return false;
	}
	return true;
}


// 辅助拼接traceId表名
string
DBHelper::get_table_name(string nick_name)
{
	string id = DBHelper::selectId(nick_name);
	string name = "trace" + id;
	return name;
}
string
DBHelper::selectId(string nick_name)
{
	string id;
	sql = "select trace_id from "+menu_table+
	      " where trace_nick_name='"+nick_name+"'";
	if(mysql_query(con, sql.c_str()))
	{
		cout <<"Query error:" << mysql_error(con) <<endl;
	}
	else
	{
		res = mysql_store_result(con);
		row = mysql_fetch_row(res);
		id = row[0];

		mysql_free_result(res);
	}
	return id;
}
template <typename Type> Type
DBHelper::stringToNum(const string& str)  
{  
    istringstream  iss(str);  
    Type           num;

    iss>>num;
    return num;
}
template <typename Type> string
DBHelper::numToString(const Type& num)  
{  
    ostringstream oss;   //创建一个格式化输出流
    oss<<num;             //把值传递如流中
    return oss.str(); 
}


string
DBHelper::get_traces()
{
    string traces;
    string s = "select * from " + menu_table;
    // mysql_query(): 执行成功返回： 0， 失败返回： 非0。
    if(mysql_query(con, s.c_str()))
    {
        std::cout << "Query Error: " << mysql_error(con) << std::endl;
        exit(1);
    }
    else
    {
        res = mysql_store_result(con);

        int rowCount   = mysql_num_rows(res);
        for(int i = 0; i < rowCount; ++i)
        {
            row = mysql_fetch_row(res);
            if(row <= 0)
            {
                break;
            }
            traces += row[1];
            if (i != rowCount -1)
            {
                traces += "-";
            }
        }

        mysql_free_result(res);
    }
    return traces;
}
