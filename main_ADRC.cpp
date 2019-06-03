#include "ADRC.hpp"
#include "flight_controller.hpp"

#include <dji_linux_helpers.hpp>
#include <string>

float ADRC_unit[3][15] =
{
    //TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO          扰动补偿           非线性组合
    //  r      h      N                beta_01   beta_02    beta_03      b0         k_0      k_1    k_2       N1     C    alpha1  alpha2   zeta
    { 75000 ,  0.02,  2,               25,        100,        100,       0.001,      0.002,   1.0,   0.0005,    5,    5,    0.8,    1.5,     50 },
    { 75000 ,  0.02,  2,               100,      1000,       2000,       0.001,      0.002,   1.0,   0.0005,    5,    5,    0.8,    1.5,     50 },
    { 50000 ,  0.005, 30,              100,      2000,      10000,           5,      0.002,   10,     0.001,    5,    5,    0.5,   1.05,     50 },
};

using namespace DJI::OSDK;
void ADRC(FlightController f_controller, std::string traceName, Fhan_Data ADRC_X_controller, Fhan_Data ADRC_Y_controller);
void ADRC_init(Fhan_Data *fhan_Input1, Fhan_Data *fhan_Input2, int base);


int main(int argc, char *argv[])
{
	// Setup OSDK.
    std:cout << "look here!" << std::endl;
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle*   vehicle = linuxEnvironment.getVehicle();
    if (vehicle == NULL)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }
    FlightController f_controller(vehicle);

    //  首先， 录制路径
//    while (true)
//    {
//        f_controller.recordFlightData();
//        usleep(20 * 1000); // record执行频率50Hz， 休眠20ms
//    }

    std::string traces = f_controller.get_trace();

    std::cout << "traces: " << traces << std::endl;
    std::cout << "plesea input trace name: " << std::endl;
    std::string msg;
    cin >> msg;

    for (int i = 0; i < 1; i++)
    {
        Fhan_Data ADRC_X_controller = { 0 }, ADRC_Y_controller = { 0 };
        // 参数初始化
        ADRC_init(&ADRC_X_controller, &ADRC_Y_controller, i);
        // 轨迹跟踪
        ADRC(f_controller, msg, ADRC_X_controller, ADRC_Y_controller);
        // 保存跟踪路径轨迹到文件， 便于与元路径进行对比
        f_controller.save("ADRC_i-i.txt", "50Hz");
        // 清空本次轨迹， 为下次记录做准备
        f_controller.clear();

        // 回到起点， 准备下次飞行
        f_controller.goHome();
    }

	return 0;
}

void ADRC(FlightController f_controller, std::string traceName, Fhan_Data ADRC_X_controller, Fhan_Data ADRC_Y_controller)
{
    // 先获取要跟踪的轨迹的飞行数据
    vector<DATA::FlightData>  flight_data;
    f_controller.getData(traceName, flight_data);
    if(flight_data.empty())
    {
        DERROR("flight data is empty.\n");
        return;
    }

    // 获取控制权
    f_controller.obtainControl();

    int d_size                      = flight_data.size();
    DATA::Positon cur_position      = {0.0,0.0,0.0};
    DATA::Positon u_position        = {0.0,0.0,0.0};

    for(int i = 0; i < d_size;i++)
    {

        u_position.x = ADRC_Control(&ADRC_X_controller,
                                    flight_data[i].pos.x,//期望x位置
                                    cur_position.x);     //实际x位置状态反馈
        u_position.y = ADRC_Control(&ADRC_Y_controller,
                                    flight_data[i].pos.y,//期望y位置
                                    cur_position.y);     //实际y位置状态反馈
        f_controller.flightByPosAndYaw(u_position, flight_data[i]);
    }

    // 获取控制权
    f_controller.releaseControl();
}

// ADRC参数初始化
void ADRC_init(Fhan_Data *fhan_Input1, Fhan_Data *fhan_Input2, int base)
{
    fhan_Input1->r = ADRC_unit[0][0];
    fhan_Input1->h = ADRC_unit[0][1];
    fhan_Input1->N0 = (uint16_t)(ADRC_unit[0][2]);
    fhan_Input1->beta_01 = ADRC_unit[0][3];
    fhan_Input1->beta_02 = ADRC_unit[0][4];
    fhan_Input1->beta_03 = ADRC_unit[0][5];
    fhan_Input1->b0 = ADRC_unit[0][6];
    fhan_Input1->k_0 = ADRC_unit[0][7];
    fhan_Input1->k_1 = ADRC_unit[0][8];
    fhan_Input1->k_2 = ADRC_unit[0][9];
    fhan_Input1->N1 = (uint16_t)(ADRC_unit[0][10]);
    fhan_Input1->c = ADRC_unit[0][11];

    fhan_Input1->alpha1 = ADRC_unit[0][12];
    fhan_Input1->alpha2 = ADRC_unit[0][13];
    fhan_Input1->zeta = ADRC_unit[0][14];

    fhan_Input2->r = ADRC_unit[1][0];
    fhan_Input2->h = ADRC_unit[1][1];
    fhan_Input2->N0 = (uint16_t)(ADRC_unit[1][2]);
    fhan_Input2->beta_01 = ADRC_unit[1][3];
    fhan_Input2->beta_02 = ADRC_unit[1][4];
    fhan_Input2->beta_03 = ADRC_unit[1][5];
    fhan_Input2->b0 = ADRC_unit[1][6];
    fhan_Input2->k_0 = ADRC_unit[1][7];
    fhan_Input2->k_1 = ADRC_unit[1][8];
    fhan_Input2->k_2 = ADRC_unit[1][9];
    fhan_Input2->N1 = (uint16_t)(ADRC_unit[1][10]);
    fhan_Input2->c = ADRC_unit[1][11];

    fhan_Input2->alpha1 = ADRC_unit[1][12];
    fhan_Input2->alpha2 = ADRC_unit[1][13];
    fhan_Input2->zeta = ADRC_unit[1][14];
}
