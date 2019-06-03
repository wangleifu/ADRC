#include "mission_manager.hpp"

ThreadManager::ThreadManager()
{
    this->isFinish = true;
}
ThreadManager::~ThreadManager()
{}


void ThreadManager::setVehicle(DJI::OSDK::Vehicle *vehicle)
{
    this->f_controller = new FlightController(vehicle);
}
void ThreadManager::setTraceName(std::string traceName)
{
    this->traceName = traceName;
}
// 获取已保存的路径 名称， 返回身体的std：：string类型， 其中 “，” 为分割符
std::string ThreadManager::getTraces()
{
    return this->f_controller->get_trace();
}

bool ThreadManager::start(int missionType)
{
    // if not finish: running or just init
    // else:          there is no thread is running, then you can start a thread
    if (!isFinish)
    {
        DSTATUS("There is another mission %d is running.", p_thread.get_id());
        return false;
    }

    // start a thread
    switch(missionType)
    {
//    case TAKE_OFF:
//        p_thread = move(thread(takeOff, (void*)this));
//        break;
//    case LAND:
//        p_thread = move(thread(land, (void*)this));
//        break;
    case GO_HOME:
        p_thread = move(thread(goHome, (void*)this));
        break;
    case TRACKING:
        // p_thread = move(thread(tracking, (void*)this));
        p_thread = move(thread(ADRC_tracking, (void*)this));
        std::cout << " p_thead id:" << p_thread.get_id() << std::endl;
        break;
    case START_RECORD:
        p_thread = move(thread(startRecord, (void*)this));
        std::cout << " p_thead id:" << p_thread.get_id() << std::endl;
        break;
    default:
        break;
    }
    p_thread.detach();
    this->isFinish = false; // 标识有任务在执行

    return true;
}

// 暂停
void ThreadManager::_pause()
{
    if (!this->isLock)
    {
        this->lock.lock();
        this->isLock = true;
        DSTATUS("pasue.");
    }
}
// 继续
void ThreadManager::_continue()
{
    if (this->isLock)
    {
        this->lock.unlock();
        this->isLock = false;
        DSTATUS("continue.");
    }
}
// 结束
void ThreadManager::_finish()
{
    _pause();
    _continue();
    isFinish = true;  // 标识任务结束，可以接受新任务
    DSTATUS(" set isFinish: true.");
}





/*---起飞---*/
void* ThreadManager::takeOff(void* __this)
{
    ThreadManager * _this = (ThreadManager*)__this;
    _this->f_controller->monitoredTakeoff();
}
/*---降落---*/
void* ThreadManager::land(void* __this)
{
    ThreadManager * _this = (ThreadManager*)__this;
    _this->f_controller->monitoredLanding();
}
/*---返回原点---*/
void* ThreadManager::goHome(void* __this)
{
    ThreadManager * _this = (ThreadManager*)__this;
    _this->f_controller->goHome();
}
/*---飞到指定点---*/
void* ThreadManager::goPosition(void* __this)
{
    ThreadManager * _this = (ThreadManager*)__this;
    _this->f_controller->goToPosition(_this->m_currentPosition);
}
/*---路径跟踪 PID---*/
void* ThreadManager::tracking(void* __this)
{
    DSTATUS("flight by position and yaw...");
    ThreadManager * _this = (ThreadManager*)__this;
    _this->f_controller->obtainControl();


    vector<DATA::FlightData>  flight_data;
    _this->f_controller->getData(_this->traceName, flight_data);

    if(flight_data.empty())
    {
        DERROR("flight data is empty.\n");
        return NULL;
    }

    int d_size           = flight_data.size();
    DATA::Positon        ut         = {0.0,0.0,0.0};
    DATA::Positon        ut_last    = {0.0,0.0,0.0};
    DATA::Positon        ut_delta   = {0.0,0.0,0.0};
    DATA::Positon        delta      = {0.0,0.0,0.0};
    DATA::Positon        delta_last = {0.0,0.0,0.0};
    DATA::Positon        delta_sum  = {0.0,0.0,0.0};

    for(int i = 0; i < d_size;i+=2)
    {
        //std::cout << boolalpha << " isFinish flag:  "<< _this->isFinish << std::endl;
        if (_this->isFinish)
        {
            std::cout << "Mission to be Finish manully." << std::endl;
            return NULL;
        }
        _this->lock.lock();
        _this->f_controller->cal_delta(flight_data[i], delta);
        _this->f_controller->cal_ut(delta, delta_sum, delta_last, ut, ut_last, ut_delta);
        _this->f_controller->flightByPosAndYaw(ut_delta, flight_data[i]);

        //_this->m_currentPosition = flight_data[i];
        _this->lock.unlock();
        usleep(20 * 1000); // 命令执行频率50Hz， 休眠20ms
    }
}

/*---路径跟踪 ADPC---*/
void* ThreadManager::ADRC_tracking(void* __this)
{
    DSTATUS("flight by position and yaw...");
    ThreadManager * _this = (ThreadManager*)__this;

    // 先获取要跟踪的轨迹的飞行数据
    vector<DATA::FlightData>  flight_data;
    _this->f_controller->getData(_this->traceName, flight_data);
    if(flight_data.empty())
    {
        DERROR("flight data is empty.\n");
        return NULL;
    }

    // 获取控制权
    _this->f_controller->obtainControl();

    int d_size                      = flight_data.size();
    DATA::Positon cur_position      = {0.0,0.0,0.0};
    DATA::Positon u_position        = {0.0,0.0,0.0};

    Fhan_Data ADRC_X_controller = { 0 }, ADRC_Y_controller = { 0 };
    ADRC_Init(&ADRC_X_controller, &ADRC_Y_controller);

    for(int i = 0; i < d_size;i++) 
    {
        //std::cout << boolalpha << " isFinish flag:  "<< _this->isFinish << std::endl;
        if (_this->isFinish)
        {
            std::cout << "Mission to be Finish manully." << std::endl;
            return NULL;
        }

        u_position.x = ADRC_Control(&ADRC_X_controller,
                                    flight_data[i].pos.x,//期望x位置
                                    cur_position.x);     //实际x位置状态反馈
        u_position.y = ADRC_Control(&ADRC_Y_controller,
                                    flight_data[i].pos.y,//期望y位置
                                    cur_position.y);     //实际y位置状态反馈
        _this->f_controller->flightByPosAndYaw(u_position, flight_data[i]);
    }

    // 获取控制权
    _this->f_controller->releaseControl();

}

/*---start record---*/
void* ThreadManager::startRecord(void* __this)
{
    ThreadManager * _this = (ThreadManager*)__this;
    while (!_this->isFinish)
    {
        _this->f_controller->recordFlightData();
        usleep(20 * 1000); // record执行频率50Hz， 休眠20ms
    }
}
/*--- save trace ---*/
void ThreadManager::saveTrace(std::string traceName)
{
    _finish();
    if (!traceName.empty())
    {
        cout << "\ntrace name is not empty, save it!" << endl;
        f_controller->save(traceName, "50Hz");
    }else
    {
        cout << "\ntrace name is empty, do not save!" << endl;
    }
    f_controller->clear();
}
