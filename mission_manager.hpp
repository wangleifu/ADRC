#ifndef THREAD_MANAGER_HPP
#define THREAD_MANAGER_HPP

#include "flight_controller.hpp"
#include "ADRC.hpp"
#include <mutex>
#include <thread>

//enum MissionType {
//    _SET_HOME_POINT,         // 0
//    _OBTAIN_CTR_AUTHORITY,   // 1
//    _RELEASE_CTR_AUTHORITY,  // 2
//    _TAKE_OFF,               // 3
//    _LAND,                   // 4
//    _TRACKING,               // 5
//    _PAUSE,                  // 6
//    _CONTINUE,               // 7
//    _FINISH,                 // 8
//    _GO_HOME                 // 9
//};


class ThreadManager
{
public:
    static const int SET_HOME_POINT        = 0;
    static const int OBTAIN_CTR_AUTHORITY  = 1;
    static const int RELEASE_CTR_AUTHORITY = 2;
    static const int START_RECORD          = 3;
    static const int END_RECORD            = 4;
    static const int TRACKING              = 5;
    static const int PAUSE                 = 6;
    static const int CONTINUE              = 7;
    static const int FINISH                = 8;
    static const int GO_HOME               = 9;

public:
  ThreadManager();
  ~ThreadManager();

public:
  void setVehicle(DJI::OSDK::Vehicle* vehicle);
  void setTraceName(std::string traceName);
  std::string getTraces();
  void saveTrace(std::string traceName);

  bool start(int missionType);
  void _pause();
  void _continue();
  void _finish();


// mission 函数
// functions(); 然后在是start中调用，move to p_thread;
private:
  std::string traceName;
  static void* takeOff(void*);
  static void* land(void*);
  static void* goHome(void*);
  static void* goPosition(void*);
  static void* tracking(void*);
  static void* ADRC_tracking(void*);
  static void* startRecord(void*);


public:
  mutex              lock;
  thread             p_thread;
  volatile bool      isLock;
  volatile bool      isFinish;
  FlightController*  f_controller;
  DATA::FlightData      m_currentPosition;
};

#endif // POSIXTHREADMANAGER_H
