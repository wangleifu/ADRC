#include "mission_manager.hpp"
#include "mobile_callback.hpp"
#include <dji_linux_helpers.hpp>

#include <string>

using namespace DJI::OSDK;

void sendTraceName(DJI::OSDK::Vehicle* vehiclePtr);
bool setupMSDKParsing(Vehicle* vehicle, LinuxSetup* linuxEnvironment);
void parseFromMobileCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData);
void parseFromMobileCallback1(Vehicle* vehicle, RecvContainer recvFrame, UserData userData);

ThreadManager threadManager;

// 程序入口函数
int main1(int argc, char *argv[])
{
    //DJI::OSDK::Log::instance().disableStatusLogging();
    //DJI::OSDK::Log::instance().enableDebugLogging();

    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle*   vehicle = linuxEnvironment.getVehicle();
    if (vehicle == NULL)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }

    threadManager.setVehicle(vehicle);
    setupMSDKParsing(vehicle, &linuxEnvironment);

    //while(true);
    int a;
    cin >> a;
    return 0;
}





// 设置mbile的监听器
bool setupMSDKParsing(Vehicle* vehicle, LinuxSetup* linuxEnvironment)
{
    DSTATUS("--------setup MSDK Callback function--------\n");
    vehicle->moc->setFromMSDKCallback(parseFromMobileCallback, linuxEnvironment);
//    //sendMessage(vehicle);
//    sendTraceName(vehicle);
//    std::cout << "send message" << std::endl;
    return true;
}

// mobile 通信内容解析 与 功能执行
void parseFromMobileCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
    DSTATUS("--------parse mobile singal--------\n");

    string message  = (std::string)reinterpret_cast<char*>(&recvFrame.recvData.raw_ack_array);
    std::cout  << "get message " << message << std::endl;

    int cmdIndex = message.find_first_of('-');
    cout << "length: " << message.length() << endl;
    cout << "cmd index: " << cmdIndex << endl;
    int cmd = std::atoi(message.substr(0, cmdIndex).c_str());


    int msgIndex = (message.substr(cmdIndex+1)).find_first_of('-');
    cout << "msg index: " << msgIndex << endl;
    string msg = "";
    if (msgIndex > 0)
    {
        msg = message.substr(cmdIndex+1, msgIndex);
    }
    cout << "cmd: " << cmd << endl;
    cout << "msg: " << msg << endl;

    switch(cmd)
    {
    case 0:
        cout << "cmd 0: send traces" << endl;
        cout << "msg: " << msg << endl;

        sendTraceName(vehicle);
        break;
    case 1:
        cout << "cmd 1: obtainCtrlAuthority" << endl;
        cout << "msg: " << msg << endl;

        vehicle->obtainCtrlAuthority(controlAuthorityMobileCallback);
        break;
    case 2:
        cout << "cmd 2: releaseCtrlAuthority" << endl;
        cout << "msg: " << msg << endl;

        vehicle->releaseCtrlAuthority(controlAuthorityMobileCallback);
        break;
    case 3:
        cout << "cmd 3: start recording" << endl;
        cout << "msg: " << msg << endl;

        threadManager.start(ThreadManager::START_RECORD);
        break;
    case 4:
        cout << "cmd 4: end recording" << endl;
        cout << "msg: " << msg << endl;
        threadManager.saveTrace(msg);

        break;
    case 5:
        cout << "cmd 5: tracking" << endl;
        cout << "msg: " << msg << endl;

        if (!msg.empty())
        {
            threadManager.setTraceName(msg);
            threadManager.start(ThreadManager::TRACKING);
        }
        sendAckToMobile(vehicle, cmd);
        break;
    case 6:
        cout << "cmd 6: pause" << endl;
        cout << "msg: " << msg << endl;

        threadManager._pause();
        sendAckToMobile(vehicle, cmd);
        break;
    case 7:
        cout << "cmd 7: continue" << endl;
        cout << "msg: " << msg << endl;

        threadManager._continue();
        sendAckToMobile(vehicle, cmd);
        break;
    case 8:
        cout << "cmd 8: terminate" << endl;
        cout << "msg: " << msg << endl;

        threadManager._finish();
        sendAckToMobile(vehicle, cmd);
        break;
    case 9:
        cout << "cmd 9: goHome" << endl;
        cout << "msg: " << msg << endl;

        vehicle->control->goHome(actionMobileCallback);
        break;
    default:
        break;
    }

    DSTATUS("----------------over----------------\n");
}

// mobile 通信内容解析 与 功能执行
void parseFromMobileCallback1(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
    DSTATUS("--------parse mobile singal--------\n");
    uint16_t mobile_data_id;
    mobile_data_id = *(reinterpret_cast<uint16_t*>(&recvFrame.recvData.raw_ack_array));

    char* c = reinterpret_cast<char*>(&recvFrame.recvData.raw_ack_array);

    int len = strlen(c);
    std::string message = (std::string) c;

    int index = message.find('-');
    index = index > 0 ? index : len;
    cout << "len: " << len << endl;
    cout << "index: " << index << endl;

    string msg = message.substr(0, index);
    if (index < len)
    {
        string id = message.substr(index+1, len);
        cout << "msg: " << msg << endl;
        cout << "cmd: " << id << endl;
    }


    std::cout  << "get message " << message << std::endl;

    std::cout << "RecvContainer Info: " << std::endl;

    std::cout << "  DJI::OSDK::ACK::Entry     recvInfo: " << std::endl;
    std::cout << "     cmd_set: "
              << recvFrame.recvInfo.cmd_set << std::endl;
    std::cout << "     cmd_id: "
              << recvFrame.recvInfo.cmd_id << std::endl;
    std::cout << "     len: "
              << recvFrame.recvInfo.len << std::endl;
    std::cout << "     buf: "
              << recvFrame.recvInfo.buf << std::endl;
    std::cout << "     seqNumber: "
              << recvFrame.recvInfo.seqNumber << std::endl;
    std::cout << "     version: "
              << recvFrame.recvInfo.version << std::endl;

    std::cout << "  DJI::OSDK::ACK::TypeUnion recvData: " << std::endl;
    std::cout << "     raw_ack_array[MAX_INCOMING_DATA_SIZE]: "
              << recvFrame.recvData.raw_ack_array << std::endl;
    std::cout << "     raw_ack_array[MAX_INCOMING_DATA_SIZE]: "
              << c << std::endl;
    std::cout << "     versionACK[MAX_ACK_SIZE]: "
              << recvFrame.recvData.versionACK << std::endl;
    std::cout << "     ack: "
              << recvFrame.recvData.ack << std::endl;
    std::cout << "     commandACK: "
              << recvFrame.recvData.commandACK << std::endl;
    std::cout << "     missionACK: "
              << recvFrame.recvData.missionACK << std::endl;
    std::cout << "     subscribeACK: "
              << recvFrame.recvData.subscribeACK << std::endl;
    std::cout << "     mfioACK: "
              << recvFrame.recvData.mfioACK << std::endl;


    std::cout << "  DJI::OSDK::DispatchInfo   dispatchInfo:" << std::endl;
    std::cout << "     isAck: "
              << recvFrame.dispatchInfo.isAck << std::endl;
    std::cout << "     isCallback: "
              << recvFrame.dispatchInfo.isCallback << std::endl;
    std::cout << "     callbackID: "
              << recvFrame.dispatchInfo.callbackID << std::endl;

    switch(mobile_data_id)
    {
    case ThreadManager::SET_HOME_POINT:
        DDEBUG("case 0: setHomePoint.");
        //threadManager.setHomePoint();
        break;
    case ThreadManager::OBTAIN_CTR_AUTHORITY:
        DDEBUG("case 1: obtainCtrAuthority.");
        vehicle->obtainCtrlAuthority(controlAuthorityMobileCallback);
        break;
    case ThreadManager::RELEASE_CTR_AUTHORITY:
        DDEBUG("case 2: releaseCtrAuthority.");
        vehicle->releaseCtrlAuthority(controlAuthorityMobileCallback);
        break;
    case ThreadManager::START_RECORD:
        DDEBUG("case 3: takeOff");
        vehicle->control->takeoff(actionMobileCallback);
        break;
    case ThreadManager::END_RECORD:
        DDEBUG("case 4: land");
        vehicle->control->land(actionMobileCallback);
        break;
    case ThreadManager::TRACKING:
        DDEBUG("case 5: missionStart");
        threadManager.start(ThreadManager::TRACKING);
        sendAckToMobile(vehicle, mobile_data_id);
        break;
    case ThreadManager::PAUSE:
        DDEBUG("case 6: pause.");
        threadManager._pause();
        sendAckToMobile(vehicle, mobile_data_id);
        break;
    case ThreadManager::CONTINUE:
        DDEBUG("case 7: continue.");
        threadManager._continue();
        sendAckToMobile(vehicle, mobile_data_id);
        break;
    case ThreadManager::FINISH:
        DDEBUG("case 8: terminate.");
        threadManager._finish();
        sendAckToMobile(vehicle, mobile_data_id);
        break;
    case ThreadManager::GO_HOME:
        DDEBUG("case 9: goHome.");
        //threadManager.start(GO_HOME);
        vehicle->control->goHome(actionMobileCallback);
        break;
    case 10:
        sendTraceName(vehicle);
        break;
    default:
        break;
    }
    DSTATUS("----------------over----------------");
}


void
sendTraceName(Vehicle* vehiclePtr)
{
    // We will get the trace names from local DB
  std::string traces = threadManager.getTraces();
  if (traces.length() > 100)
  {
      DERROR("Data is too large for Onboard transfor to Mobile.");
  }
  char     tempRawString[100] = { 0 };
  memcpy(&tempRawString, traces.c_str(), traces.length());
  // Now pack this uo into a mobile packet
  TraceNamePacket tracesPack(&tempRawString[0]);

  // And send it
  vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&tracesPack),
                                  sizeof(tracesPack));
}
