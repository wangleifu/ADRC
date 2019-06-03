#ifndef MOBILE_CALLBACK_HPP
#define MOBILE_CALLBACK_HPP

// DJI OSDK includes
#include <dji_vehicle.hpp>

typedef struct AckReturnToMobile
{
  uint16_t cmdID;
  uint16_t ack;
} AckReturnToMobile;

typedef struct TraceNamePacket
{
    char  TraceNames[100];

  TraceNamePacket(char* traceNames);
} TraceNamePacket;

// mobile callback functions
void controlAuthorityMobileCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                    DJI::OSDK::RecvContainer recvFrame,
                                    DJI::OSDK::UserData      userData);
void actionMobileCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                          DJI::OSDK::RecvContainer recvFrame,
                          DJI::OSDK::UserData      userData);

// Utility functions
void sendAckToMobile(DJI::OSDK::Vehicle* vehicle, uint16_t cmdID,
                     uint16_t ack = 0);
void sendMessage(DJI::OSDK::Vehicle* vehiclePtr);

#endif // MOBILE_CALLBACK_HPP
