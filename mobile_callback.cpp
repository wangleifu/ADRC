#include "mobile_callback.hpp"

using namespace DJI::OSDK;

// mobile callback functions
void
controlAuthorityMobileCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                               UserData userData)
{
  ACK::ErrorCode ack;
  ack.data = OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;

  unsigned char data    = 0x1;
  int           cbIndex = vehiclePtr->callbackIdIndex();

  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= sizeof(uint16_t))
  {
    ack.data = recvFrame.recvData.ack;
    ack.info = recvFrame.recvInfo;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }

  if (ACK::getError(ack))
  {
    if (ack.data ==
        OpenProtocolCMD::ControlACK::SetControl::OBTAIN_CONTROL_IN_PROGRESS)
    {
      ACK::getErrorCodeMessage(ack, __func__);
      vehiclePtr->obtainCtrlAuthority(controlAuthorityMobileCallback);
    }
    else if (ack.data == OpenProtocolCMD::ControlACK::SetControl::
                           RELEASE_CONTROL_IN_PROGRESS)
    {
      ACK::getErrorCodeMessage(ack, __func__);
      vehiclePtr->releaseCtrlAuthority(controlAuthorityMobileCallback);
    }
    else
    {
      ACK::getErrorCodeMessage(ack, __func__);
    }
  }
  else
  {
    // We have a success case.
    // Send this data to mobile
    AckReturnToMobile mobileAck;
    // Find out which was called: obtain or release
    if (recvFrame.recvInfo.buf[2] == ACK::OBTAIN_CONTROL)
    {
      mobileAck.cmdID = 0x02;
    }
    else if (recvFrame.recvInfo.buf[2] == ACK::RELEASE_CONTROL)
    {
      mobileAck.cmdID = 0x03;
    }
    mobileAck.ack = static_cast<uint16_t>(ack.data);
    vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                    sizeof(mobileAck));
    // Display ACK message
    ACK::getErrorCodeMessage(ack, __func__);
  }
}

void
actionMobileCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                     UserData userData)
{
  ACK::ErrorCode ack;

  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= sizeof(uint16_t))
  {

    ack.info = recvFrame.recvInfo;

    if (vehiclePtr->isLegacyM600())
      ack.data = recvFrame.recvData.ack;
    else if (vehiclePtr->isM100())
      ack.data = recvFrame.recvData.ack;
    else
      ack.data = recvFrame.recvData.commandACK;

    // Display ACK message
    ACK::getErrorCodeMessage(ack, __func__);

    AckReturnToMobile mobileAck;
    const uint8_t     cmd[] = { recvFrame.recvInfo.cmd_set,
                            recvFrame.recvInfo.cmd_id };

    // startMotor supported in FW version >= 3.3
    // setArm supported only on Matrice 100 && M600 old FW
    if (vehiclePtr->isM100() || vehiclePtr->isLegacyM600())
    {
      if ((memcmp(cmd, OpenProtocolCMD::CMDSet::Control::setArm, sizeof(cmd)) &&
           recvFrame.recvInfo.buf[2] == true))
      {
        mobileAck.cmdID = 0x05;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
      else if ((memcmp(cmd, OpenProtocolCMD::CMDSet::Control::setArm,
                       sizeof(cmd)) &&
                recvFrame.recvInfo.buf[2] == false))
      {
        mobileAck.cmdID = 0x06;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] ==
               Control::FlightCommand::LegacyCMD::takeOff)
      {
        mobileAck.cmdID = 0x07;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] ==
               Control::FlightCommand::LegacyCMD::landing)
      {
        mobileAck.cmdID = 0x08;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] ==
               Control::FlightCommand::LegacyCMD::goHome)
      {
        mobileAck.cmdID = 0x09;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
    }
    else // Newer firmware
    {
      if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::startMotor)
      {
        mobileAck.cmdID = 0x05;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::stopMotor)
      {
        mobileAck.cmdID = 0x06;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::takeOff)
      {
        mobileAck.cmdID = 0x07;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::landing)
      {
        mobileAck.cmdID = 0x08;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::goHome)
      {
        mobileAck.cmdID = 0x09;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                        sizeof(mobileAck));
      }
    }
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }
}

// Utility functions
void
sendAckToMobile(DJI::OSDK::Vehicle* vehiclePtr, uint16_t cmdID, uint16_t ack)
{
    // Generate a local ACK to send the ACK back to mobile
    AckReturnToMobile mobileAck;
    mobileAck.cmdID = cmdID;
    mobileAck.ack   = ack;
    vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                    sizeof(mobileAck));
}

void
sendMessage(DJI::OSDK::Vehicle* vehiclePtr)
{
  // We will reformat the cached drone version and send it back.
    std::string versionString     = "trace1,trace2,trace3";
  char        tempRawString[100] = { 0 };
  memcpy(&tempRawString, versionString.c_str(), versionString.length());
  // Now pack this up into a mobile packet
  TraceNamePacket versionPack(&tempRawString[0]);

  // And send it
  vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&versionPack),
                                  sizeof(versionPack));
}

TraceNamePacket::TraceNamePacket(char* traces)
  : TraceNames{ 0 }
{
  memcpy(this->TraceNames, traces, sizeof(TraceNames) / sizeof(TraceNames[0]));
}

