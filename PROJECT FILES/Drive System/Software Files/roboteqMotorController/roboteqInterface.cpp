#include "roboteqInterface.h"


//Initialize
bool roboteqCommunication::initialize(std::string port) {
  int status;
  status = device.Connect(port);

  if(status != RQ_SUCCESS) {
    std::cout << "Error connecting to the motor controller: " << status << std::endl;
    return false;
  }

  return true;
}

//Retrieve absolute encoder counts
bool roboteqCommunication::readAbsoluteEncoderCounts(int &counts)
{
  if(device.GetValue(_ABCNTR,counts) != RQ_SUCCESS) {
    std::cout << "Failed to read encoder!!!" << std::endl;
    return false;
  }
  usleep(10);

  std::cout << "Absolute Encoder ch1:" << counts << endl;
  return true;
}
