#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

class roboteqCommunication{
public:

bool initialize(std::string port);

bool readAbsoluteEncoderCounts(int &counts);

private:
RoboteqDevice device;

}
