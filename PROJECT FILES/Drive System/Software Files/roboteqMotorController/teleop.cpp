//Original code developed by Roboteq, Inc. 
//8426 E. Shea Blvd., Scottsdale AZ 85260 USA
//
//Modifications made using Willow Garage's ROS tutorials package, turtlesim stack,
//teleop_turtle_key.cpp file are to provide keyboard manipulation functions to 
//the Original code.
//
// By Pareshkumar Brahmbhatt,  Drexel University, Drexel Autonomous Systems Lab
// This code was originally found on the public domain and the originals 
//can be found on their respective organization sites.
 
 
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
 
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"
 
 
#include <signal.h>
#include <termios.h>
#include <stdio.h>
 
#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_S 0x73
#define KEYCODE_E 0x65
 
 
using namespace std;
 
 
void keyLoop();
int kfd = 0;
struct termios cooked, raw;
RoboteqDevice device;
 
 
int main(int argc, char *argv[])
{
    string response = "";
     
    int status = device.Connect("/dev/ttyACM0");
 
    if(status != RQ_SUCCESS)
    {
        cout<<"Error connecting to device: \n\r"<<status<<"."<<endl;
        return 1;
    }
 
    cout<<"- SetConfig(_DINA, 1, 1)...\n\r";
    if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
        cout<<"failed --> "<<status<<endl;
    else
        cout<<"succeeded.\n\r"<<endl;
 
    //Wait 10 ms before sending another command to device
    sleepms(10);
 
    int result;
    cout<<"- GetConfig(_DINA, 1)...\n\r";
    if((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS)
        cout<<"failed --> "<<status<<endl;
    else
        cout<<"returned --> \n\r"<<result<<endl;
 
    //Wait 10 ms before sending another command to device
    sleepms(10);
 
    cout<<"- GetValue(_ANAIN, 1)...\n\r";
    if((status = device.GetValue(_ANAIN, 1, result)) != RQ_SUCCESS)
        cout<<"failed --> "<<status<<endl;
    else
        cout<<"returned --> "<<result<<"\n\r"<<endl;
 
    //Wait 10 ms before sending another command to device
    sleepms(10);
 
 
/*   
 
//Original Roboteq API code to simply move motor 1 , at value 1
 
    cout<<"- SetCommand(_GO, 1, 1)...";
    if((status = device.SetCommand(_GO, 1, 1)) != RQ_SUCCESS)
        cout<<"failed --> "<<status<<endl;
    else
        cout<<"succeeded."<<endl;
*/
 
 
    keyLoop();
 
    device.Disconnect();
    return 0;
}
 
 
 
//The function below allows the use of keyboard input
//to manipulate motor movement in a forward-reverse on-off fashion
//copied from teleop_turtle_key.cpp
 
void keyLoop()
{
  char c;
  bool dirty=false;
 
 
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
 
  puts("Reading from keyboard\n\r");
  puts("---------------------------\n\r");
  puts("-----------Q for EXIT----------------\n\r");
  puts("Use arrow keys to move the turtle.\n\r");
 
 
  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      break;
    }
 
     
 
   
    switch(c)
    {
      case KEYCODE_L:
        cout<<"LEFT\r";
    //When the left arrow is pressed, motor 1 is set to move
    //an increment of value 150 and motor 2 is set to move
    //an increment of value -150 
        device.SetCommand(_GO, 1, -150);
	device.SetCommand(_GO, 2, 150);
        dirty = true;
        break;
      case KEYCODE_R:
        cout<<"RIGHT\r";
    //When the right arrow is pressed, motor 1 is set to move
    //an increment of value -150 and motor 2 is set to move
    //an increment of value 150 
        device.SetCommand(_GO, 1, 150);
        device.SetCommand(_GO, 2, -150);
        dirty = true;
        break;
      case KEYCODE_U:
        cout<<"UP\r";
    //When up arrow is pressed, both motors are set to move a
    // single increment of value 150
	device.SetCommand(_GO, 1, 150);
        device.SetCommand(_GO, 2, 150);
        dirty = true;
        break;
      case KEYCODE_D:
        cout<<"DOWN\r";
    //When down arrow is pressed, both motors are set to move a
    // single increment of value -150
        device.SetCommand(_GO, 1, -150);
	device.SetCommand(_GO, 2, -150);
        dirty = true;
        break;
       case KEYCODE_S:
        cout<<"STOP\r";
    //When the "s" key is pressed, both motors stop
        device.SetCommand(_GO, 1, 0);
	device.SetCommand(_GO, 2, 0);
        dirty = true;
        break;
       case KEYCODE_E:
        cout<<"EMERGENCY STOP\r";
    //When the "s" key is pressed, both motors stop
        device.SetCommand(_ESTOP);
        dirty = true;
        break;

      case KEYCODE_Q:
    cout<<endl<<"You have Quit interacting with the Roboteq Motor Controller\r"<<endl;
    dirty = true;
    device.Disconnect();
    return;
     
    }
    
 
 
    if(dirty ==true)
    {
           
      dirty=false;
    }
  }
 
 
 
}
