
#include"i2c.h"
#include"i2c.h"
#include <iostream>
#include<math.h>


using namespace std;
#define Task_t 10          //s3 Task Time in milli seconds
// Main code -----------------------------------------------------------------
int main(){
#if 0
  compass_x_offset = 242.18;
  compass_y_offset = 207.18;
  compass_z_offset = 206.38;
  
  compass_x_gainError = 1.03;
  compass_y_gainError = 1.11;
  compass_z_gainError = 1.03;
#endif   
  
  compass_init(2);
  //compass_debug = 1;
  
  compass_offset_calibration(3);

while(1){
  
  
  compass_scalled_reading(compass_address);
  
  //Serial.print("x = ");
  //Serial.println(compass_x_scalled);
  //Serial.print("y = ");
  //Serial.println(compass_y_scalled);
  //Serial.print("z = ");
  //Serial.println(compass_z_scalled);
  

  compass_heading(compass_address);
  cout <<"Heading angle = "; cout << bearing; cout<<" Degree" <<endl;
  usleep(500);
  
  
}

return 0;
}









