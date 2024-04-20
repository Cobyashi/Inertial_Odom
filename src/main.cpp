/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Coby Smith                                       */
/*    Created:      Wed Apr 17 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// inertialSens         inertial      6               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

competition Competition;

const double REFRESH_RATE = 20;

InertialOdom odom(inertialSens, REFRESH_RATE);

void usercontrol()
{

  while(1)
  {
    odom.findDisplacement();
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor( 1, 1);

    Brain.Screen.print("X:\t");
    Brain.Screen.print(odom.tr_Acceleration.x);
    Brain.Screen.newLine();
    Brain.Screen.print("Y:\t");
    Brain.Screen.print(odom.tr_Acceleration.y);
    Brain.Screen.newLine();
    Brain.Screen.print("Z:\t");
    Brain.Screen.print(odom.tr_Acceleration.z);

    wait(REFRESH_RATE, msec);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Competition.drivercontrol(usercontrol);

  while(true)
  {

  }
  
}
