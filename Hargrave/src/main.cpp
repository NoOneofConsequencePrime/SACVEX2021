/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LF                   motor         1               
// LB                   motor         2               
// RB                   motor         3               
// RF                   motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

#define Ct1 Controller1
#define db double

// A global instance of competition
competition Competition;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();


}

void autonomous(void) {}

// Variables
static int threshold = 10, negThreshold = -10;
static float C1, C2, C3, C4;
static int highSpd = 75, lowSpd = 30;// pct
static int spd = highSpd;
static int turnReduc = 2;

void getChannel() {// General deadzone check
  C1 = (Ct1.Axis1.position(pct) >= threshold || Ct1.Axis1.position(pct) <= negThreshold)? Ct1.Axis1.position(pct) : 0;
  C2 = (Ct1.Axis2.position(pct) >= threshold || Ct1.Axis2.position(pct) <= negThreshold)? Ct1.Axis2.position(pct) : 0;
  C3 = (Ct1.Axis3.position(pct) >= threshold || Ct1.Axis3.position(pct) <= negThreshold)? Ct1.Axis3.position(pct) : 0;
  C4 = (Ct1.Axis4.position(pct) >= threshold || Ct1.Axis4.position(pct) <= negThreshold)? Ct1.Axis4.position(pct) : 0;
}

void getSpd() {
  if (Ct1.ButtonA.pressing()) spd = highSpd;
  else if (Ct1.ButtonB.pressing()) spd = lowSpd;
}

void xDrive() {
  LF.setStopping(brake);
  LB.setStopping(brake);
  RF.setStopping(brake);
  RB.setStopping(brake);

  if (Ct1.ButtonUp.pressing()) {
    LF.spin(forward, spd, pct);
    LB.spin(forward, spd, pct);
    RF.spin(forward, spd, pct);
    RB.spin(forward, spd, pct);
  } else if (Ct1.ButtonRight.pressing()) {
    LF.spin(forward, spd, pct);
    LB.spin(reverse, spd, pct);
    RF.spin(reverse, spd, pct);
    RB.spin(forward, spd, pct);
  } else if (Ct1.ButtonLeft.pressing()) {
    LF.spin(reverse, spd, pct);
    LB.spin(forward, spd, pct);
    RF.spin(forward, spd, pct);
    RB.spin(reverse, spd, pct);
  } else if (Ct1.ButtonDown.pressing()) {
    LF.spin(reverse, spd, pct);
    LB.spin(reverse, spd, pct);
    RF.spin(reverse, spd, pct);
    RB.spin(reverse, spd, pct);
  } else if (C1 != 0) {
    LF.spin(forward, C1/turnReduc, pct);
    LB.spin(forward, C1/turnReduc, pct);
    RF.spin(reverse, C1/turnReduc, pct);
    RB.spin(reverse, C1/turnReduc, pct);
  } else {
    LF.stop();
    LB.stop();
    RF.stop();
    RB.stop();
  }
}

void usercontrol(void) {
  while (1) {
    getSpd();
    getChannel();
    xDrive();

    wait(5, msec);
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
