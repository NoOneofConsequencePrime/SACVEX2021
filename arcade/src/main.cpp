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
// LB                   motor         11              
// LT                   motor         2               
// RB                   motor         3               
// RT                   motor         4               
// roller               motor         7               
// RArm                 motor         9               
// LArm                 motor         5               
// Hook                 motor         8               
// Gyro                 inertial      20              
// ---- END VEXCODE CONFIGURED DEVICES ----

/*
LB is left bottom motor
LT is left top motor
RB is right bottom motor
RT is right top motor
Fried Ports: 2, 6, 10
*/

#include "vex.h"
#include <cmath>
#include <string>

using namespace vex;

// Program init
#define print Brain.Screen.print
#define Ct1 Controller1
#define db double

competition Competition;

// Program Parameters
static const double PI = 3.1415926;

void pre_auton(void) {
  vexcodeInit();
  // Robot Init
  // Gyro.startCalibration(2000);
  // wait(200, msec);
  // while (Gyro.isCalibrating()) {
  //   wait(10, msec);
  //   task::sleep(10);
  // }
}

void debug() {
  Brain.Screen.clearScreen();
  // print("LArm: ");
  // print(LArm.position(degrees));
  // Brain.Screen.setCursor(2, 1);
  // print("RArm: ");
  // print(RArm.position(degrees));
  
  Brain.Screen.setCursor(1, 1);
  print("LT: ");
  print(LT.position(turns));
  Brain.Screen.setCursor(2, 1);
  print("LB: ");
  print(LB.position(turns));
  Brain.Screen.setCursor(3, 1);
  print("RT: ");
  print(RT.position(turns));
  Brain.Screen.setCursor(4, 1);
  print("RB: ");
  print(RB.position(turns));
  Brain.Screen.setCursor(5, 1);
  print("Robot rotation: ");
  print(Gyro.rotation(deg));
  
  /*
  Forward:
  Left side - negative
  Right side - positive
  Backward:
  Left side - positive
  Right side - negative
  */

  // print("Robot Heading: ");
  // print(Gyro.heading(degrees));
  // Brain.Screen.setCursor(2, 1);
  // print("Robot Rotation: ");
  // print(Gyro.rotation(degrees));

  // print("Hook Degs: ");
  // print(Hook.position(degrees));
}

void moveForward(double dist, int spd) {// (cm, pct)
  // Settings
  LT.setStopping(brake);
  LB.setStopping(brake);
  RT.setStopping(brake);
  RB.setStopping(brake);

  // Data
  struct st {
    double lt, lb;// neg = forward
    double rt, rb;// pos = forward
  };
  st curWheels = {LT.position(turns), LB.position(turns), RT.position(turns), RB.position(turns)};
  double rotNeed = dist/(10.16*PI);

  if (dist > 0) {// forward
    rotNeed -= std::min(rotNeed, ((db)spd*spd/1000 + 3)/(10.16*PI));
    RT.setVelocity(spd, pct);
    LB.setVelocity(-spd, pct);
    RB.setVelocity(spd, pct);
    LT.setVelocity(-spd, pct);
  } else if (dist < 0) {// backward
    rotNeed += std::max(rotNeed, ((db)spd*spd/1000 + 3)/(10.16*PI));
    RT.setVelocity(-spd, pct);
    LB.setVelocity(spd, pct);
    RB.setVelocity(-spd, pct);
    LT.setVelocity(spd, pct);
  }

  RT.spinToPosition(curWheels.rt+rotNeed, turns);
  LB.spinToPosition(curWheels.lb-rotNeed, turns);
  RB.spinToPosition(curWheels.rb+rotNeed, turns);
  LT.spinToPosition(curWheels.lt-rotNeed, turns);

  // while (RT.position(turns) < curWheels.rt+rotNeed || RB.position(turns) < curWheels.rb+rotNeed || LT.position(turns) > curWheels.lt-rotNeed || LB.position(turns) > curWheels.lb-rotNeed) {
  // }
  // LT.stop();
  // RT.stop();
  // LB.stop();
  // RB.stop();
}

void rotateTowards(double rot, int spd) {
  // Settings
  LT.setStopping(brake);
  LB.setStopping(brake);
  RT.setStopping(brake);
  RB.setStopping(brake);

  // Data
  double initDeg = Gyro.rotation(deg);

  if (rot > 0) {
    // Adjustment
    rot = std::max(0.0, rot - ((db)spd*spd/250 + rot/15));
    RT.setVelocity(-spd, pct);
    LB.setVelocity(-spd, pct);
    RB.setVelocity(-spd, pct);
    LT.setVelocity(-spd, pct);
    RT.spin(forward);
    LB.spin(forward);
    RB.spin(forward);
    LT.spin(forward);
    while (Gyro.rotation(deg) < initDeg+rot) {
      wait(5, msec);
    }
    LT.stop();
    RT.stop();
    LB.stop();
    RB.stop();
  } else if (rot < 0) {
    RT.setVelocity(-spd, pct);
    LB.setVelocity(-spd, pct);
    RB.setVelocity(-spd, pct);
    LT.setVelocity(-spd, pct);
  }
}

void autonomous(void) {
  // moveForward(-100, 30);
  rotateTowards(180, 30);
  // rotateTowards(-90, 30);
  debug();
}

// Global Variables
static int threshold = 10, negThreshold = -10;
static float Channel1, Channel2, Channel3, Channel4;

void getChannel() {// General deadzone check
  Channel1 = (Ct1.Axis1.position(pct) >= threshold || Ct1.Axis1.position(pct) <= negThreshold)? Ct1.Axis1.position(pct) : 0;
  Channel2 = (Ct1.Axis2.position(pct) >= threshold || Ct1.Axis2.position(pct) <= negThreshold)? Ct1.Axis2.position(pct) : 0;
  Channel3 = (Ct1.Axis3.position(pct) >= threshold || Ct1.Axis3.position(pct) <= negThreshold)? Ct1.Axis3.position(pct) : 0;
  Channel4 = (Ct1.Axis4.position(pct) >= threshold || Ct1.Axis4.position(pct) <= negThreshold)? Ct1.Axis4.position(pct) : 0;
}

void movL(float lVal) {
  LB.spin(reverse, lVal, pct);
  LT.spin(reverse, lVal, pct);
}

void movR(float rVal) {
  RB.spin(forward, rVal, pct);
  RT.spin(forward, rVal, pct);
}

void tankDrive(){ //Drivetrain (W.I.P.)
  if (Channel3 != 0) {
    LB.spin(reverse, Channel3, percent);
    LT.spin(reverse, Channel3, percent);
  } else {
    LB.stop();
    LT.stop();
  }
  if (Channel2 != 0) {
    RB.spin(forward, Channel2, percent);
    RT.spin(forward, Channel2, percent);
  } else {
    RB.stop();
    RT.stop();
  }
}

void arcadeDrive(float joyX, float joyY) {
  // Settings
  // LT.setStopping(coast);
  // LB.setStopping(coast);
  // RT.setStopping(coast);
  // RB.setStopping(coast);

  // Base variables
  float motMixL, motMixR;
  float pivYLim = 100.0;
  // TEMP
  float motPremixL, motPremixR, pivScale;
  float pivSpd;

  // Base drive turn
  if (joyY >= 0) {// Forward
    motPremixL = (joyX >= 0) ? 100.0 : (100.0 + joyX);
    motPremixR = (joyX >= 0) ? (100.0 - joyX) : 100.0;
  } else { // Reverse
    motPremixL = (joyX >= 0) ? (100.0 - joyX) : 100.0;
    motPremixR = (joyX >= 0) ? 100.0 : (100.0 + joyX);
  }

  // First scale
  motPremixL *= (joyY/100.0);
  motPremixR *= (joyY/100.0);

  // Get pivot amount
  pivSpd = joyX/2;
  pivScale = (std::abs(joyY) > pivYLim) ? 0.0 : (1.0 - std::abs(joyY)/pivYLim);

  // Calculate final
  motMixL = (1.0-pivScale)*motPremixL + pivScale*pivSpd;
  motMixR = (1.0-pivScale)*motPremixR + pivScale*(-pivSpd);

  // Move
  movL(motMixL);
  movR(motMixR);
}

void rollerSpin(){ //Spin roller + intake
  if (Ct1.ButtonL1.pressing()){
    roller.spin(forward, 70, percent);
  } else if (Ct1.ButtonL2.pressing()){
    roller.spin(reverse, 70, percent);
  } else {
    roller.stop();
  }
}

void moveArm() { //Arm up + down
  // LArm raise = negative
  // RArm raise = positive

  // Guided Manual
  if (Ct1.ButtonR2.pressing() && RArm.position(degrees) > -15 && LArm.position(degrees) < 15) {// lower
    RArm.spin(reverse, 35, pct);
    LArm.spin(forward, 35, pct);
  } else if (Ct1.ButtonR1.pressing() && RArm.position(degrees) < 260 && LArm.position(degrees) > -260) {// raise
    RArm.spin(forward, 35, pct);
    LArm.spin(reverse, 35, pct);
  } else {
    RArm.setStopping(coast);
    LArm.setStopping(coast);
    RArm.stop();
    LArm.stop();
  }

  // RArm.setStopping(coast);
  // LArm.setStopping(coast);
  // if (Ct1.ButtonR1.pressing()){
  //   RArm.spin(forward, 35, percent);
  //   LArm.spin(reverse, 35, percent);
  // } else if (Ct1.ButtonR2.pressing()){
  //   RArm.spin(reverse, 35, percent);
  //   LArm.spin(forward, 35, percent);
  // } else {
  //   RArm.stop();
  //   LArm.stop();
  // }
}

void moveHook() { //Hook up + down (300 deg <-> 1100 deg)
  // Guided Manual
  if (Ct1.ButtonA.pressing() && Hook.position(degrees) > 300) {// lower
    Hook.setVelocity(100, pct);
    Hook.spin(reverse);
  } else if (Ct1.ButtonB.pressing() && Hook.position(degrees) < 1100) {// raise
    Hook.setVelocity(100, pct);
    Hook.spin(forward);
  } else {
    Hook.setStopping(hold);
    Hook.stop();
  }
  // // Auto
  // if (Ct1.ButtonA.pressing()) {
  //   Hook.setVelocity(100, pct);
  //   if (Hook.position(degrees) > 750) {// closer to down (go up)
  //     Hook.spinToPosition(300, degrees);
  //   } else {
  //     Hook.spinToPosition(1100, degrees);
  //   }
  // } else {
  //   Hook.stop();
  // }
}

void robotTurn(float turnVal) {
  if (turnVal > 0) {
    
  } else if (turnVal < 0) {

  }
}

void usercontrol(void) {
  while (true) {
    getChannel();
    rollerSpin();
    moveArm();
    moveHook();
    // tankDrive();
    // Bruce Drive
    arcadeDrive(Channel1, Channel3);
    // // Normal Arcade Drive
    // arcadeDrive(Channel4, Channel3);
    debug();
    wait(5, msec); // Sleep the task for a short amount of time to prevent wasted resources
  }
}

// Main will set up the competition functions and callbacks.

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

/*
* Junk
*/