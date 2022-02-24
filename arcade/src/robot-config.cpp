#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LB = motor(PORT11, ratio18_1, false);
motor LT = motor(PORT2, ratio18_1, true);
motor RB = motor(PORT3, ratio18_1, false);
motor RT = motor(PORT4, ratio18_1, true);
motor roller = motor(PORT7, ratio18_1, false);
motor RArm = motor(PORT9, ratio18_1, false);
motor LArm = motor(PORT5, ratio18_1, false);
motor Hook = motor(PORT8, ratio18_1, false);
inertial Gyro = inertial(PORT20);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1UpDownButtonsControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // check the ButtonUp/ButtonDown status to control LB
      if (Controller1.ButtonUp.pressing()) {
        LB.spin(forward);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonDown.pressing()) {
        LB.spin(reverse);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (!Controller1UpDownButtonsControlMotorsStopped) {
        LB.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1UpDownButtonsControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}