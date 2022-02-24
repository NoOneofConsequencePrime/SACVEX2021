using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LB;
extern motor LT;
extern motor RB;
extern motor RT;
extern motor roller;
extern motor RArm;
extern motor LArm;
extern motor Hook;
extern inertial Gyro;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );