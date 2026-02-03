#include "main.h"
#include "pros/motors.hpp"

//motors
extern pros::Motor lowerIntake;
extern pros::Motor upperIntake;

//functions
extern void setIntake(int lowerIntakeSpeed, int upperIntakeSpeed);
extern void intakeLoad();
extern void intakeLongGoal();
extern void intakeCenterGoal();
extern void intakeOuttake();
extern void intakeHalt();

extern void asyncIntakeControl (void * param);

extern bool loading;
extern bool longGoalScoring;
extern bool centerGoalScoring;
extern bool outtaking;
extern bool halted;
