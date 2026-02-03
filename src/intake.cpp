#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

// intake motors
pros::Motor lowerIntake(-3);
pros::Motor upperIntake(-1);

// intake speeds
volatile int lowerIntakeSpeed, upperIntakeSpeed;

// states
volatile bool loading = false;
volatile bool longGoalScoring = false;
volatile bool centerGoalScoring = false;
volatile bool outtaking = false;
volatile bool halted = false;

// anti jam timer
u_int32_t lastCheckTime = pros::millis();
bool intakeJammed = false;

// set intake speed
void setIntake(int lowerSpeed, int upperSpeed) {
  lowerIntakeSpeed = lowerSpeed;
  upperIntakeSpeed = upperSpeed;

  // states
  loading = false;
  longGoalScoring = false;
  centerGoalScoring = false;
  outtaking = false;
  halted = false;

  lastCheckTime = pros::millis();
}

void intakeLoad() {
  // speed
  lowerIntakeSpeed = 127;
  upperIntakeSpeed = -95;

  // states
  if (!loading)
    lastCheckTime = pros::millis();
  loading = true;
  longGoalScoring = false;
  centerGoalScoring = false;
  outtaking = false;
  halted = false;
}

void intakeCenterGoal() {
  // speed
  lowerIntakeSpeed = 110;
  upperIntakeSpeed = 90;

  // states
  if (!centerGoalScoring)
    lastCheckTime = pros::millis();
  loading = false;
  longGoalScoring = false;
  centerGoalScoring = true;
  outtaking = false;
  halted = false;
}

void intakeLongGoal() {
  // speed
  lowerIntakeSpeed = 127;
  upperIntakeSpeed = 127;

  // states
  if (!longGoalScoring)
    lastCheckTime = pros::millis();
  loading = false;
  longGoalScoring = true;
  centerGoalScoring = false;
  outtaking = false;
  halted = false;
}

void intakeOuttake() {
  // speed
  lowerIntakeSpeed = -127;
  upperIntakeSpeed = -127;

  // states
  if (!outtaking)
    lastCheckTime = pros::millis();
  loading = false;
  longGoalScoring = false;
  centerGoalScoring = false;
  outtaking = true;
  halted = false;
}

void intakeHalt() {
  // speed
  lowerIntakeSpeed = 0;
  upperIntakeSpeed = 0;

  // states
  loading = false;
  longGoalScoring = false;
  centerGoalScoring = false;
  outtaking = false;
  halted = true;
}

// intake async control
void asyncIntakeControl(void *param) {
  printf("Intake Task Started\n");
  while (true) {
    // run intake
    lowerIntake.move(lowerIntakeSpeed);
    upperIntake.move(upperIntakeSpeed);

    if (centerGoalScoring) {
      hood.set_value(true);
    }
    if (longGoalScoring) {
      hood.set_value(false);
    }

    // anti jam
    if (pros::millis() - lastCheckTime > 1000) {
      if (!halted) {
        bool jammed = false;
        // check if any motors are jammed while intaking
        if (longGoalScoring || centerGoalScoring) {
          if (fabs(upperIntake.get_actual_velocity()) < 3 ||
              fabs(lowerIntake.get_actual_velocity()) < 3) {
            upperIntake.move(-127);
            lowerIntake.move(-127);
            jammed = true;
          }
        } else if (outtaking) {
          if (fabs(upperIntake.get_actual_velocity()) < 3 ||
              fabs(lowerIntake.get_actual_velocity()) < 3) {
            upperIntake.move(127);
            lowerIntake.move(127);
            jammed = true;
          }
        }

        if (jammed) {
          // delay to let the "pulse" happen
          pros::delay(75);
          // reset timer
          lastCheckTime = pros::millis();
        }
      }
    }

    // delay
    pros::delay(20);
  }
}
