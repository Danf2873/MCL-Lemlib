#include "main.h"
#include "intake.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "mcl.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-12, -10, 9}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({15, 8, -18}, pros::MotorGearset::blue);

// Inertial Sensor on port 14
pros::Imu imu(14);

// Distance sensors for MCL
pros::Distance distFront(11); // Front facing sensor
pros::Distance distRight(12); // Right facing sensor

// Sensor physical offsets from robot center
// Coordinate system: X+ = Right, Y+ = Forward, Theta: 0° = Forward
// Adjust these values to match your physical robot!
lemlib::Pose distFrontOffset(0, 6.0,
                             0); // 6" forward from center, facing forward (0°)
lemlib::Pose distRightOffset(5.0, 0,
                             90); // 5" right from center, facing right (90°)

// MCL Sensors
mcl::MCLSensors mclSensors = {
    {&distFront, &distRight},          // sensors
    {distFrontOffset, distRightOffset} // offsets
};

// drivetrain settings
lemlib::Drivetrain
    drivetrain(&leftMotors,                // left motor group
               &rightMotors,               // right motor group
               10,                         // 10 inch track width
               lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
               450,                        // drivetrain rpm
               2                           // horizontal drift
    );

lemlib::ControllerSettings
    lateral_controller(5.8, // proportional gain (kP)
                       0,   // integral gain (kI)
                       5,   // derivative gain (kD)
                       0,   // anti windup
                       3,   // small error range, in inches
                       250, // small error range timeout, in milliseconds
                       5,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

lemlib::ControllerSettings
    angularController(3.16, // proportional gain (kP)
                      0,    // integral gain (kI)
                      20,   // derivative gain (kD)
                      5,    // anti windup
                      1,    // small error range, in degrees
                      100,  // small error range timeout, in milliseconds
                      5,    // large error range, in degrees
                      500,  // large error range timeout, in milliseconds
                      0     // maximum acceleration (slew)
    );

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2
                            &imu     // inertial sensor
);

// input curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// create the chassis
mcl::MCLChassis chassis(drivetrain, lateral_controller, angularController,
                        sensors, mclSensors, &throttleCurve, &steerCurve);

void cdrift(float lV, float rV, int timeout, bool cst) {
  if (cst) {
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  } else {
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
  }
  leftMotors.move(lV);
  rightMotors.move(rV);
  pros::delay(timeout);
  leftMotors.brake();
  rightMotors.brake();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
  // intake control
  pros::Task intakeTask(asyncIntakeControl);

  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors and start MCL

  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      lemlib::Pose chassisPose = chassis.getPose();
      lemlib::Pose mclPose = chassis.getMCLPose();
      pros::lcd::print(0, "Chassis X: %.2f Y: %.2f T: %.2f", chassisPose.x,
                       chassisPose.y, chassisPose.theta);
      pros::lcd::print(1, "MCL X: %.2f Y: %.2f T: %.2f", mclPose.x, mclPose.y,
                       mclPose.theta);
      pros::lcd::print(2, "Conf: %.2f%% F: %d R: %d",
                       chassis.getMCLConfidence() * 100.0,
                       distFront.get_distance(), distRight.get_distance());
      pros::delay(5);
    }
  });
}

void disabled() {}

void competition_initialize() {}

/**
 * @brief Wait for MCL localization confidence to reach a minimum threshold
 * This ensures the robot knows where it is before executing critical movements
 * For sub-0.01" accuracy, we require 95%+ confidence
 *
 * @param minConfidence Minimum confidence (0.0 to 1.0), default 0.95 (95%)
 * @param timeout Maximum time to wait in milliseconds, default 3000ms
 * @return true if confidence reached, false if timeout
 */
bool waitForMCLConfidence(double minConfidence = 0.95, int timeout = 3000) {
  uint32_t startTime = pros::millis();
  while (pros::millis() - startTime < timeout) {
    double confidence = chassis.getMCLConfidence();
    if (confidence >= minConfidence) {
      pros::lcd::print(3, "MCL Ready: %.2f%% (High Precision)",
                       confidence * 100.0);
      return true;
    }
    pros::delay(20);
  }
  // Timeout - print warning but continue (don't halt auton completely)
  pros::lcd::print(3, "MCL Warning: Low Conf %.2f%%",
                   chassis.getMCLConfidence() * 100.0);
  return false;
}

void linpid() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.setPose(lemlib::Pose(-48, 24, 180));
  // mclFilter.setPose(lemlib::Pose(-48, 24, 180)); // Sync MCL with starting
  // pose - handled by chassis.setPose override
  chassis.moveToPoint(-48, -24, 2000);
  chassis.waitUntilDone();
}

void angpid() {
  chassis.setPose(lemlib::Pose(0, 0, 0));
  // mclFilter.setPose(lemlib::Pose(0, 0, 0)); // Sync MCL with starting pose -
  // handled by chassis.setPose override
  chassis.turnToHeading(90, 2000);
  chassis.waitUntilDone();
}

// Placeholder Rush autonomous routine (stub)
void Rush() {
  // Simple example: drive forward 24" and wait
  chassis.moveToPoint(24, 0, 2000);
  chassis.waitUntilDone();
}

void autonomous() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  Rush();
  // selector.run_auton();
}

void opcontrol() {

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  while (true) {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    chassis.arcade(leftY, rightX);

    pros::delay(10);
  }
}
