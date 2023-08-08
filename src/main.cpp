#include <memory>

#include "main.h"
#include "motors.h"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "okapi/impl/device/controller.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "pros/rtos.hpp"
#include "autoSelect/selection.h"
enum autonChoices { LEFT, RIGHT };
autonChoices choice = LEFT;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // pros::lcd::initialize();
  // pros::lcd::set_text(1, "The Millenial Falcon tm");

  // pros::lcd::register_btn1_cb(on_center_button);
  selector::init();
  Logger::setDefaultLogger(std::make_shared<Logger>(
      TimeUtilFactory::createDefault().getTimer(),  // It needs a Timer
      "/ser/sout",            // Output to the PROS terminal
      Logger::LogLevel::warn  // Show errors and warnings
      ));
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  Motor expander1(11);
  Motor spinnerMotor(8);
  spinnerMotor.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
  Motor elevatorMotor(14);
  Motor flyWheelMotor(12);
  // flyWheelMotor.moveVoltage(-12000);


  std::shared_ptr<okapi::ChassisController> chassis =
      ChassisControllerBuilder()
          .withMotors(3,   // Top left
                      -4,  // Top right (reversed)
                      -1,  // Bottom right (reversed)
                      2    // Bottom left
                      )
          .withGains({0.001, 0, 0.0001},  // Distance controller gains
                     {0.001, 0, 0.0001},  // Turn controller gains
                     {0.001, 0, 0.0001}
                     // Angle controller gains (helps drive straight)
                     )
          .withDimensions(AbstractMotor::gearset::green,
                          {{4_in, 19.5_in}, imev5GreenTPR})
          .build();
  switch(selector::auton){
    case 1:
    case -1:
      chassis->moveDistance(-3.5_in);
      spinnerMotor.moveRelative(-.7, 300); 
      break;
    case 2:
    case -2:
      pros::delay(1);
      chassis->setMaxVelocity(120);
      chassis->moveDistance(-24_in);
      chassis->turnAngle(120_deg);
      chassis->moveDistance(-3.5_in);
      spinnerMotor.moveRelative(-.7, 200);
      break;
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  Controller controller;
  ControllerButton expanderButton(ControllerDigital::A);
  ControllerButton spinUpButton(ControllerDigital::L2);
  ControllerButton spinDownButton(ControllerDigital::L1);
  ControllerButton sweepDownButton(ControllerDigital::R1);
  ControllerButton sweepUpButton(ControllerDigital::R2);
  ControllerButton flywheelToggleButton(ControllerDigital::B);
  Motor expander1(11);
  // Motor expander2 (10);
  Motor spinnerMotor(8);
  Motor flyWheelMotor(12);
  Motor elevatorMotor(14);
  // Arcade Drive
  // X drive 4 wheels
  std::shared_ptr<ChassisController> drive =
      ChassisControllerBuilder()
          .withMotors(3,   // Top left
                      -4,  // Top right (reversed)
                      -1,  // Bottom right (reversed)
                      2    // Bottom left
                      )
          .withDimensions(AbstractMotor::gearset::green,
                          {{4_in, 19.5_in}, imev5GreenTPR})
          .build();
  auto xModel = std::dynamic_pointer_cast<XDriveModel>(drive->getModel());
  expander1.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
  bool spin = true;
  flyWheelMotor.moveVelocity(-150);
  bool fastSpin = false;
  elevatorMotor.setCurrentLimit(1800);
  while (true) {
    // Skid steer 4 wheela
    // std::shared_ptr<ChassisController> drive = ChassisControllerBuilder()
    // .withMotors(
    //     {-1, -2}, // Left motors are 1 & 2 (reversed)
    //     {3, 4}    // Right motors are 3 & 4
    // )
    // .withDimensions(AbstractMotor::gearset::green, {{4_in, 15.5_in},
    // imev5GreenTPR}) .build();

    // xModel->arcade(controller.getAnalog(ControllerAnalog::leftY),
    //                           controller.getAnalog(ControllerAnalog::leftX));

    // Field oriented X arcade
    xModel->fieldOrientedXArcade(-(controller.getAnalog(ControllerAnalog::leftX)),
                                (controller.getAnalog(ControllerAnalog::leftY)),
                                 controller.getAnalog(ControllerAnalog::rightX),
                                 90_deg);
    // X arcade
    // xModel->xArcade(controller.getAnalog(ControllerAnalog::leftX),
    // controller.getAnalog(ControllerAnalog::leftY),
    // controller.getAnalog(ControllerAnalog::rightX)); Tank Drive
    // xModel->tank(controller.getAnalog(ControllerAnalog::leftY),
    //                         controller.getAnalog(ControllerAnalog::rightY));
    // Delay for Robot to do background tasks
    if (spinUpButton.changedToPressed()) {
      spinnerMotor.moveVelocity(-100);
    } else if (spinUpButton.changedToReleased()) {
      spinnerMotor.moveVelocity(0);
    }
    if (spinDownButton.changedToPressed()) {
      spinnerMotor.moveVelocity(100);
    } else if (spinDownButton.changedToReleased()) {
      spinnerMotor.moveVelocity(0);
    }
    if(!fastSpin && flywheelToggleButton.changedToPressed()){
      flyWheelMotor.moveVelocity(-300);
      fastSpin=true;
    } else if (fastSpin  && flywheelToggleButton.changedToPressed()){
      flyWheelMotor.moveVelocity(-150);
      fastSpin=false;
    }
    if (sweepUpButton.changedToPressed()) {
      elevatorMotor.moveVelocity(-300);
    } else if (sweepUpButton.changedToReleased()) {
      elevatorMotor.moveVelocity(0);
    }
    if (sweepDownButton.changedToPressed()) {
      elevatorMotor.moveVelocity(300);
    } else if (sweepDownButton.changedToReleased()) {
      elevatorMotor.moveVelocity(0);
    }
    if (expanderButton.changedToPressed()) {
      expander1.moveRelative(.5, 60);
    }
    pros::delay(1);
  }
}