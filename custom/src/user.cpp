#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 3;
  switch(auton_selected) {
    case 1:
      exampleAuton();
      break;
    case 2:
      exampleAuton2();
      break;  
    case 3:
      redGoalRush();
      break;
    case 4:
      break; 
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

// “button charge detection” for toggles (edge detection)
bool matchloadCD = true;
bool descoreCD = true;
bool liftCD = true;

bool matchloadState = false;
bool descoreState = false;
bool liftState = false;

void runDriver() {
  stopChassis(coast);
  heading_correction = false;
  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // intake control: R1 in, R2 out
    int intakePct = (r1 ? 100 : 0) - (r2 ? 100 : 0);
    if (intakePct > 0) {
      intake_motor.spin(directionType::fwd, intakePct, velocityUnits::pct);
    } else if (intakePct < 0) {
      intake_motor.spin(directionType::rev, -intakePct, velocityUnits::pct);
    } else {
      intake_motor.stop(brakeType::coast);
    }

    // matchload toggle on down arrow
    if (button_down_arrow) {
      if (matchloadCD) {
        matchloadCD = false;
        matchloadState = !matchloadState;
        matchload.set(matchloadState);
      }
    } else {
      matchloadCD = true;
    }

    // descore toggle on B
    if (button_b) {
      if (descoreCD) {
        descoreCD = false;
        descoreState = !descoreState;
        descore.set(descoreState);
      }
    } else {
      descoreCD = true;
    }

    // hood on L1 / off otherwise
    if (l1) {
      hood.set(true);
    } else {
      hood.set(false);
    }

    // lift toggle on L2
    if (l2) {
      if (liftCD) {
        liftCD = false;
        liftState = !liftState;
        lift.set(liftState);
      }
    } else {
      liftCD = true;
    }

    driveChassis(ch3 * 0.12 + ch1 * 0.12 , ch3 * 0.12 - ch1 * 0.12);

    wait(10, msec);
  }
}

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }
}