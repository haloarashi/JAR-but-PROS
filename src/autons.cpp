#include "main.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(90, 4.7625, 0, 21.167, 0, 20); // max 127
  chassis.set_heading_constants(64, 2.5, 0, 0, 0);
  chassis.set_turn_constants(105.833333, 2.54, .105833333, 17.4625, 15.0); // max 105
  chassis.set_swing_constants(116.4166667, 3.704166667, 0.08466667, 21.1666667, 15);
  chassis.set_wall_constants(127, 0.529166667, 0, 0, 0);
  //chassis.set_drive_min_constants(12, 1.45, 0, 2, 0, 1.5);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.8, 50, 3000);
  chassis.set_turn_exit_conditions(1.82, 50, 2000);
  chassis.set_swing_exit_conditions(1, 250, 3000);

  chassis.follow_distance = 3;
}