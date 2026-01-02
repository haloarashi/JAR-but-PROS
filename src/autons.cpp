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
  chassis.set_drive_constants(12, 0.47, 0, 2, 0, 0); //default (10, 1.5, 0, 10, 0); // I CHANGED KPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
  chassis.set_heading_constants(6, .27, 0, 1, 0); //  chassis.set_heading_constants(6, .375, 0, 1, 0); // I CHANGED KPPPPPPPPPPPPPPPPP
  chassis.set_turn_constants(10, .24, .01, 1.65, 15); // I CHANGED KPPPPPPPPPPPPPPPPPPPPPPP
  chassis.set_swing_constants(11, .35, .008, 2, 15); //(12, .3, .001, 2, 15);
  chassis.set_wall_constants(8, 0.05, 0, 0, 0); //chassis.set_wall_constants(4.5, 0.065, 0, 0, 0);
  //chassis.set_drive_min_constants(12, 1.45, 0, 2, 0, 1.5);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.8, 115, 3000);
  chassis.set_turn_exit_conditions(1.82, 75, 2000); // I CHANGED SETTLE_TIMEEEEEEEEEE
  chassis.set_swing_exit_conditions(1, 250, 3000);
}