#pragma once
#include "Template/api.h"

using namespace pros;

extern Drive chassis;

extern IMU inertial;
extern Rotation fwd_tracker;
extern Rotation sideways_tracker;
extern MotorGroup leftMotors;
extern MotorGroup rightMotors;

// Your motors, sensors, etc. should go here.  Below are examples
// inline pros::adi::DigitalIn limit_switch('A');

extern adi::DigitalOut claw;
extern adi::DigitalOut lift_intake;
extern adi::DigitalOut shovel;

extern Distance distance_sensorL;
extern Distance distance_sensorR;
extern Distance distance_sensorDown; // for keeping balls in intake
extern Distance distance_sensorUp; // for keeping balls in intake
extern Optical outtake_optical;