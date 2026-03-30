#pragma once
#include "Template/api.h"

using namespace pros;

extern Drive chassis;

extern IMU inertial;
extern Rotation fwd_tracker;
extern Rotation sideways_tracker;

extern MotorGroup leftMotors;
extern MotorGroup rightMotors;

extern Motor intake;
extern Motor outtake;

extern adi::DigitalOut claw;
extern adi::DigitalOut shovel;
extern adi::DigitalOut lift;
extern adi::DigitalOut upper;
extern adi::DigitalOut outtake_lift;

extern Distance distance_sensorL;
extern Distance distance_sensorR;

void init();

// Your motors, sensors, etc. should go here.  Below are examples
// inline pros::adi::DigitalIn limit_switch('A');