#pragma once
#include "Template/api.h"

using namespace pros;

extern Drive chassis;

extern IMU inertial;
extern Rotation fwd_tracker;
// extern Rotation sideways_tracker;
extern MotorGroup leftMotors;
extern MotorGroup rightMotors;

// Your motors, sensors, etc. should go here.  Below are examples
// inline pros::adi::DigitalIn limit_switch('A');

extern adi::DigitalOut claw;
extern adi::DigitalOut lift;
extern adi::DigitalOut shovel;
extern adi::DigitalOut upper;
extern adi::DigitalOut odomlift;

extern Distance distance_sensorL;
extern Distance distance_sensorR;
extern Distance distance_sensorDown; // for keeping balls in intake
extern Distance distance_sensorUp; // for keeping balls in intake
extern Optical outtake_optical;

extern Motor intake_up;
extern Motor intake_mid;
extern Motor intake_down;

extern int intake_stats;
extern int intake_jam_count;
extern int intake_stats_count;
extern int midintake;
extern int upintake;
extern int intake_task;

void init();