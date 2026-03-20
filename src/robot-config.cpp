#include "main.h"

IMU inertial(3);
Rotation fwd_tracker(2); // I just put a random number here but we dont have a forward tracker
Rotation sideways_tracker(1);  // I just put a random number here but we dont have a sideways tracker

// MotorGroup leftMotors({-10, 15, 18}); // negative port number means reversed
// MotorGroup rightMotors({-20, 7, -8}); // negative port number means reversed

MotorGroup leftMotors({-16, -20, -14}); // (front, mid, back) negative port number means reversed
MotorGroup rightMotors({13, 12, 11}); // (front, mid, back) negative port number means reversed

Motor intake_up(9, MotorGears::green);
Motor intake_mid(17, MotorGears::green);
Motor intake_down(-6, MotorGears::green);

adi::DigitalOut claw('B');
adi::DigitalOut shovel('F');
adi::DigitalOut lift('G');
adi::DigitalOut upper('E');
adi::DigitalOut odomlift('D');

Distance distance_sensorDown(13);
Distance distance_sensorUp(14);
Distance distance_sensorL(1);
Distance distance_sensorR(2);
Optical block_optical(21);

int intake_stats;
int intake_jam_count;
int intake_status_count;
int intake_task;
int midintake;
int upintake;



Drive chassis(

    // Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
    // You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

    // Drive Style (see drive.h for option list): 
    Drive::DriveStyle::ZERO_TRACKER,
    
    // Left Motors:
    leftMotors,

    // Right Motors:
    rightMotors,

    // Your inertial sensor:
    inertial,

    // Input your wheel diameter. (4" omnis are actually closer to 4.125"):
    3.25,

    // External ratio, must be in decimal, in the format of input teeth/output teeth.
    // If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
    // If the motor drives the wheel directly, this value is 1:
    48/36,

    // Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
    // For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
    360,

    // If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
    // If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
    fwd_tracker,

    // Input the Forward Tracker diameter (reverse it to make the direction switch)
    // For a tank drive using odom without a forward tracker, this value is useless and does not affect anything:
    2,

    // Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
    // For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
    // This distance is in inches:
    5.5,


    // Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
    sideways_tracker,

    // Sideways tracker diameter (reverse it to make the direction switch):
    0,

    // Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
    0
);

void init() {
    leftMotors.set_gearing(E_MOTOR_GEARSET_06);
    rightMotors.set_gearing(E_MOTOR_GEARSET_06);
    
    delay(2500); // wait for imu to calibrate
    static Task screen_task(simple_screen_task); // This must be called in competition_initialize() or later. Calling this in initialize() doesn't work. 
    Controller(CONTROLLER_MASTER).rumble("..");
}