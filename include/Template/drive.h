#pragma once
#include "main.h"
using namespace pros;

class Drive{
    private: 
        float wheel_diameter;
        float wheel_ratio;
        float gyro_scale;
        float drive_in_to_deg_ratio;
        float ForwardTracker_center_distance;
        float ForwardTracker_diameter;
        float ForwardTracker_in_to_deg_ratio;
        float SidewaysTracker_center_distance;
        float SidewaysTracker_diameter;
        float SidewaysTracker_in_to_deg_ratio;

        Controller master;

    public: 
        MotorGroup& DriveL;
        MotorGroup& DriveR;
        IMU& Gyro;
        Rotation& Fwd_tracker;
        Rotation& Sideways_tracker;

        float turn_max_voltage;
        float turn_kp;
        float turn_ki;
        float turn_kd;
        float turn_starti;

        float turn_settle_error;
        float turn_settle_time;
        float turn_timeout;

        float drive_min_voltage;
        float drive_max_voltage; // currently only used in drive_to_point() and drive_to_pose()
        float drive_kp;
        float drive_ki;
        float drive_kd;
        float drive_starti;

        float drive_settle_error;
        float drive_settle_time;
        float drive_timeout;

        float heading_max_voltage;
        float heading_kp;
        float heading_ki;
        float heading_kd;
        float heading_starti;

        float swing_max_voltage;
        float swing_kp;
        float swing_ki;
        float swing_kd;
        float swing_starti;

        float swing_settle_error;
        float swing_settle_time;
        float swing_timeout;

        float boomerang_lead;
        float boomerang_setback;

        float wall_max_voltage;
        float wall_kp;
        float wall_ki;
        float wall_kd;
        float wall_starti;

        float drive_error;

        Drive(MotorGroup& left_motors, MotorGroup& right_motors, IMU& inertial, 
              float wheel_diameter, float motor_gear_ratio, float gyro_scale, 
              Rotation& fwd_tracker, float fwd_tracker_diameter, float fwd_tracker_dist, 
              Rotation& sideways_tracker, float sideways_tracker_diameter, float sideways_tracker_dist);
        
        void drive_with_voltage(int left_voltage, int right_voltage);

        float get_absolute_heading();

        float get_left_position_in();
        float get_right_position_in();

        void drive_stop(MotorBrake mode);

        void set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti); 
        void set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float drive_min_voltage);
        void set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
        void set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti);
        void set_wall_constants(float wall_max_voltage, float wall_kp, float wall_ki, float wall_kd, float wall_starti);
        // void set_drive_min_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float drive_min_voltage);
        void set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout);
        void set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout);
        void set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout);

        void turn_to_angle(float angle);
        void turn_to_angle(float angle, float turn_max_voltage);
        void turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout);
        void turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti);

        void drive_distance(float distance);
        void drive_distance(float distance, float heading);
        void drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage);
        void drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout);
        void drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

        void drive_min_distance(float distance);
        void drive_min_distance(float distance, float heading);
        void drive_min_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage);
        void drive_min_distance(float distance, float heading, float drive_min_voltage);
        void drive_min_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);


        void left_swing_to_angle(float angle);
        void left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti);
        void right_swing_to_angle(float angle);
        void right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti);
        
        void left_wall_distance(float distance, float heading, float left_wall_dis_target);
        void min_left_wall_distance(float distance, float heading, float left_wall_dis_target,float _drive_min_voltage);
        void right_wall_distance(float distance, float heading, float right_wall_dis_target);
        void min_right_wall_distance(float distance, float heading, float right_wall_dis_target,float _drive_min_voltage);
        
        Odom odom;
        float get_ForwardTracker_position();
        float get_SidewaysTracker_position();
        void set_coordinates(float X_position, float Y_position, float orientation_deg);
        void set_heading(float orientation_deg);
        void position_track();
        static int position_track_task();
        // vex::task odom_task;
        Task* odom_task = nullptr;
        float get_X_position();
        float get_Y_position();

        // void drive_stop(vex::brakeType mode);

        void drive_to_point(float X_position, float Y_position);
        void drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage);
        void drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout);
        void drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
        
        void drive_to_pose(float X_position, float Y_position, float angle);
        void drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage);
        void drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage);
        void drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout);
        void drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
        
        void turn_to_point(float X_position, float Y_position);
        void turn_to_point(float X_position, float Y_position, float extra_angle_deg);
        void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout);
        void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti);

        void control_arcade();
        void control_tank();
        void control_holonomic();
};