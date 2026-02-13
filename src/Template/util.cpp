#include "main.h"
#include <cmath>

void brake_with_mode(Motor& motor, MotorBrake new_mode) {
    MotorBrake old_mode = motor.get_brake_mode();

    motor.set_brake_mode(new_mode);

    motor.brake();

    motor.set_brake_mode(old_mode);
}

void brake_with_mode_group(MotorGroup& motor_group, MotorBrake new_mode) {
    MotorBrake old_mode = motor_group.get_brake_mode();

    motor_group.set_brake_mode(new_mode);

    motor_group.brake();

    motor_group.set_brake_mode(old_mode);
}

/**
 * Converts an angle to an equivalent one in the range [0, 360).
 * 
 * @param angle The angle to be reduced in degrees.
 * @return Reduced angle.
 */

float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

/**
 * Converts an angle to an equivalent one in the range [-180, 180).
 * 
 * @param angle The angle to be reduced in degrees.
 * @return Reduced angle.
 */

float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}

/**
 * Converts an angle to an equivalent one in the range [-90, 90).
 * If the angle has no equivalent, then the angle halfway around
 * the circle is returned.
 * 
 * @param angle The angle to be reduced in degrees.
 * @return Reduced angle.
 */

float reduce_negative_90_to_90(float angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { angle += 180; }
    if(angle >= 90) { angle -= 180; }
  }
  return(angle);
}

/**
 * Converts an angle in degrees to radians.
 * 
 * @param angle The angle in degrees.
 * @return Angle in radians.
 */

float to_rad(float angle_deg){
  return(angle_deg/(180.0/M_PI));
}

/**
 * Converts an angle in radians to degrees.
 * 
 * @param angle The angle in radians.
 * @return Angle in degrees.
 */

float to_deg(float angle_rad){
  return(angle_rad*(180.0/M_PI));
}

/**
 * "Clamps" a number between a min and max.
 * Does no error checking and assumes min is less
 * than or equal to max.
 * 
 * @param input The number to be clamped.
 * @param min Minimum edge of the clamp.
 * @param max Maximum edge of the clamp.
 * @return Clamped number.
 */

float clamp(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

/**
 * Checks if an input number corresponds to a reversed drive port.
 * 
 * @param input The number to be checked.
 * @return Whether the input is negative.
 */

bool is_reversed(double input){
  if(input<0) return(true);
  return(false);
}

/**
 * Scales a joystick to drive voltage scale.
 * Values get multiplied by 1.27 because motors can
 * output a max of 127 volts.
 * 
 * @param percent The input joystick reading.
 * @return The equivalent value in volts.
 */

float to_volt(float percent){
  return(percent*127.0/100.0);
}

/**
 * Converts an input port to a triport.
 * Does error checking to prevent invalid values from being
 * passed to the triport array.
 * 
 * @param port Input integer port.
 * @return Port-1, to convert it to a triport.
 */

int to_port(int port){
  if(port > 8 || port < 1){
    return(0);
  }
  return(port-1);
}

/**
 * Deadband function for joystick control.
 * If the joystick value is small, we should just consider it 0.
 * 
 * @param input The input joystick value.
 * @param width Minimum value to not get zeroed out.
 * @return The deadbanded value.
 */

float deadband(float input, float width){
  if (std::fabs(input)<width){
    return(0);
  }
  return(input);
}

/**
 * Settling control for odometry functions.
 * Draws a line perpendicular to the line from the robot to the desired 
 * endpoint, and checks if the robot has crossed that line. Allows for
 * very quick settling, and thereby chaining for fast motion control.
 * 
 * @param desired_X The ending X position in inches.
 * @param desired_Y The ending Y position in inches.
 * @param desired_angle_deg The direction of the line to be drawn.
 * @param current_X The robot's X position in inches.
 * @param current_Y The robot's Y position in inches.
 * @return Whether the robot can be considered settled.
 */

bool is_line_settled(float desired_X, float desired_Y, float desired_angle_deg, float current_X, float current_Y){
  return( (desired_Y-current_Y) * cos(to_rad(desired_angle_deg)) <= -(desired_X-current_X) * sin(to_rad(desired_angle_deg)) );
}

/**
 * Voltage scaling to keep from applying more than 12 volts to either side of the drive.
 * Divides both drive and heading output proportionally to get a similar result to the
 * desired one.
 * 
 * @param drive_output The forward output of the drive.
 * @param heading_output The angular output of the drive.
 * @return The scaled voltage for the left side of the robot.
 */

float left_voltage_scaling(float drive_output, float heading_output){
  float ratio = std::max(std::fabs(drive_output+heading_output), std::fabs(drive_output-heading_output))/127.0;
  if (ratio > 1) {
    return (drive_output+heading_output)/ratio;
  }
  return drive_output+heading_output;
}

/**
 * Voltage scaling to keep from applying more than 12 volts to either side of the drive.
 * Divides both drive and heading output proportionally to get a similar result to the
 * desired one.
 * 
 * @param drive_output The forward output of the drive.
 * @param heading_output The angular output of the drive.
 * @return The scaled voltage for the right side of the robot.
 */

float right_voltage_scaling(float drive_output, float heading_output){
  float ratio = std::max(std::fabs(drive_output+heading_output), std::fabs(drive_output-heading_output))/127.0;
  if (ratio > 1) {
    return (drive_output-heading_output)/ratio;
  }
  return drive_output-heading_output;
}

/**
 * Brings an output up to the minimum voltage if it's too slow.
 * Used for minimum voltage calculations for movement chaining.
 * Has no effect on 0 voltage output, because how do we know 
 * which way it's supposed to be going?
 * 
 * @param drive_output The forward output of the drive.
 * @param drive_min_voltage The minimum output of the drive.
 * @return The voltage with the minimum applied.
 */

float clamp_min_voltage(float drive_output, float drive_min_voltage){
  if(drive_output < 0 && drive_output > -drive_min_voltage){
      return -drive_min_voltage;
  }
  if(drive_output > 0 && drive_output < drive_min_voltage){
    return drive_min_voltage;
  }
  return drive_output;
}

uint32_t hsv_to_rgb(float h, float s, float v, float time) {
    int i = int(h / 60) % 6;
    float f = h / 60 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    float r, g, b;
    switch (i) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default:r = v; g = p; b = q; break;
    }
    
    // time = time / 100000; // scale speed
    // r = uint8_t((sin(time) * 0.5f + 0.5f) * 255);
    // g = uint8_t((sin(time + 2) * 0.5f + 0.5f) * 255);
    // b = uint8_t((sin(time + 4) * 0.5f + 0.5f) * 255);

    return (uint32_t(r * 255) << 16) | (uint32_t(g * 255) << 8) | uint32_t(b * 255);
}

uint32_t rgb_to_uint32(int r, int g, int b) {
    return (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
}

int overheat = 0;
int screen_time = 0;
void simple_screen_task(){
	// chassis.set_coordinates(29.804, 66.47, 180);
	while (true){
		pros::screen::erase();
    int overheat_threshold = 40;
    overheat = 0;

    if(chassis.DriveL.get_temperature(0) > overheat_threshold || chassis.DriveL.get_temperature(1) > overheat_threshold || chassis.DriveL.get_temperature(2) > overheat_threshold){
      pros::screen::print(TEXT_LARGE_CENTER, 10, "DRIVE_L OVERHEATING");
      overheat = 1;
    }
    if(chassis.DriveR.get_temperature(0) > overheat_threshold || chassis.DriveR.get_temperature(1) > overheat_threshold || chassis.DriveR.get_temperature(2) > overheat_threshold){
      pros::screen::print(TEXT_LARGE_CENTER, 10, "DRIVE_R OVERHEATING");
      overheat = 1;
	  }
    // if(intake_down.get_temperature() > overheat_threshold){
    //   pros::screen::print(TEXT_LARGE_CENTER, 10, "INTAKE_DOWN OVERHEATING");
    //   overheat = 2;
    // }
    // if(intake_mid.get_temperature() > overheat_threshold){
    //   pros::screen::print(TEXT_LARGE_CENTER, 10, "INTAKE_MID OVERHEATING");
    //   overheat = 3;
    // }
    // if(intake_up.get_temperature() > overheat_threshold){
    //   pros::screen::print(TEXT_LARGE_CENTER, 10, "INTAKE_UP OVERHEATING");
    //   overheat = 3;
    // }
    
    uint32_t flash_color;

    switch(overheat){
      case 1:
        flash_color = pros::c::COLOR_RED;
        break;

      case 2:
        flash_color = pros::c::COLOR_ORANGE;
        break;
      
      case 3:
        flash_color = pros::c::COLOR_GREEN;
        break;
      
      default:
        // pros::screen::set_pen(uint32_t((screen_time * 10) % 16777215));
        // pros::screen::set_eraser(uint32_t((screen_time * 10) % 16777215));
        float hue = fmod(screen_time  * 0.06, 360);
        // pros::screen::set_eraser(hsv_to_rgb(hue, 1, 1, screen_time));
        pros::screen::set_pen(pros::c::COLOR_BLACK);
        pros::screen::set_eraser(pros::c::COLOR_BLACK);
        pros::screen::fill_rect(1,1,480,240);
        pros::screen::set_pen(hsv_to_rgb(hue, 1, 1, screen_time));
        break;
    }

    if(overheat != 0) {
      if(screen_time % 250 < 125){
        pros::screen::set_pen(pros::c::COLOR_BLACK);
        pros::screen::set_eraser(pros::c::COLOR_BLACK);
        pros::screen::fill_rect(1,1,480,240);
      }
      else{
        pros::screen::set_pen(flash_color);
        pros::screen::set_eraser(flash_color);
        pros::screen::fill_rect(1,1,480,240);
      }
      pros::screen::set_pen(pros::c::COLOR_WHITE);
    }

		pros::screen::print(TEXT_MEDIUM, 1, "X: %.2f,  Y: %.2f, Heading: %.2f", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
		// pros::screen::print(TEXT_MEDIUM, 2, "Intake temp: down %.2f, mid %.2f, up %.2f", intake_down.get_temperature(), intake_mid.get_temperature(), intake_up.get_temperature());
		pros::screen::print(TEXT_MEDIUM, 3, "DT L temp: %.2f, %.2f, %.2f", chassis.DriveL.get_temperature(0), chassis.DriveL.get_temperature(1), chassis.DriveL.get_temperature(2));
		pros::screen::print(TEXT_MEDIUM, 4, "DT R temp: %.2f, %.2f, %.2f", chassis.DriveR.get_temperature(0), chassis.DriveR.get_temperature(1), chassis.DriveR.get_temperature(2));
		pros::screen::print(TEXT_MEDIUM, 5, "Distance L (mm): %d", distance_sensorL.get());
		pros::screen::print(TEXT_MEDIUM, 6, "Distance R (mm): %d", distance_sensorR.get());
    pros::screen::print(TEXT_MEDIUM, 7, "Gyro pitch: %.2f", chassis.Gyro.get_pitch());
    pros::screen::print(TEXT_MEDIUM, 8, "Distance Up (mm): %d", distance_sensorUp.get());
    // pros::screen::print(TEXT_MEDIUM, 9, "Mid jam time: %d", mid_jam_time);
    // pros::screen::print(TEXT_MEDIUM, 10, "Unjam attempt time: %d Rest time: %d", unjam_attempt_time, rest_time);
    // pros::screen::print(TEXT_LARGE_CENTER, 9, "Progress: %d", progress);

    delay(20);
    screen_time += 20;
  }
}

double mm_to_inch(double mm){
  return(mm/25.4);
}

void wait_until(std::function<bool()> condition, int check_interval_ms, int timeout_ms) {
  while (!condition() && (timeout_ms > 0)) {
    delay(check_interval_ms);
    if (timeout_ms > 0) timeout_ms -= check_interval_ms;
  }
}

void wait_until_bool(bool condition, int check_interval_ms, int timeout_ms){ // if timeout_ms is 0 or below, wait indefinitely
  while (!condition && (timeout_ms > 0)) {
    delay(check_interval_ms);
    timeout_ms -= check_interval_ms;
  }
}