#include "main.h"

/**
 * Constructor for a CurvePoint, which is a point on a path with extra information for the pure pursuit algorithm. 
 * @param point The point on the field that the robot should head towards.
 * @param drive_voltage The voltage the robot should try to drive at when heading towards this
 * @param heading_max_voltage The voltage the robot should try to turn at when heading towards this point.
 * @param follow_distance The lookahead distance for the pure pursuit algorithm at this point.
 * @param drive_settle_error The distance from the point that is close enough to be considered "settled".
 * @param point_length NOT IMPLEMENTED YET The distance from this point to the next point, used for velocity profiling.
 * @param slow_down_turn_radians NOT IMPLEMENTED YET The angle in radians that the robot should start slowing down for a turn.
 * @param slow_down_turn_amount NOT IMPLEMENTED YET The amount that the robot should slow down when it has to turn, from 0 to 1, where 0 means no slowing down and 1 means stop completely.
 */
CurvePoint::CurvePoint(Point point, float drive_voltage, float heading_max_voltage, float follow_distance, float drive_settle_error, float point_length, float slow_down_turn_radians, float slow_down_turn_amount) :
  point(point),
  drive_voltage(drive_voltage),
  heading_max_voltage(heading_max_voltage),
  follow_distance(follow_distance),
  drive_settle_error(drive_settle_error),
  point_length(point_length),
  slow_down_turn_radians(slow_down_turn_radians),
  slow_down_turn_amount(slow_down_turn_amount)
{};

CurvePoint::CurvePoint(Point point, float drive_voltage, float heading_max_voltage, float follow_distance) :
  point(point),
  drive_voltage(drive_voltage),
  heading_max_voltage(heading_max_voltage),
  follow_distance(follow_distance),
  drive_settle_error(chassis.drive_settle_error)
{};

// Point CurvePoint::to_point(){
//   return point;
// }