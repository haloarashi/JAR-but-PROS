#include "main.h"

/**
 * Constructor for a CurvePoint, which is a point on a path with extra information for the pure pursuit algorithm. 
 * @param point The point on the field that the robot should head towards.
 * @param drive_voltage The voltage the robot should try to drive at when heading towards this
 * @param turn_voltage The voltage the robot should try to turn at when heading towards this point.
 * @param follow_distance The lookahead distance for the pure pursuit algorithm at this point.
 * @param point_length NOT IMPLEMENTED YET The distance from this point to the next point, used for velocity profiling.
 * @param slow_down_turn_radians NOT IMPLEMENTED YET The angle in radians that the robot should start slowing down for a turn.
 * @param slow_down_turn_amount NOT IMPLEMENTED YET The amount that the robot should slow down when it has to turn, from 0 to 1, where 0 means no slowing down and 1 means stop completely.
 */
CurvePoint::CurvePoint(Point point, float drive_voltage, float turn_voltage, float follow_distance, float point_length, float slow_down_turn_radians, float slow_down_turn_amount) :
  x(point.x),
  y(point.y),
  drive_voltage(drive_voltage),
  turn_voltage(turn_voltage),
  follow_distance(follow_distance),
  point_length(point_length),
  slow_down_turn_radians(slow_down_turn_radians),
  slow_down_turn_amount(slow_down_turn_amount)
{};

Point CurvePoint::to_point(){
  return Point(x, y);
}