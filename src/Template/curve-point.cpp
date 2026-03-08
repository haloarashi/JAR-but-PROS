#include "main.h"

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