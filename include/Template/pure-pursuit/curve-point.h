#pragma once
#include "point.h"

class CurvePoint{
    public:
        Point point;
        float drive_voltage;
        float heading_max_voltage;
        float follow_distance;
        float drive_settle_error;
        float point_length;
        float slow_down_turn_radians;
        float slow_down_turn_amount;

        CurvePoint(Point point, float drive_voltage, float heading_max_voltage, float follow_distance, float drive_settle_error, float point_length, float slow_down_turn_radians, float slow_down_turn_amount);
        CurvePoint(Point point, float drive_voltage, float heading_max_voltage, float follow_distance);
        // TODO: remove unnecessary parameters (float drive_settle_error, float point_length, float slow_down_turn_radians, float slow_down_turn_amount)

        Point to_point();
};