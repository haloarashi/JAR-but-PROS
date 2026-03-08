#pragma once

class CurvePoint{
    public:
        float x;
        float y;
        float drive_voltage;
        float turn_voltage;
        float follow_distance;
        float point_length;
        float slow_down_turn_radians;
        float slow_down_turn_amount;

        CurvePoint(Point point, float drive_voltage, float turn_voltage, float follow_distance, float point_length, float slow_down_turn_radians, float slow_down_turn_amount);
        
        Point to_point();
};