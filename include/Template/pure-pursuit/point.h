#pragma once

class Point{
    public:
        float x;
        float y;
        float heading;
        bool has_heading;

        Point();
        Point(float x, float y);
        Point(float x, float y, float heading);
};