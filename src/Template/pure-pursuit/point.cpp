#include "point.h"

Point::Point() :
    x(0),
    y(0),
    heading(0),
    has_heading(false)
{};

Point::Point(float x, float y, float heading) :
    x(x),
    y(y),
    heading(heading),
    has_heading(true)
{};

Point::Point(float x, float y) :
    x(x),
    y(y),
    heading(0),
    has_heading(false)
{};