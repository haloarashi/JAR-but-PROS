#include "point.h"

Point::Point() :
    x(0),
    y(0),
    heading(0)
{};

Point::Point(float x, float y, float heading) :
    x(x),
    y(y),
    heading(heading)
{};