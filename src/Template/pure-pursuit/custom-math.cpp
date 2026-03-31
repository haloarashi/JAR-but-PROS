#include "custom-math.h"

float pt_to_pt_distance(Point point1, Point point2){
    return hypot(point1.x - point2.x, point1.y - point2.y);
}

std::vector<Point> line_circle_intersection(Point circle_center, float radius, Point line_point_1, Point line_point_2){
    if(fabs(line_point_1.y - line_point_2.y) < 0.0001){
        line_point_1.y = line_point_2.y + 0.0001;
    }

    if(fabs(line_point_1.x - line_point_2.x) < 0.0001){
        line_point_1.x = line_point_2.x + 0.0001;
    }

    float line_m = (line_point_2.y - line_point_1.y)/(line_point_2.x - line_point_1.x);
    float circle_x = circle_center.x - line_point_1.x;
    float circle_y = circle_center.y - line_point_1.y;

    float minX = line_point_1.x < line_point_2.x ? line_point_1.x : line_point_2.x;
    float maxX = line_point_1.x > line_point_2.x ? line_point_1.x : line_point_2.x;
    
    float quadraticA = pow(line_m, 2) + 1;
    float quadraticB = 2*line_m*circle_y - 2*pow(line_m, 2)*circle_x;
    float quadraticC = pow(line_m, 2) * pow(circle_x, 2) - 2*line_m*circle_y*circle_x + pow(circle_y, 2) - pow(radius, 2);
    float discriminant = pow(quadraticB, 2) - 4*quadraticA*quadraticC;

    std::vector<Point> all_intersections = {};
    
    if(discriminant < 0){
        return all_intersections;
    }
    
    float sqrt_discriminant = sqrt(discriminant);
    
    float x_root1 = (-quadraticB + sqrt_discriminant)/(2*quadraticA) + circle_center.x;
    float y_root1 = line_m*(x_root1 - circle_x) + circle_y + circle_center.y;
    
    float x_root2 = (-quadraticB - sqrt_discriminant)/(2*quadraticA) + circle_center.x;
    float y_root2 = line_m*(x_root2 - circle_x) + circle_y + circle_center.y;

    if(minX < x_root1 && x_root1 < maxX){
        all_intersections.push_back(Point(x_root1, y_root1));
    }

    if(minX < x_root2 && x_root2 < maxX){
        all_intersections.push_back(Point(x_root2, y_root2));
    }

    return all_intersections;
}

bool is_in_segment(Point point, Point line_point_1, Point line_point_2){
    float minX = fmin(line_point_1.x, line_point_2.x);
    float maxX = fmax(line_point_1.x, line_point_2.x);
    float minY = fmin(line_point_1.y, line_point_2.y);
    float maxY = fmax(line_point_1.y, line_point_2.y);

    if(point.x < minX || point.x > maxX || point.y < minY || point.y > maxY){
        return false;
    }

    return true;
}

Point extend_path(Point line_point_1, Point line_point_2, float extension_length_in){
    float line_angle = atan2(line_point_2.y - line_point_1.y, line_point_2.x - line_point_1.x);
    return Point(line_point_2.x + cos(line_angle)*extension_length_in, line_point_2.y + sin(line_angle)*extension_length_in);
}

// dot(AR, AB) > |AB|²
// Project AR onto AB, if the projection is longer than AB, then the robot is past the point. 
bool is_past_segment(Point robot_pos, Point line_point_1, Point line_point_2){
    float ABx = line_point_2.x - line_point_1.x;
    float ABy = line_point_2.y - line_point_1.y;

    float ARx = robot_pos.x - line_point_1.x;
    float ARy = robot_pos.y - line_point_1.y;

    float dot_AR_AB = ARx * ABx + ARy * ABy;
    float AB_length_squared = ABx * ABx + ABy * ABy; // basically pt_to_pt_distance squared but without the computation of sqrt then square

    return dot_AR_AB > AB_length_squared;
}

int sgn(float x){
    return (x > 0) - (x < 0);
}

/**
 * @brief Get the curvature of a circle that intersects the robot and the lookahead point
 *
 * @param robot_pos the position of the robot
 * @param heading the heading of the robot
 * @param lookahead the lookahead point
 * @return float curvature
 */
double findLookaheadCurvature(Point robot_pos, float heading, Point lookahead) {
    // calculate whether the robot is on the left or right side of the circle
    int side = sgn(sin(to_rad(heading)) * (lookahead.x - robot_pos.x) - cos(to_rad(heading)) * (lookahead.y - robot_pos.y));
    
    // calculate center point and radius
    double a = -tan(to_rad(heading));
    double c = tan(to_rad(heading)) * robot_pos.x - robot_pos.y;
    double x = fabs(a * lookahead.x + lookahead.y + c) / sqrt((a * a) + 1);
    double d_squared = pow(lookahead.x - robot_pos.x, 2) + pow(lookahead.y - robot_pos.y, 2);

    // return curvature
    return side * ((2 * x) / (d_squared));
}

/**
 * @param max_change the maximum change in voltage per iteration
 * @param restrict_direction 0 for no restriction, 1 for only allow increasing, -1 for only allow decreasing
 */
float slew(float target, float current, float max_change, int restrict_direction) {
    if (max_change == 0) return target;

    const float change = target - current;

    // only restrict change for specified directions
    if (restrict_direction == 1 && change < 0) return target;
    if (restrict_direction == -1 && change > 0) return target;

    if (abs(change) > abs(max_change))
        return current + (max_change * sgn(change));

    // return the target if no restriction is necessary
    return target;
}