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