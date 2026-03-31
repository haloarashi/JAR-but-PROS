#pragma once
#include "main.h"

std::vector<Point> line_circle_intersection(Point circle_center, float radius, Point line_point_1, Point line_point_2);

float pt_to_pt_distance(Point point1, Point point2);

bool is_in_segment(Point point, Point line_point_1, Point line_point_2);

Point extend_path(Point line_point_1, Point line_point_2, float extension_length_in);

bool is_past_segment(Point robot_pos, Point line_point_1, Point line_point_2);

int sgn(float x);

double findLookaheadCurvature(Point robot_pos, float heading, Point lookahead);

float slew(float target, float current, float max_change, int restrict_direction);