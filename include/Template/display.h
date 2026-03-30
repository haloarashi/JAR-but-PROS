#pragma once
#include "main.h"
using namespace pros;

void simple_screen_task();

void print_curvepoints(std::vector<CurvePoint> points);
void print_point(Point point, int line);

void init_map();
void map_curvepoints(std::vector<CurvePoint> points);
void update_map();
void map_task();