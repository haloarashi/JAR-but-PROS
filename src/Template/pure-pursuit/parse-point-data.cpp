#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include "parse-point-data.h"

// Given data from path.jerryio.com is in the format "x,y,drive_voltage" or "x,y,drive_voltage,heading" per line

std::vector<CurvePoint> parse_point_data(std::string point_data){
    std::vector<CurvePoint> path_points = {};
    std::stringstream ss(point_data);
    std::string line;

    while(std::getline(ss, line)){
        std::stringstream linestream(line);
        std::string value;
        std::vector<float> values;

        while(std::getline(linestream, value, ',')){
            values.push_back(std::stof(value));
        }

        if(int(values.size()) == 3){
            CurvePoint point = CurvePoint(Point(values[0], values[1]), values[2], chassis.turn_max_voltage, chassis.follow_distance);
            path_points.push_back(point);
        }
        else{
            CurvePoint point = CurvePoint(Point(values[0], values[1], values[3]), values[2], chassis.turn_max_voltage, chassis.follow_distance);
            path_points.push_back(point);
        }
    }

    return path_points;
}