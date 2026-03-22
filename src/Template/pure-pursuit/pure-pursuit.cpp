#include "Template/drive.h"

// TODO: Make this non-blocking. In get_follow_point(), only update last_found_index if the robot is past the point. 
void Drive::go_to_point(float X_position, float Y_position, float drive_voltage, float heading_max_voltage, float drive_settle_error){
    // PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
    float start_angle_deg = to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()));
    PID headingPID(start_angle_deg-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
    // bool line_settled = false;
    // bool prev_line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
    // drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());

    // while(drive_error > drive_settle_error){
    // while(true){
        // bool line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());

        // drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position()); // keep this so other parts of the code can access drive_error correctly
        
        float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
        float heading_scale_factor = cos(to_rad(heading_error));
        
        heading_error = reduce_negative_90_to_90(heading_error);
        float heading_output = headingPID.compute(heading_error);
        
        float drive_output = drive_voltage * heading_scale_factor;

        // if (drive_error<drive_settle_error) { heading_output = 0; }

        drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
        heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

        // drive_output = clamp_min_voltage(drive_output, heading_scale_factor*drive_min_voltage);

        // if(line_settled){
        //     // drive_output = 0;
        //     break;
        // }
        // if(line_settled && fabs(heading_error) < turn_settle_error){
        //     break;
        // }
        
        drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    //     delay(10);
    // }
}

CurvePoint Drive::get_follow_point(std::vector<CurvePoint> path_points, Point robot_pos, float follow_radius){
    CurvePoint follow_me = CurvePoint(path_points[last_found_index + 1]);
    
    for(int i = last_found_index + 1; i < path_points.size()-1; i++){ // -1 is because we need two points per loop. There is no second -1 (to form a -2) because the last appended extra point must be chosen as a target, just not drive to it. 
        CurvePoint start = path_points[i];
        CurvePoint end = path_points[i+1];

        // pure pursuit
        std::vector<Point> intersections = line_circle_intersection(robot_pos, follow_radius, start.point, end.point);
        
        if(intersections.size() == 0){
            continue;
        }

        Point follow_point;
        if(intersections.size() > 1){ // choose the intersection point closer to the end // maybe intersections[1] would work since it should be the one closer to the end, but this is safer just in case
            follow_point = pt_to_pt_distance(intersections[0], end.point) < pt_to_pt_distance(intersections[1], end.point) ? intersections[0] : intersections[1];
        }
        else{
            follow_point = intersections[0];
        }

        if(pt_to_pt_distance(robot_pos, end.point) <= follow_radius){
            follow_me = end; // prevents the robot from going backwards
        }
        else{
            follow_me = CurvePoint(follow_point, end.drive_voltage, end.heading_max_voltage, end.follow_distance, end.drive_settle_error, end.point_length, end.slow_down_turn_radians, end.slow_down_turn_amount);
        }

        // last_found_index = i;
        break;
    }

    return follow_me;
}

void Drive::follow_path(std::vector<CurvePoint> path_points){
    last_found_index = -1;

    // Extend path by 12in so robot doesn't oscillate crazily towards the end of the path
    Point extend_point = extend_path(path_points[path_points.size()-2].point, path_points[path_points.size()-1].point, 12);
    path_points.push_back(CurvePoint(extend_point, path_points[path_points.size()-1].drive_voltage, path_points[path_points.size()-1].heading_max_voltage, path_points[path_points.size()-1].follow_distance, path_points[path_points.size()-1].drive_settle_error, path_points[path_points.size()-1].point_length, path_points[path_points.size()-1].slow_down_turn_radians, path_points[path_points.size()-1].slow_down_turn_amount));

    // Figure out where robot is on the path when starting to follow it
    for(int i = last_found_index + 1; i < path_points.size()-2; i++){ // -1 because last_found_index starts at -1, which is out of bounds
        if(is_in_segment(Point(get_X_position(), get_Y_position()), path_points[i].point, path_points[i+1].point)){
            last_found_index = i - 1; // -1 because loop starts with last_found_index + 1
            break;
        }
    }

    // Follow the path
    while(last_found_index < (int)path_points.size() - 2){ // while we're not following the last point
        Point robot_pos = Point(get_X_position(), get_Y_position(), get_absolute_heading());
        CurvePoint follow_me = get_follow_point(path_points, robot_pos, path_points[last_found_index + 1].follow_distance); // last_found_index + 1 is the next point (the point we are following)
    
        go_to_point(follow_me.point.x, follow_me.point.y, follow_me.drive_voltage, follow_me.heading_max_voltage, follow_me.drive_settle_error);
        
        if(is_past_segment(robot_pos, path_points[last_found_index].point, path_points[last_found_index + 1].point)){ // TODO: THIS WILL CRASH BECAUSE last_found_index CAN BE -1. 
            last_found_index++;
        }
    }
}