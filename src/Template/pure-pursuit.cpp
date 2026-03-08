#include "drive.h"

void Drive::go_to_point(float X_position, float Y_position, float drive_voltage){
    // PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
    float start_angle_deg = to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()));
    PID headingPID(start_angle_deg-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
    bool line_settled = false;
    bool prev_line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());

    while(true){
        line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
        if(line_settled && !prev_line_settled){ break; }
        prev_line_settled = line_settled;

        // drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position()); // used only to update drive_error so that otherparts of the code can access the variable correctly
        float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
        float drive_output = drive_voltage;

        float heading_scale_factor = cos(to_rad(heading_error));
        drive_output*=heading_scale_factor;
        heading_error = reduce_negative_90_to_90(heading_error);
        float heading_output = headingPID.compute(heading_error);
        
        // if (drive_error<drive_settle_error) { heading_output = 0; }

        drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
        heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

        drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

        drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
        delay(10);
    }
}

CurvePoint Drive::get_follow_point(std::vector<CurvePoint> path_points, Point robot_pos, float follow_radius){
    CurvePoint follow_me = CurvePoint(path_points[last_found_index + 1]);
    
    for(int i = last_found_index + 1; i < path_points.size()-2; i++){ // -2 explanation: -1 is because we need two points per loop, and another -1 is because an extra point is always appended to the end of path_points
        CurvePoint start = path_points[i];
        CurvePoint end = path_points[i+1];

        // determine current segment
        float y_intersect = (robot_pos.x - path_points[i].x);

        // pure pursuit
        std::vector<Point> intersections = line_circle_intersection(robot_pos, follow_radius, start.to_point(), end.to_point());
        
        if(intersections.size() == 0){
            continue;
        }

        Point follow_point;
        if(intersections.size() > 1){
            follow_point = pt_to_pt_distance(intersections[0], end.to_point()) < pt_to_pt_distance(intersections[1], end.to_point()) ? intersections[0] : intersections[1];
        }
        else{
            follow_point = intersections[0];
        }

        if(pt_to_pt_distance(robot_pos, end.to_point()) < pt_to_pt_distance(follow_point, end.to_point())){
            follow_me = end;
        }
        else{
            follow_me = CurvePoint(follow_point, end.drive_voltage, end.turn_voltage, end.follow_distance, end.point_length, end.slow_down_turn_radians, end.slow_down_turn_amount);
        }

        last_found_index = i;
        break;
    }

    return follow_me;
}

void Drive::follow_path(std::vector<CurvePoint> path_points){
    last_found_index = -1;

    // extend path by 12in so robot doesn't oscillate crazily towards the end of the path
    Point extend_point = extend_path(path_points[path_points.size()-2].to_point(), path_points[path_points.size()-1].to_point(), 12);
    path_points.push_back(CurvePoint(extend_point, path_points[path_points.size()-1].drive_voltage, path_points[path_points.size()-1].turn_voltage, path_points[path_points.size()-1].follow_distance, path_points[path_points.size()-1].point_length, path_points[path_points.size()-1].slow_down_turn_radians, path_points[path_points.size()-1].slow_down_turn_amount));


    // figure out where robot is on the path
    for(int i = last_found_index + 1; i < path_points.size()-1; i++){
        if(is_in_segment(Point(get_X_position(), get_Y_position()), path_points[i].to_point(), path_points[i+1].to_point())){
            last_found_index = i - 1; // -1 because loop starts with last_found_index + 1
            break;
        }
    }
    
    // follow the path
    while(last_found_index < path_points.size() - 1){
        Point robot_pos = Point(get_X_position(), get_Y_position(), get_absolute_heading());
        CurvePoint follow_me = get_follow_point(path_points, robot_pos, path_points[last_found_index].follow_distance);
        go_to_point(follow_me.x, follow_me.y, follow_me.drive_voltage);
    }
}