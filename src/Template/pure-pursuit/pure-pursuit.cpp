#include "Template/drive.h"

// void Drive::go_to_point(PID headingPID, float X_position, float Y_position, float drive_voltage, float heading_max_voltage, float drive_settle_error){
//     drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position()); // keep this so other parts of the code can access drive_error correctly
    
//     float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
//     float heading_scale_factor = cos(to_rad(heading_error));
    
//     // heading_error = reduce_negative_90_to_90(heading_error);
//     float heading_output = headingPID.compute(heading_error);
    
//     float drive_output = drive_voltage;
    
//     if (drive_error<drive_settle_error) {
//         heading_output = 0;
//     }
//     else{
//         drive_output *= heading_scale_factor;
//     }

//     drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
//     heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
//     // drive_output = clamp_min_voltage(drive_output, drive_min_voltage);
//     if(fabs(drive_output) < drive_min_voltage){
//         drive_output = 10;
//     }

//     // printf("drive_min_voltage: %.0f, drive_output: %.0f, heading_output: %.0f\n", drive_min_voltage, drive_output, heading_output);

//     drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
// }

// void Drive::follow(std::vector<CurvePoint> path_points, float lookahead, int timeout, bool forwards, bool async) {
//     Point pose = this->getPose(true);
//     Point lastPose = pose;
//     Point lookaheadPose(0, 0, 0);
//     Point lastLookahead = pathPoints.at(0);
//     lastLookahead.theta = 0;
//     float curvature;
//     float targetVel;
//     float prevLeftVel = 0;
//     float prevRightVel = 0;
//     int closestPoint;
//     float leftInput = 0;
//     float rightInput = 0;
//     float prevVel = 0;
//     int compState = pros::competition::get_status();

//     // loop until the robot is within the end tolerance
//     for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState && this->motionRunning; i++) {
//         // get the current position of the robot
//         pose = this->getPose(true);
//         if (!forwards) pose.theta -= M_PI;

//         // update completion vars
//         lastPose = pose;
//         lastLookahead = lookaheadPose; // update last lookahead position

//         // // get the curvature of the arc between the robot and the lookahead point
//         // float curvatureHeading = M_PI / 2 - pose.theta;
//         // curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

//         // // get the target velocity of the robot
//         // targetVel = pathPoints.at(closestPoint).theta;
//         // targetVel = slew(targetVel, prevVel, lateralSettings.slew);
//         // prevVel = targetVel;

//         // // calculate target left and right velocities
//         // float targetLeftVel = follow_me.drive_voltage * (2 + curvature * ForwardTracker_center_distance) / 2;
//         // float targetRightVel = follow_me.drive_voltage * (2 - curvature * ForwardTracker_center_distance) / 2;

//         // // ratio the speeds to respect the max speed
//         // float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / 127;
//         // if (ratio > 1) {
//         //     targetLeftVel /= ratio;
//         //     targetRightVel /= ratio;
//         // }

//         // update previous velocities
//         prevLeftVel = targetLeftVel;
//         prevRightVel = targetRightVel;

//         // move DT

//         pros::delay(10);
//     }
// }
float prevVel = 0;
void Drive::go_to_point(Point robot_pos, CurvePoint follow_me){
    double curvature = -findLookaheadCurvature(robot_pos, robot_pos.heading, follow_me.point);
    
    // get the target velocity of the robot
    float targetVel = follow_me.drive_voltage;
    // targetVel = slew(targetVel, prevVel, 5, 0);
    prevVel = targetVel;
    
    double targetLeftVel = follow_me.drive_voltage * (2 - curvature * ForwardTracker_center_distance) / 2;
    double targetRightVel = follow_me.drive_voltage * (2 + curvature * ForwardTracker_center_distance) / 2;
    
    
    // if(fabs(targetLeftVel) > drive_max_voltage || fabs(targetRightVel) > drive_max_voltage){
    //     float ratio = std::max(fabs(targetLeftVel), fabs(targetRightVel)) / drive_max_voltage;
    //     targetLeftVel /= ratio;
    //     targetRightVel /= ratio;
    // }
    
    printf("last_found_index: %d, curvature: %.2f, targetLeftVel: %.2f, targetRightVel: %.2f\n", last_found_index, curvature, targetLeftVel, targetRightVel);
    
    chassis.drive_with_voltage(targetLeftVel, targetRightVel);
}

CurvePoint Drive::get_follow_point(std::vector<CurvePoint> path_points, Point robot_pos, float follow_radius){
    CurvePoint follow_me = CurvePoint(path_points[last_found_index]);
    
    for(int i = last_found_index; i < path_points.size()-1; i++){ // -1 is because we need two points per loop. There is no second -1 (to form a -2) because the last appended extra point must be chosen as a target, just not drive to it. 
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

        break;
    }

    return follow_me;
}

void Drive::follow_path(std::vector<CurvePoint> path_points){
    last_found_index = 0;

    // Extend path by 12in so robot doesn't oscillate crazily towards the end of the path
    Point extend_point = extend_path(path_points[path_points.size()-2].point, path_points[path_points.size()-1].point, 12);
    path_points.push_back(CurvePoint(extend_point, path_points[path_points.size()-1].drive_voltage, path_points[path_points.size()-1].heading_max_voltage, path_points[path_points.size()-1].follow_distance, path_points[path_points.size()-1].drive_settle_error, path_points[path_points.size()-1].point_length, path_points[path_points.size()-1].slow_down_turn_radians, path_points[path_points.size()-1].slow_down_turn_amount));

    // Figure out where robot is on the path when starting to follow it
    for(int i = last_found_index; i < path_points.size()-2; i++){
        if(is_in_segment(Point(get_X_position(), get_Y_position()), path_points[i].point, path_points[i+1].point)){
            last_found_index = i;
            break;
        }
    }

    // Follow the path
    // PID headingPID(0, heading_kp, heading_ki, heading_kd, heading_starti);
    while(last_found_index < path_points.size() - 2){ // while we're not following the last point
        Point robot_pos = Point(get_X_position(), get_Y_position(), get_absolute_heading());
        CurvePoint follow_me = get_follow_point(path_points, robot_pos, path_points[last_found_index + 1].follow_distance); // last_found_index + 1 is the next point (the point we are following)
        
        go_to_point(robot_pos, follow_me);
        // go_to_point(headingPID, follow_me.point.x, follow_me.point.y, follow_me.drive_voltage, follow_me.heading_max_voltage, follow_me.drive_settle_error);
        
        if(is_past_segment(robot_pos, path_points[last_found_index].point, path_points[last_found_index + 1].point)){
            last_found_index++;
        }

        delay(10);
    }
}