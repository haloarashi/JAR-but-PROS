#include "main.h"

void initialize() {
}

void disabled() {
	odomlift.set_value(false);
	// chassis.set_coordinates(chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
	chassis.set_coordinates(cm_to_inch(-35),cm_to_inch(42.386), 270);
}


void competition_initialize() {
	init();
	claw.set_value(true);
}

void autonomous() {
	default_constants();
	claw.set_value(false);
	chassis.set_coordinates(cm_to_inch(-35),cm_to_inch(42.386), 270);

// 	// std::vector<CurvePoint> path_points = {
// 	// 	CurvePoint(Point(0, 20, 0), 80, 60, 3, 3, 3, to_rad(30), .5),
// 	// 	CurvePoint(Point(25, 20, 90), 60, 60, 3, 3, 3, to_rad(30), .5),
// 	// 	// CurvePoint(Point(20, 20, 0), 127, 40, 3, 3, 3, to_rad(30), .5),
// 	// 	// CurvePoint(Point(25, 30), 10, 40, 3, 3, 3, to_rad(30), .5)
// 	// };
	
	// chassis.follow_path(path_points);

// 	std::vector<CurvePoint> path_points = parse_point_data(R"(-37.69,42.386,113.272,270
// -47.537,44.86,109.455
// -56.928,48.69,105.916
// -65.448,54.167,104.103
// -72.494,61.459,111.849
// -77.97,70.008,117.9
// -82.727,78.986,112.752
// -88.092,87.6,99.97
// -95.184,94.818,99.217
// -104.001,99.806,103.785
// -113.792,102.446,114.085
// -123.866,103.661,117.76
// -134.182,103.932,119.633,270
// -134.182,103.932,0,270
// )");

	std::vector<CurvePoint> path_points = parse_point_data(R"(-55,42.386,120,270
		-30, 42.386,120)");
	
	// Point extend_point = extend_path(path_points[path_points.size()-2].point, path_points[path_points.size()-1].point, 12);
    // path_points.push_back(CurvePoint(extend_point, path_points[path_points.size()-1].drive_voltage, path_points[path_points.size()-1].heading_max_voltage, path_points[path_points.size()-1].follow_distance, path_points[path_points.size()-1].drive_settle_error, path_points[path_points.size()-1].point_length, path_points[path_points.size()-1].slow_down_turn_radians, path_points[path_points.size()-1].slow_down_turn_amount));
	// print_curvepoints(path_points);

	chassis.follow_path(path_points);
	
	chassis.drive_stop(MotorBrake::brake);
}

void opcontrol() {
	while (true) {
		claw.set_value(false);
		chassis.control_arcade();
		delay(10);
	}
}