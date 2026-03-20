#include "main.h"

void initialize() {
}

void disabled() {
	odomlift.set_value(false);
	// chassis.set_coordinates(chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
	chassis.set_coordinates(0, 0, 0);
}


void competition_initialize() {
	init();
	claw.set_value(true);
}

void autonomous() {
	default_constants();
	claw.set_value(false);
	chassis.set_coordinates(0, 0, 0);
	std::vector<CurvePoint> path_points = {
		// CurvePoint(Point(0, 0), 40, 0, 0, 0, 0, 0),
		CurvePoint(Point(0, 10, 0), 30, 20, 3, 3, 3, to_rad(30), .5),
		CurvePoint(Point(25, 15, 10), 80, 20, 3, 3, 3, to_rad(30), .5),
		// CurvePoint(Point(20, 20, 0), 127, 40, 3, 3, 3, to_rad(30), .5),
		// CurvePoint(Point(25, 30), 10, 40, 3, 3, 3, to_rad(30), .5)
	};

	// Point extend_point = extend_path(path_points[path_points.size()-2].to_point(), path_points[path_points.size()-1].to_point(), 12);
    // path_points.push_back(CurvePoint(extend_point, path_points[path_points.size()-1].drive_voltage, path_points[path_points.size()-1].turn_voltage, path_points[path_points.size()-1].follow_distance, path_points[path_points.size()-1].drive_settle_error, path_points[path_points.size()-1].point_length, path_points[path_points.size()-1].slow_down_turn_radians, path_points[path_points.size()-1].slow_down_turn_amount));
	// print_curvepoints(path_points);

	chassis.follow_path(path_points);
}

void opcontrol() {
	while (true) {
		claw.set_value(false);
		chassis.control_arcade();
		delay(10);
	}
}