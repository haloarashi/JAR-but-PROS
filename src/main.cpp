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
		CurvePoint(Point(10, 10, -25), 80, 40, 3, 3, to_rad(30), .5),
		CurvePoint(Point(10, 15, 10), 40, 40, 3, 3, to_rad(30), .5),
		CurvePoint(Point(20, 20, 0), 127, 40, 3, 3, to_rad(30), .5),
		CurvePoint(Point(25, 30), 10, 40, 3, 3, to_rad(30), .5)
	};

	chassis.follow_path(path_points);
	// chassis.go_to_point(10, 10, 80);
}

void opcontrol() {
	while (true) {
		claw.set_value(false);
		chassis.control_arcade();
		delay(10);
	}
}