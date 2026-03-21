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

	std::vector<CurvePoint> path_points = parse_point_data(R"(-37.69,42.386,115.031,270
-42.655,43.458,113.272
-47.546,44.825,111.389
-52.332,46.522,109.455
-56.968,48.592,105.916
-61.359,51.141,104.607
-65.456,54.137,103.619
-69.181,57.586,105.247
-72.484,61.443,109.223
-75.381,65.614,111.849
-77.96,69.99,117.9
-80.35,74.473,123.844
-82.714,78.969,116.494
-85.227,83.383,109.069
-88.06,87.597,102.474
-91.346,91.464,98.289
-95.138,94.835,97.927
-99.383,97.613,101.259
-103.967,99.793,103.785
-108.79,101.378,109.248
-113.734,102.537,111.798
-118.756,103.291,116.071
-123.816,103.731,117.76
-128.891,103.926,119.173
-134.182,103.932,119.633,270
-134.182,103.932,0,270
)");
	
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