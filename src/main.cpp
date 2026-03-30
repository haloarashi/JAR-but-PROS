#include "main.h"

void initialize() {
}

void disabled() {
	// chassis.set_coordinates(chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
	// chassis.set_coordinates(cm_to_inch(-35),cm_to_inch(42.386), 270);
	chassis.set_coordinates(0, 0, 0);
}


void competition_initialize() {
	init();
	claw.set_value(true);
}

void autonomous() {
	default_constants();
	claw.set_value(false);
	// chassis.set_coordinates(cm_to_inch(-30),cm_to_inch(42.386), 270);
	chassis.set_coordinates(0, 0, 0);

	// CurvePoint(Point point, float drive_voltage, float heading_max_voltage, float follow_distance, float drive_settle_error, float point_length, float slow_down_turn_radians, float slow_down_turn_amount)

	// std::vector<CurvePoint> path_points = {
	// 	CurvePoint(Point(0, 20, 0), 80, 60, 3, 3, 3, to_rad(30), .5),
	// 	CurvePoint(Point(25, 20, 90), 60, 60, 3, 3, 3, to_rad(30), .5),
	// 	// CurvePoint(Point(20, 20, 0), 127, 40, 3, 3, 3, to_rad(30), .5),
	// 	// CurvePoint(Point(25, 30), 10, 40, 3, 3, 3, to_rad(30), .5)
	// };

	std::vector<CurvePoint> path_points = parse_point_data(R"(0,0,73.1,0
0.071,5.08,73.1
0.262,10.155,73.1
0.608,15.223,72.849
1.156,20.272,71.867
1.965,25.286,70.553
3.171,30.218,66.567
4.89,34.994,63.781
7.412,39.394,57.257
10.997,42.962,52.924
15.493,45.276,61.485
20.426,46.462,73.1
25.45,47.213,70.917
30.402,48.324,59.07
34.961,50.517,52.668
38.629,53.994,59.324
41.236,58.339,65.597
43.051,63.08,68.046
44.265,68.01,71.424
45.115,73.016,72.518
45.689,78.063,73.1
46.049,83.129,73.1
46.246,88.204,73.1
46.337,93.751,73.1,0
46.337,93.751,0,0)");

	// std::vector<CurvePoint> path_points = parse_point_data(R"(-55,42.386,120,270
	// 	-30, 42.386,120)");
	
	// Point extend_point = extend_path(path_points[path_points.size()-2].point, path_points[path_points.size()-1].point, 12);
    // path_points.push_back(CurvePoint(extend_point, path_points[path_points.size()-1].drive_voltage, path_points[path_points.size()-1].heading_max_voltage, path_points[path_points.size()-1].follow_distance, path_points[path_points.size()-1].drive_settle_error, path_points[path_points.size()-1].point_length, path_points[path_points.size()-1].slow_down_turn_radians, path_points[path_points.size()-1].slow_down_turn_amount));
	// print_curvepoints(path_points);

	map_curvepoints(path_points);
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