#include "main.h"

void initialize() {
}

void disabled() {
	// chassis.set_coordinates(chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
	// chassis.set_coordinates(cm_to_inch(-124.753),cm_to_inch(39.548), 0);
	chassis.set_coordinates(0, 0, 0);
}


void competition_initialize() {
	init();
	claw.set_value(true);
}

void autonomous() {
	default_constants();
	claw.set_value(false);
	// chassis.set_coordinates(cm_to_inch(-124.753),cm_to_inch(39.548), 0);
	chassis.set_coordinates(0, 0, 0);

	// CurvePoint(Point point, float drive_voltage, float heading_max_voltage, float follow_distance, float drive_settle_error, float point_length, float slow_down_turn_radians, float slow_down_turn_amount)

	// std::vector<CurvePoint> path_points = {
	// 	CurvePoint(Point(0, 20, 0), 80, 60, 3, 3, 3, to_rad(30), .5),
	// 	CurvePoint(Point(25, 20, 90), 60, 60, 3, 3, 3, to_rad(30), .5),
	// 	// CurvePoint(Point(20, 20, 0), 127, 40, 3, 3, 3, to_rad(30), .5),
	// 	// CurvePoint(Point(25, 30), 10, 40, 3, 3, 3, to_rad(30), .5)
	// };

	std::vector<CurvePoint> path_points = parse_point_data(R"(0,0,31.557,0
0.36,5.067,31.231
0.67,10.138,30.463
0.921,15.212,29.694
1.101,20.288,28.926
1.195,25.367,28.157
1.19,30.447,27.389
1.071,35.526,26.62
0.821,40.599,25.852
0.42,45.663,25.083
-0.15,50.71,24.315
-0.911,55.732,23.546
-1.887,60.717,22.778
-3.099,65.649,22.009
-4.613,70.498,21.241
-6.416,75.245,20.472
-8.525,79.864,19.704
-10.954,84.324,18.935
-13.752,88.562,18.167
-16.88,92.561,17.398
-20.314,96.301,16.63
-24.032,99.759,16.886
-28.018,102.906,17.143
-32.237,105.734,17.4
-36.636,108.27,17.657
-41.186,110.525,17.914
-45.858,112.516,18.171
-50.627,114.262,18.428
-55.473,115.784,18.685
-60.378,117.103,18.942
-65.329,118.237,19.199
-70.323,119.169,19.456
-75.342,119.952,19.713
-80.379,120.605,19.969
-85.43,121.145,20.226
-90.495,121.534,20.483
-95.566,121.819,20.74
-100.642,122.028,20.997
-105.721,122.12,21.254
-110.725,122.155,21.254,270
-110.725,122.155,0,270)");

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