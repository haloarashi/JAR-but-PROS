#include "main.h"

void initialize() {
}

void disabled() {
	// chassis.set_coordinates(chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
	chassis.set_coordinates(cm_to_inch(-124.753),cm_to_inch(39.548), 0);
	// chassis.set_coordinates(0, 0, 0);
}


void competition_initialize() {
	init();
	claw.set_value(true);
}

void autonomous() {
	default_constants();
	claw.set_value(false);
	chassis.set_coordinates(cm_to_inch(-124.753),cm_to_inch(39.548), 0);
	// chassis.set_coordinates(0, 0, 0);

	// CurvePoint(Point point, float drive_voltage, float heading_max_voltage, float follow_distance, float drive_settle_error, float point_length, float slow_down_turn_radians, float slow_down_turn_amount)

	// std::vector<CurvePoint> path_points = {
	// 	CurvePoint(Point(0, 20, 0), 80, 60, 3, 3, 3, to_rad(30), .5),
	// 	CurvePoint(Point(25, 20, 90), 60, 60, 3, 3, 3, to_rad(30), .5),
	// 	// CurvePoint(Point(20, 20, 0), 127, 40, 3, 3, 3, to_rad(30), .5),
	// 	// CurvePoint(Point(25, 30), 10, 40, 3, 3, 3, to_rad(30), .5)
	// };

	std::vector<CurvePoint> path_points = parse_point_data(R"(-124.753,39.548,64.519,0
-124.578,44.624,66.163
-124.586,49.704,68.304
-124.649,54.784,68.351
-124.701,59.863,67.829
-124.682,64.943,66.932
-124.561,70.022,66.52
-124.297,75.094,65.714
-123.849,80.154,65.305
-123.207,85.193,64.889
-122.315,90.193,64.035
-121.144,95.135,63.607
-119.69,100.001,63.19
-117.913,104.758,62.454
-115.766,109.36,62.172
-113.279,113.786,61.973
-110.451,118.003,61.876
-107.279,121.969,62.027
-103.763,125.633,62.273
-99.966,129.004,62.616
-95.921,132.072,63.036
-91.663,134.838,63.507
-87.228,137.309,64.005
-82.648,139.501,64.506
-77.952,141.433,64.994
-73.164,143.129,65.454
-68.299,144.591,65.454
-63.37,145.817,65.878
-58.401,146.869,66.263
-53.403,147.773,66.606
-48.376,148.502,66.606
-43.328,149.064,66.91
-38.27,149.533,67.176
-33.2,149.847,67.176
-28.125,150.053,67.408
-23.047,150.182,67.408
-18.766,150.21,67.486,90
-18.766,150.21,0,90)");

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