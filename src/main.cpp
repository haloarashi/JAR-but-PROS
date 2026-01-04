#include "main.h"

void initialize() {
	Task screen_task(simple_screen_task);
}


void disabled() {}


void competition_initialize() {

}

void autonomous() {
	
}

void opcontrol() {
	while (true) {
		chassis.control_arcade();
		delay(10);
	}
}