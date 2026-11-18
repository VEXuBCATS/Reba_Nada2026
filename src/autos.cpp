#include "main.h"
#include "robotConfigs.h"
#include "autos.h"
#include "constants.h"

AutoMode autoMode = AutoMode::auto1;

//Se importan los txt de las rutas
ASSET(rutaPrueba1_txt);
ASSET(rutaPrueba2_txt);
ASSET(path1_txt);
ASSET(path2_txt);


void auto2() {
	chassis.setPose(0, -64, 0);
	chassis.follow(path1_txt, 18, 4000);
	chassis.turnToHeading(180, 750);

	elevator.move_absolute(5000, 200);
	pros::delay(1500);
	chassis.moveToPoint(0, 0, 1000, {.maxSpeed = 80}, false);

	chassis.moveToPoint(0, 24, 1000, {.forwards = false, .maxSpeed = 80}, true);
	elevator.move_absolute(500, 200);
	pros::delay(1500);

	chassis.turnToHeading(135, 750);
	chassis.follow(path2_txt, 18, 4000);

	chassis.turnToHeading(0, 750);
	chassis.moveToPoint(0, -64, 3000, {.forwards = false, .maxSpeed = 80}, false);
}

void auto1() {
	chassis.setPose(0, -64, 0);
	chassis.follow(rutaPrueba2_txt, 18, 6500);
	chassis.moveToPoint(0, -64, 2000);
	chassis.turnToHeading(0, 1000);
	chassis.moveToPoint(0, -64, 2000, {.forwards = false});
}

void autoTest() {
	chassis.setPose(0, 0, 0);
	//claw.set_zero_position(-1000);

	chassis.moveToPoint(0, 32, 1250, {.maxSpeed = 80}, true);
	elevator.move_absolute(2300, 200);
	pros::delay(1000);

	//claw.move_absolute(-1000, 100);  
	pros::delay(1000);

	
	chassis.moveToPoint(0, 15, 1250, {.forwards = false, .maxSpeed = 80}, true);
	elevator.move_absolute(200, 200);
	pros::delay(1000);
	//pros::delay(500);
	chassis.turnToHeading(40, 750, {.maxSpeed = 80});
	//pros::delay(500);
	chassis.moveToPoint(11.5, 22.5, 1250, {.maxSpeed = 80}, false);

	chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 750, {.maxSpeed = 80});

	chassis.moveToPoint(11.5, 56.5, 1250, {.maxSpeed = 80}, false);

	chassis.turnToHeading(-90, 750, {.maxSpeed = 80});

	chassis.moveToPoint(-19, 56.5, 1250, {.maxSpeed = 80}, false);

	chassis.turnToHeading(-180, 750, {.maxSpeed = 80});

	chassis.moveToPoint(-19, 25, 1250, {.maxSpeed = 80}, false);

	chassis.moveToPoint(0, 0, 1250, {.maxSpeed = 80}, false);

	chassis.turnToHeading(0, 750, {.maxSpeed = 80});

	chassis.moveToPoint(0, 0, 1250, {.forwards = false, .maxSpeed = 80}, false);
}

void autoOffset() {
    horizontal_encoder.reset_position();
    chassis.turnToHeading(1800, 4000, {.maxSpeed = 80}, false);
    double horizontalTicks = horizontal_encoder.get_position();
    double verticalTicks = vertical_encoder.get_position();

    double radiansTurned = 1800 * (M_PI / 180.0);

    double inchesPerRev = trackingWheelDiameter * M_PI;

    double horizontalInches = (horizontalTicks / 360.0) * inchesPerRev;
    double verticalInches = (verticalTicks / 360.0) * inchesPerRev;
    double horizontalOffset = horizontalInches / radiansTurned;
    double verticalOffset = verticalInches / radiansTurned;

    pros::lcd::print(2, "Offset Horizontal: %f", horizontalOffset);
    pros::lcd::print(3, "Offset Vertical: %f", verticalOffset);

    pros::delay(1000); // Wait for 5 seconds to read the values
}