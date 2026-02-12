#include "main.h"
#include "auton.h"

void startIntake() {
    intakeMotor.move(-127);
    //outTakeMotor.move(127);
}

void stopIntake() {
    intakeMotor.move(0);
    outTakeMotor.move(0);
}

void startOuttake() {
    outTakeMotor.move(127);
    intakeMotor.move(-127);
}

void stopOuttake() {
    outTakeMotor.move(0);
    intakeMotor.move(0);
}

void runAutonomous() {
    chassis.setPose(48.5, -10.2, 246);
    switchPiston.set_value(true);
    matchLoader.set_value(false);

    // Turn using IMU heading (consistent without tracking wheels)
    chassis.turnToHeading(251, 1000);
    startIntake();
    chassis.moveToPoint(15, -22, 3000, {});
    pros::delay(1500);

    chassis.turnToHeading(149, 1500);
    chassis.moveToPoint(35.9, -48.3, 2000);
    stopIntake();

    chassis.turnToHeading(88, 1500);
    chassis.waitUntilDone();
    matchLoader.set_value(true);
    //chassis.moveToPoint(55, -52, 3000);

    chassis.moveToPoint(22.6, -45, 2500, {.forwards = false, .maxSpeed = 80, .minSpeed = 10});
    chassis.waitUntilDone();
    startOuttake();
    pros::delay(2000);
    stopOuttake();

    startIntake();
    chassis.moveToPoint(51.9, -48.7, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(51.9, -48, 2000);
    chassis.moveToPoint(51.9, -48.7, 2000);
    chassis.moveToPoint(51.9, -48, 2000);
    pros::delay(2000);
    stopIntake();


}
