#include "main.h"
#include "auton.h"
#include "manual.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue);

// Inertial Sensor on port 10
pros::Imu imu(10);
pros::adi::DigitalOut switchPiston('B');
pros::adi::DigitalOut matchLoader('A');
pros::adi::DigitalOut descore('E');
pros::Motor intakeMotor(20);
pros::Motor outTakeMotor(9);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10,
                              lemlib::Omniwheel::NEW_325, 360, 2);

// lateral motion controller
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);

// angular motion controller
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// input curves for driver control
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// get a path used for pure pursuit
ASSET(example_txt);

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "throwing ant and korey in my private dungeon....");

    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(20);
    }

    pros::lcd::set_text(1, "oiled up and ready to have fun with!");

    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    outTakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.calibrate();

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    runAutonomous();
}

void opcontrol() {
    autonomous(); // run autonomous during driver control for testing purposes
    runOpcontrol();
}
