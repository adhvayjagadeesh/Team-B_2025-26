#include "main.h"
#include "manual.h"
#include <cmath>

// ----- TUNING PARAMETERS ----- //
// MOTOR CALIBRATION - Adjust these to balance left/right sides
const double LEFT_SIDE_MULTIPLIER = 1.0;   // Increase this if left is slower (try 1.05-1.15)
const double RIGHT_SIDE_MULTIPLIER = 1.0;   // Keep this at 1.0, adjust left instead

// Deadzones
const int JOYSTICK_DEADZONE = 5;
const int TURN_DEADZONE = 20;

// TURN SENSITIVITY - Lower = less sensitive, Higher = more sensitive
const double TURN_SENSITIVITY = 0.55;  // Reduced from 1.0 (default). Try 0.5-0.9 for less sensitivity

// Minimum power to overcome friction
const int MIN_DRIVE_POWER = 12;  // Increased to help overcome the initial friction difference

// Slew rate
const int ACCEL_STEP = 15;    // Slightly slower to give both sides time to sync
const int DECEL_STEP = 9.8;

// Heading correction
const double HEADING_KP = 0.6;
const double HEADING_KD = 0.15;
const double MAX_CORRECTION = 30.0;

// ----- STATE VARIABLES ----- //
static int prevLeftPower = 0;
static int prevRightPower = 0;

static double referenceHeading = 0;
static double lastHeadingError = 0;
static bool headingLocked = false;

static bool descoreState = false;
static bool switchState = false;
static bool matchLoaderState = false;

static uint32_t lastDebugTime = 0;

// ----- HELPER FUNCTIONS ----- //

static int applyDeadzone(int value, int deadzone) {
    return (std::abs(value) > deadzone) ? value : 0;
}

static int applyMinPower(int value, int minPower) {
    if (value == 0) return 0;
    if (value > 0) return value + minPower;
    return value - minPower;
}

// Apply motor calibration to compensate for mechanical differences
static int calibrateMotor(int power, double multiplier) {
    if (power == 0) return 0;
    int calibrated = (int)std::round(power * multiplier);
    return std::clamp(calibrated, -127, 127);
}

static void applySlewRate(int leftTarget, int rightTarget, int &prevLeft, int &prevRight) {

    int leftDelta = leftTarget - prevLeft;
    int rightDelta = rightTarget - prevRight;

    bool accelerating = (std::abs(leftTarget) > std::abs(prevLeft)) ||
                       (std::abs(rightTarget) > std::abs(prevRight));

    int maxStep = accelerating ? ACCEL_STEP : DECEL_STEP;

    int maxDelta = std::max(std::abs(leftDelta), std::abs(rightDelta));

    if (maxDelta > maxStep) {
        double scale = (double)maxStep / (double)maxDelta;
        leftDelta = (int)std::round(leftDelta * scale);
        rightDelta = (int)std::round(rightDelta * scale);
    }

    prevLeft += leftDelta;
    prevRight += rightDelta;

    prevLeft = std::clamp(prevLeft, -127, 127);
    prevRight = std::clamp(prevRight, -127, 127);
}

// ----- DRIVER CONTROL ----- //
void runOpcontrol() {

    prevLeftPower = 0;
    prevRightPower = 0;
    headingLocked = false;
    lastHeadingError = 0;

    referenceHeading = imu.get_heading();

    while (true) {

        // ----- READ CONTROLLER INPUT ----- //
        int rawForward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rawTurn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int forward = applyDeadzone(rawForward, JOYSTICK_DEADZONE);
        int turn = applyDeadzone(rawTurn, TURN_DEADZONE);

        // Apply turn sensitivity
        turn = (int)(turn * TURN_SENSITIVITY);

        forward = applyMinPower(forward, MIN_DRIVE_POWER);

        // ----- HEADING CORRECTION ----- //
        double correction = 0;

        if (turn == 0 && forward != 0) {

            if (!headingLocked) {
                referenceHeading = imu.get_heading();
                headingLocked = true;
                lastHeadingError = 0;
            }

            double currentHeading = imu.get_heading();
            double error = referenceHeading - currentHeading;

            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double derivative = error - lastHeadingError;
            correction = (error * HEADING_KP) + (derivative * HEADING_KD);
            correction = std::clamp(correction, -MAX_CORRECTION, MAX_CORRECTION);

            lastHeadingError = error;

        } else {
            headingLocked = false;
            lastHeadingError = 0;
        }

        // ----- CALCULATE MOTOR POWERS ----- //
        int leftPower = (int)(forward + turn + correction);
        int rightPower = (int)(forward - turn - correction);

        // Apply slew rate limiting
        applySlewRate(leftPower, rightPower, prevLeftPower, prevRightPower);

        // **MOTOR CALIBRATION** - Compensate for mechanical differences
        int calibratedLeft = calibrateMotor(prevLeftPower, LEFT_SIDE_MULTIPLIER);
        int calibratedRight = calibrateMotor(prevRightPower, RIGHT_SIDE_MULTIPLIER);

        // Send to motors
        leftMotors.move(calibratedLeft);
        rightMotors.move(calibratedRight);

        // ----- SUBSYSTEMS ----- //

        // R1: Only conveyer (intake)
        // R2: Conveyer and outtake
        // ----- CONVEYOR / OUTTAKE CONTROL -----
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            // conveyer reverse only
            intakeMotor.move(127);   // reversed
            outTakeMotor.move(0);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            // outtake reverse only
            intakeMotor.move(0);
            outTakeMotor.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // Conveyor forward only
            intakeMotor.move(-127);
            outTakeMotor.move(0);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            // Conveyor + outtake
            intakeMotor.move(-127);
            outTakeMotor.move(127);
        }
        else {
            // Stop both
            intakeMotor.move(0);
            outTakeMotor.move(0);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            matchLoaderState = !matchLoaderState;
            matchLoader.set_value(matchLoaderState);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            switchState = !switchState;
            switchPiston.set_value(switchState);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            descoreState = !descoreState;
            descore.set_value(descoreState);
        }

        // Manual heading reset
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            referenceHeading = imu.get_heading();
            controller.rumble(".");
        }

        // ----- DEBUG OUTPUT ----- //
        uint32_t now = pros::millis();
        if (now - lastDebugTime > 150) {
            lastDebugTime = now;

            pros::lcd::print(3, "Raw: F=%4d T=%4d", rawForward, rawTurn);
            pros::lcd::print(4, "Pwr: L=%4d(%4d) R=%4d(%4d)",
                           prevLeftPower, calibratedLeft, prevRightPower, calibratedRight);
            pros::lcd::print(5, "Hdg: %.1f | Ref: %.1f | Corr: %.1f",
                           imu.get_heading(), referenceHeading, correction);
        }

        pros::delay(20);
    }
}
