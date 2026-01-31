#include "robot_config.h"
#include "pros/rtos.hpp"  // For pros::Task

// Vertical Tracking Wheel
pros::Rotation rotation_sensor(11);

pros::Imu imu(13);

pros::MotorGroup left_motors({-20, -17, 18}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({19, 16, -15}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450rpm (im pretty sure)
                              2 // horizontal drift is 2 (for now)
);

// tracking wheel configuration
lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_275, -.25);
// vertical wheel, 2.75" diameter, -.25" offset from tracking center

// odometry sensors configuration
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // no second vertical wheel
                            nullptr, // no horizontal tracking wheel
                            nullptr, // no second horizontal wheel
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(5,   // kP
                                              0,   // kI
                                              8,   // kD
                                              3,   // anti windup
                                              1,   // small error range
                                              100, // small error range timeout
                                              3,   // large error range
                                              500, // large error range timeout
                                              127  // slew - START HIGH
);

// angular PID controller - TUNED
lemlib::ControllerSettings angular_controller(1.7, // kP
                                              0,   // kI
                                              14,  // kD
                                              3,   // anti windup - ENABLE
                                              1,   // small error range, in degrees - ENABLE
                                              100, // small error range timeout - ENABLE
                                              3,   // large error range, in degrees - ENABLE
                                              500, // large error range timeout - ENABLE
                                              0    // slew
);

// Add these BEFORE the chassis declaration
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve
);

// --------------------- Motors ---------------------

pros::Motor Intake(INTAKE_PORT, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor Outtake(OUTTAKE_PORT, pros::MotorGears::blue, pros::MotorUnits::degrees);

// --------------------- Sensors ---------------------
pros::adi::DigitalOut Descore('A');
pros::adi::DigitalOut Unloader('C');
pros::adi::DigitalOut MidScoring('B');

// --------------------- Controller ---------------------
pros::Controller master(pros::E_CONTROLLER_MASTER);

// --------------------- Autonomous Selector Logic ---------------------
int autonSelection = 0; // 0=Skills, 1=Left, 2=Right, 3=RightDescore
bool selectorLocked = false;
uint32_t lockTimer = 0; // Timer for auto-lock feature

const char* autonNames[] = {
    "SKILLS",
    "LEFT",
    "RIGHT",
    "R-DESCORE"
};

void drawAutonSelector() {
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::fill_rect(0, 0, 480, 240); // Clear screen
    
    // Draw 4 buttons (2x2 grid)
    for (int i = 0; i < 4; i++) {
        int x = (i % 2) * 240;  // 0 or 240
        int y = (i / 2) * 120;  // 0 or 120
        
        // Highlight selected button
        if (i == autonSelection) {
            pros::screen::set_pen(pros::c::COLOR_GREEN);
        } else {
            pros::screen::set_pen(pros::c::COLOR_BLUE);
        }
        
        pros::screen::fill_rect(x + 5, y + 5, x + 235, y + 115);
        
        // Draw text
        pros::screen::set_pen(pros::c::COLOR_WHITE);
        pros::screen::print(pros::E_TEXT_LARGE, x + 60, y + 50, "%s", autonNames[i]);
    }
}

void drawLockScreen() {
    // Cool lock screen design
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_rect(0, 0, 480, 240);
    
    // Red border
    pros::screen::set_pen(pros::c::COLOR_RED);
    pros::screen::draw_rect(10, 10, 470, 230);
    pros::screen::draw_rect(12, 12, 468, 228);
    pros::screen::draw_rect(14, 14, 466, 226);
    
    // Lock icon
    pros::screen::set_pen(pros::c::COLOR_YELLOW);
    pros::screen::fill_circle(240, 80, 30);
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_circle(240, 80, 20);
    pros::screen::set_pen(pros::c::COLOR_YELLOW);
    pros::screen::fill_rect(220, 80, 260, 120);
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_circle(240, 100, 8);
    
    // Text
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 160, 140, "LOCKED");
    pros::screen::set_pen(pros::c::COLOR_YELLOW);
    pros::screen::print(pros::E_TEXT_MEDIUM, 140, 180, "Auton: %s", autonNames[autonSelection]);
}

void handleScreenTouch() {
    if (selectorLocked) return;
    
    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    
    if (status.touch_status == pros::E_TOUCH_PRESSED) {
        int x = status.x;
        int y = status.y;
        
        // Determine which quadrant was pressed
        if (x < 240 && y < 120) autonSelection = 0; // Top-left: Skills
        else if (x >= 240 && y < 120) autonSelection = 1; // Top-right: Left
        else if (x < 240 && y >= 120) autonSelection = 2; // Bottom-left: Right
        else autonSelection = 3; // Bottom-right: RightDescore
        
        drawAutonSelector();
        pros::delay(200); // Debounce
    }
}

void runAutonSelector(uint32_t timeout_ms) {
    // RESET lock state for new match/run
    selectorLocked = false;
    
    uint32_t startTime = pros::millis();
    drawAutonSelector();
    
    while (true) {
        // Handle input
        handleScreenTouch();
        
        // Exit condition 1: Timeout reached (if not infinite/0)
        if (timeout_ms > 0 && (pros::millis() - startTime > timeout_ms)) {
            break;
        }
        
        // Exit condition 2: Match started (Competition switch)
        if (!pros::competition::is_disabled()) {
            break;
        }

        pros::delay(20);
    }
}

void checkAndLockSelector(uint32_t lockDelay) {
    static bool lockScreenDrawn = false;
    
    // Auto-lock after delay
    if (!selectorLocked && pros::millis() - lockTimer > lockDelay) {
        selectorLocked = true;
        lockScreenDrawn = false;
    }
    
    // Draw lock screen once
    if (selectorLocked && !lockScreenDrawn) {
        drawLockScreen();
        lockScreenDrawn = true;
    }
}

void initializeRobot() {
    // Non-blocking calibration - fixes "Run" mode hang
    chassis.calibrate(false);
    
    // Set brake modes
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);   // Prevents drifting, smooth control
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);  // Prevents drifting, smooth control
    Outtake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);        // Holds position, prevents backdriving
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);         // Prevents backdriving when stopped
    
    Outtake.set_reversed(false);
    Intake.set_reversed(true);
    MidScoring.set_value(false);

    // Descore startup cycle in background
    pros::Task descoreTask([]() {
        pros::delay(500);
        Descore.set_value(true);
        pros::delay(1000);
        Descore.set_value(false);
    });
}