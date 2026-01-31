#pragma once
#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep

// Motor ports
constexpr int INTAKE_PORT = 21;
constexpr int OUTTAKE_PORT = 12;

// Declare all hardware (using extern so they're defined once in .cpp)
extern pros::Rotation rotation_sensor;
extern pros::Imu imu;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

extern lemlib::Drivetrain drivetrain;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;

extern pros::Motor Intake;
extern pros::Motor Outtake;

extern pros::adi::DigitalOut Descore;
extern pros::adi::DigitalOut Unloader;
extern pros::adi::DigitalOut MidScoring;

extern pros::Controller master;

// Initialization function
void initializeRobot();

// --------------------- Autonomous Selector ---------------------
extern int autonSelection;
extern const char* autonNames[];
extern bool selectorLocked;
extern uint32_t lockTimer;

void drawAutonSelector();
void drawLockScreen();
void handleScreenTouch();
void runAutonSelector(uint32_t timeout_ms);
void checkAndLockSelector(uint32_t lockDelay);