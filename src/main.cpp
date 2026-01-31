#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "main.h"
#include "robot_config.h"
#include "autonomous.h"
#include "subsystems/intake.h"
#include "subsystems/outtake.h"
#include "subsystems/pneumatics.h"

void initialize() {
    initializeRobot();
    drawAutonSelector(); // Just draw it, don't block
}

void disabled() {}

void competition_initialize() {
    // If connected to field: wait for enable (timeout = 0)
    // If not connected (testing): 15 second timeout
    if (pros::competition::is_connected()) {
        runAutonSelector(0); // Competition mode - waits for field enable
    } else {
        runAutonSelector(15000); // Practice mode - 15 seconds
    }
}

void autonomous() {
    switch(autonSelection) {
        case 0:
            skills_auton();
            break;
        case 1:
            leftAuton();
            break;
        case 2:
            rightAuton();
            break;
        case 3:
            rightAutonDescore();
            break;
    }
}

// Small deadband to prevent drift (applies to values close to 0)
int applyDeadband(int value, int threshold = 8) {
    return (abs(value) < threshold) ? 0 : value;
}

void opcontrol() {
    IntakeControl intake;
    OuttakeControl outtake;
    PneumaticControl pneumatics;
    
    // Reset lock timer when driver control starts
    lockTimer = pros::millis();
    
    // Tracking for warnings (don't spam alerts)
    uint32_t lastTempCheck = 0;
    uint32_t lastBatteryCheck = 0;
    bool lowBatteryWarned = false;

    while (true) {
        // Check and lock selector after 5 seconds
        checkAndLockSelector(5000);

        // Tank Drive with deadband
        int left = applyDeadband(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int right = applyDeadband(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        left_motors.move(left);
        right_motors.move(right);

        // Update subsystems
        outtake.update();
        intake.update(outtake.isMidScoring());
        pneumatics.update();
        
        // === MOTOR TEMPERATURE MONITORING (every 2 seconds) ===
        if (pros::millis() - lastTempCheck > 2000) {
            lastTempCheck = pros::millis();
            
            // Check all motor temperatures (overheat starts at 55°C)
            double maxTemp = 0;
            std::string hotMotor = "";
            
            // Check drive motors (get_temperature returns vector for groups)
            auto leftTemps = left_motors.get_temperature_all();
            auto rightTemps = right_motors.get_temperature_all();
            
            for (double temp : leftTemps) {
                if (temp > maxTemp) { maxTemp = temp; hotMotor = "L-Drive"; }
            }
            for (double temp : rightTemps) {
                if (temp > maxTemp) { maxTemp = temp; hotMotor = "R-Drive"; }
            }
            if (Intake.get_temperature() > maxTemp) { 
                maxTemp = Intake.get_temperature(); hotMotor = "Intake"; 
            }
            if (Outtake.get_temperature() > maxTemp) { 
                maxTemp = Outtake.get_temperature(); hotMotor = "Outtake"; 
            }
            
            // Warn at 50°C (before power reduction kicks in at 55°C)
            if (maxTemp >= 50) {
                master.print(0, 0, "HOT: %s %.0fC   ", hotMotor.c_str(), maxTemp);
            }
        }
        
        // === LOW BATTERY WARNING (10%) ===
        if (pros::millis() - lastBatteryCheck > 5000) {
            lastBatteryCheck = pros::millis();
            
            int batteryLevel = pros::battery::get_capacity();
            if (batteryLevel <= 10 && !lowBatteryWarned) {
                master.rumble("---");  // Long rumble pattern
                master.print(1, 0, "LOW BATTERY: %d%%", batteryLevel);
                lowBatteryWarned = true;
            }
        }

        pros::delay(20);
    }
}