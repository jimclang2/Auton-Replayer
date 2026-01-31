#include "subsystems/outtake.h"
#include "robot_config.h"

OuttakeControl::OuttakeControl()
    : toggleForward(false), toggleReverse(false),
      L1_lastState(false), L2_lastState(false),
      midScoringMode(false), X_lastState(false),
      unjamStartTime(0), isUnjamming(false) {}

void OuttakeControl::update() {
    // Handle unjam sequence
    if (isUnjamming) {
        if (pros::millis() - unjamStartTime >= 225) {  // Time of delay
            isUnjamming = false;
            // Don't set intake here - let it fall through to mid-scoring mode logic below
        } else {
            Intake.move(127); // Unjam (reverse)
            Outtake.move(-127); // Make sure outtake also runs during unjam
            return; // Exit early during unjam
        }
    }

    // Mid Scoring Toggle (Button X)
    bool X_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (X_current && !X_lastState) {
        midScoringMode = !midScoringMode;
        
        if (midScoringMode) {
            // ENTERING mid-scoring mode
            MidScoring.set_value(true); // Retract piston
            isUnjamming = true;
            unjamStartTime = pros::millis();
        } else {
            // EXITING mid-scoring mode
            MidScoring.set_value(false); // Extend piston
            isUnjamming = false;
            Intake.move(0); // Stop intake
            Outtake.move(0); // Stop outtake
            // Reset toggles so they start fresh
            toggleForward = false;
            toggleReverse = false;
            X_lastState = X_current; // Update state BEFORE returning
            return; // Exit function immediately after turning off mid-scoring
        }
    }
    X_lastState = X_current;

    int power = 0;

    if (midScoringMode && !isUnjamming) {
        // Mid-scoring mode AFTER unjam: run intake forward and outtake reverse
        Intake.move(-127);  // This will run continuously during mid-scoring
        power = -127;
        // Update last states to prevent "stored" button presses
        L1_lastState = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        L2_lastState = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    } else if (!midScoringMode) {
        // Normal mode: L1/L2 control outtake
        bool L1_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool L2_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (L1_current && !L1_lastState) {
            toggleForward = !toggleForward;
            if (toggleForward) toggleReverse = false;
        }
        if (L2_current && !L2_lastState) {
            toggleReverse = !toggleReverse;
            if (toggleReverse) toggleForward = false;
        }
        
        L1_lastState = L1_current;
        L2_lastState = L2_current;

        if (toggleForward) power = 127;
        else if (toggleReverse) power = -127;
    }

    Outtake.move(power);
}

int OuttakeControl::getPower() {
    if (midScoringMode) return -127;
    if (toggleForward) return 127;
    if (toggleReverse) return -127;
    return 0;
}

bool OuttakeControl::isMidScoring() {
    return midScoringMode;
}