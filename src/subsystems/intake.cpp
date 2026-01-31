#include "subsystems/intake.h"
#include "robot_config.h"

IntakeControl::IntakeControl() 
    : toggleForward(false), toggleReverse(false),
      R1_lastState(false), R2_lastState(false) {}

void IntakeControl::update(bool isBlocked) {
    // Only allow R1/R2 control when NOT in mid-scoring mode
    // We check this via the isBlocked flag passed from Outtake
    
    bool R1_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool R2_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    if (R1_current && !R1_lastState) {
        toggleReverse = !toggleReverse;
        if (toggleReverse) toggleForward = false;
    }
    if (R2_current && !R2_lastState) {
        toggleForward = !toggleForward;
        if (toggleForward) toggleReverse = false;
    }
    
    R1_lastState = R1_current;
    R2_lastState = R2_current;

    // Only move intake here if NOTblocked by outtake (mid-scoring)
    if (!isBlocked) {
        Intake.move(getPower());
    }
}

int IntakeControl::getPower() {
    if (toggleForward) return 127;
    if (toggleReverse) return -127;
    return 0;
}