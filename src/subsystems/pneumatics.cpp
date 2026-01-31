#include "subsystems/pneumatics.h"
#include "robot_config.h"

PneumaticControl::PneumaticControl()
    : descoreState(false), unloaderState(false),
      A_lastState(false), B_lastState(false) {}

void PneumaticControl::update() {
    // Descore (Button A)
    bool A_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    if (A_current && !A_lastState) {
        descoreState = !descoreState;
        Descore.set_value(descoreState);
    }
    A_lastState = A_current;

    // Unloader (Button B)
    bool B_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (B_current && !B_lastState) {
        unloaderState = !unloaderState;
        Unloader.set_value(unloaderState);
    }
    B_lastState = B_current;
}

bool PneumaticControl::getDescoreState() {
    return descoreState;
}

bool PneumaticControl::getUnloaderState() {
    return unloaderState;
}