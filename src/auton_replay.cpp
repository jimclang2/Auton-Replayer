#include "auton_replay.h"
#include "robot_config.h"
#include <cstdio>
#include <cmath>

// Global instance
AutonReplay autonReplay;

// Helper to pack button states into a single byte
static uint8_t packButtons() {
    uint8_t buttons = 0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) buttons |= (1 << BTN_R1);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) buttons |= (1 << BTN_R2);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) buttons |= (1 << BTN_L1);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) buttons |= (1 << BTN_L2);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))  buttons |= (1 << BTN_X);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))  buttons |= (1 << BTN_A);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))  buttons |= (1 << BTN_B);
    return buttons;
}

// Helper to check if a button was just pressed (edge detection)
static bool wasPressed(uint8_t current, uint8_t prev, uint8_t bit) {
    return (current & (1 << bit)) && !(prev & (1 << bit));
}

void AutonReplay::startRecording() {
    recording.clear();
    recordStartTime = pros::millis();
    _isRecording = true;
    
    // Reset IMU heading to 0 at start of recording for consistent reference
    imu.set_heading(0);
    
    master.print(0, 0, "RECORDING...       ");
    master.rumble("-");  // Short vibration to confirm
    
    drawStatusIndicator();
}

void AutonReplay::stopRecording(bool saveToSD) {
    _isRecording = false;
    
    master.print(0, 0, "STOPPED: %d frames ", recording.size());
    master.rumble(".");  // Confirm vibration
    
    if (saveToSD) {
        if (this->saveToSD()) {
            master.print(1, 0, "SAVED TO SD!       ");
        } else {
            master.print(1, 0, "SD SAVE FAILED!    ");
        }
    }
    
    drawStatusIndicator();
}

void AutonReplay::recordFrame() {
    if (!_isRecording) return;
    
    RecordedFrame frame;
    frame.timestamp = pros::millis() - recordStartTime;
    frame.leftStick = static_cast<int8_t>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    frame.rightStick = static_cast<int8_t>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    frame.heading = imu.get_heading();  // Record heading for drift correction
    frame.buttons = packButtons();
    
    recording.push_back(frame);
    
    // Blink indicator every 500ms
    if ((frame.timestamp / 500) % 2 == 0) {
        pros::screen::set_pen(pros::c::COLOR_RED);
    } else {
        pros::screen::set_pen(pros::c::COLOR_DARK_RED);
    }
    pros::screen::fill_circle(460, 20, 15);
}

void AutonReplay::applyHeadingCorrection(int& left, int& right, float targetHeading, float currentHeading) {
    // Calculate heading error (account for wrap-around at 360)
    float error = targetHeading - currentHeading;
    
    // Normalize error to -180 to 180 range
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    // Apply proportional correction
    float correction = error * imuCorrectionGain;
    
    // Clamp correction to prevent overcorrection
    if (correction > 30) correction = 30;
    if (correction < -30) correction = -30;
    
    // Apply correction (positive error = robot is too far right, need to turn left)
    left = static_cast<int>(left - correction);
    right = static_cast<int>(right + correction);
    
    // Clamp final values to valid motor range
    if (left > 127) left = 127;
    if (left < -127) left = -127;
    if (right > 127) right = 127;
    if (right < -127) right = -127;
}

void AutonReplay::playback() {
    if (recording.empty()) {
        // Try loading from SD card
        if (!loadFromSD()) {
            master.print(0, 0, "NO RECORDING!      ");
            return;
        }
    }
    
    _isPlaying = true;
    prevButtons = 0;
    
    // Reset IMU heading to match the recording start
    imu.set_heading(0);
    pros::delay(50);  // Brief delay to let IMU settle
    
    uint32_t playStartTime = pros::millis();
    size_t frameIndex = 0;
    
    master.print(0, 0, "REPLAYING...       ");
    
    // Draw green indicator
    pros::screen::set_pen(pros::c::COLOR_GREEN);
    pros::screen::fill_circle(460, 20, 15);
    
    while (frameIndex < recording.size()) {
        uint32_t elapsed = pros::millis() - playStartTime;
        
        // Process frames up to current time
        while (frameIndex < recording.size() && recording[frameIndex].timestamp <= elapsed) {
            RecordedFrame& frame = recording[frameIndex];
            
            // Get base motor values
            int left = frame.leftStick;
            int right = frame.rightStick;
            
            // Apply IMU heading correction
            float currentHeading = imu.get_heading();
            applyHeadingCorrection(left, right, frame.heading, currentHeading);
            
            // Apply motor movements
            left_motors.move(left);
            right_motors.move(right);
            
            // Handle button presses with edge detection for toggle buttons
            uint8_t currentButtons = frame.buttons;
            
            // Intake (R1 = forward toggle, R2 = reverse toggle)
            if (wasPressed(currentButtons, prevButtons, BTN_R1)) {
                // Toggle intake forward
                static bool intakeForward = false;
                intakeForward = !intakeForward;
                if (intakeForward) {
                    Intake.move(127);
                } else {
                    Intake.move(0);
                }
            }
            if (wasPressed(currentButtons, prevButtons, BTN_R2)) {
                // Toggle intake reverse
                static bool intakeReverse = false;
                intakeReverse = !intakeReverse;
                if (intakeReverse) {
                    Intake.move(-127);
                } else {
                    Intake.move(0);
                }
            }
            
            // Outtake (L1 = forward toggle, L2 = reverse toggle)
            if (wasPressed(currentButtons, prevButtons, BTN_L1)) {
                static bool outtakeForward = false;
                outtakeForward = !outtakeForward;
                if (outtakeForward) {
                    Outtake.move(127);
                } else {
                    Outtake.move(0);
                }
            }
            if (wasPressed(currentButtons, prevButtons, BTN_L2)) {
                static bool outtakeReverse = false;
                outtakeReverse = !outtakeReverse;
                if (outtakeReverse) {
                    Outtake.move(-127);
                } else {
                    Outtake.move(0);
                }
            }
            
            // Mid-scoring toggle (X)
            if (wasPressed(currentButtons, prevButtons, BTN_X)) {
                static bool midScoring = false;
                midScoring = !midScoring;
                MidScoring.set_value(midScoring);
                if (midScoring) {
                    // Mid-scoring mode: intake reverse, outtake reverse
                    Intake.move(-127);
                    Outtake.move(-127);
                }
            }
            
            // Pneumatics (A = descore, B = unloader)
            if (wasPressed(currentButtons, prevButtons, BTN_A)) {
                static bool descore = false;
                descore = !descore;
                Descore.set_value(descore);
            }
            if (wasPressed(currentButtons, prevButtons, BTN_B)) {
                static bool unloader = false;
                unloader = !unloader;
                Unloader.set_value(unloader);
            }
            
            prevButtons = currentButtons;
            frameIndex++;
        }
        
        // Blink green indicator
        if ((elapsed / 500) % 2 == 0) {
            pros::screen::set_pen(pros::c::COLOR_GREEN);
        } else {
            pros::screen::set_pen(pros::c::COLOR_DARK_GREEN);
        }
        pros::screen::fill_circle(460, 20, 15);
        
        pros::delay(10);  // 10ms polling for smooth playback
    }
    
    // Stop all motors at end
    left_motors.move(0);
    right_motors.move(0);
    Intake.move(0);
    Outtake.move(0);
    
    _isPlaying = false;
    
    master.print(0, 0, "REPLAY COMPLETE!   ");
    
    // Clear indicator
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_circle(460, 20, 15);
}

void AutonReplay::clearRecording() {
    recording.clear();
    master.print(0, 0, "RECORDING CLEARED  ");
}

uint32_t AutonReplay::getDuration() const {
    if (recording.empty()) return 0;
    return recording.back().timestamp;
}

bool AutonReplay::saveToSD() {
    FILE* file = fopen(filePath.c_str(), "wb");
    if (!file) {
        return false;
    }
    
    // Write number of frames first
    uint32_t frameCount = recording.size();
    fwrite(&frameCount, sizeof(uint32_t), 1, file);
    
    // Write all frames
    for (const auto& frame : recording) {
        fwrite(&frame, sizeof(RecordedFrame), 1, file);
    }
    
    fclose(file);
    return true;
}

bool AutonReplay::loadFromSD() {
    FILE* file = fopen(filePath.c_str(), "rb");
    if (!file) {
        return false;
    }
    
    // Read frame count
    uint32_t frameCount = 0;
    if (fread(&frameCount, sizeof(uint32_t), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Sanity check (max ~15000 frames = 5 minutes at 50Hz)
    if (frameCount > 15000) {
        fclose(file);
        return false;
    }
    
    // Read all frames
    recording.clear();
    recording.resize(frameCount);
    
    for (uint32_t i = 0; i < frameCount; i++) {
        if (fread(&recording[i], sizeof(RecordedFrame), 1, file) != 1) {
            fclose(file);
            recording.clear();
            return false;
        }
    }
    
    fclose(file);
    master.print(0, 0, "LOADED: %d frames  ", frameCount);
    return true;
}

void AutonReplay::drawStatusIndicator() {
    // Draw status in top-right corner
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_rect(350, 0, 480, 50);
    
    if (_isRecording) {
        pros::screen::set_pen(pros::c::COLOR_RED);
        pros::screen::fill_circle(460, 20, 15);
        pros::screen::set_pen(pros::c::COLOR_WHITE);
        pros::screen::print(pros::E_TEXT_SMALL, 360, 10, "REC");
    } else if (_isPlaying) {
        pros::screen::set_pen(pros::c::COLOR_GREEN);
        pros::screen::fill_circle(460, 20, 15);
        pros::screen::set_pen(pros::c::COLOR_WHITE);
        pros::screen::print(pros::E_TEXT_SMALL, 360, 10, "PLAY");
    } else if (!recording.empty()) {
        pros::screen::set_pen(pros::c::COLOR_YELLOW);
        pros::screen::fill_circle(460, 20, 15);
        pros::screen::set_pen(pros::c::COLOR_WHITE);
        pros::screen::print(pros::E_TEXT_SMALL, 360, 10, "%d frm", recording.size());
    }
}
