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

bool AutonReplay::isSDCardInserted() const {
    // Check if SD card is present by attempting to stat the directory
    FILE* test = fopen("/usd/.", "r");
    if (test) {
        fclose(test);
        return true;
    }
    return false;
}

bool AutonReplay::checkEmergencyStop() {
    // Check for Left + Right arrow combo as emergency stop
    // Requires BOTH buttons pressed simultaneously to prevent accidental stops
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && 
           master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
}

void AutonReplay::displayCountdown(int secondsRemaining) {
    // Display on brain screen
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_rect(180, 80, 300, 160);
    pros::screen::set_pen(pros::c::COLOR_YELLOW);
    pros::screen::print(pros::E_TEXT_LARGE, 200, 100, "Starting in");
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 230, 130, "%d", secondsRemaining);
    
    // Display on controller
    master.print(0, 0, "Starting in %d...  ", secondsRemaining);
    master.rumble(".");
}

void AutonReplay::abortPlayback() {
    _abortRequested = true;
}

void AutonReplay::startRecording() {
    // Check SD card before starting if we plan to save
    if (!isSDCardInserted()) {
        master.print(0, 0, "WARNING: No SD Card!");
        master.rumble("---");
        pros::delay(1000);
        master.print(0, 0, "Recording anyway...");
    }
    
    // Countdown before recording starts
    if (countdownDuration > 0) {
        int seconds = countdownDuration / 1000;
        for (int i = seconds; i > 0; i--) {
            displayCountdown(i);
            pros::delay(1000);
        }
    }
    
    recording.clear();
    
    // Reserve memory to avoid reallocation during recording (5 minutes at 50Hz)
    try {
        recording.reserve(15000);
    } catch (...) {
        master.print(0, 0, "MEM RESERVE FAILED!");
        master.rumble("---");
    }
    
    // Use microseconds for precision timing
    recordStartTime = pros::micros();
    _isRecording = true;
    
    // Reset IMU heading to 0 at start of recording for consistent reference
    imu.set_heading(0);
    
    // Boost task priority during recording for consistent timing
    pros::Task::current().set_priority(TASK_PRIORITY_MAX - 1);
    
    master.print(0, 0, "RECORDING...       ");
    master.rumble("-");  // Short vibration to confirm
    
    drawStatusIndicator();
}

void AutonReplay::stopRecording(bool saveToSD) {
    _isRecording = false;
    
    // Restore normal task priority
    pros::Task::current().set_priority(TASK_PRIORITY_DEFAULT);
    
    master.print(0, 0, "STOPPED: %d frames ", recording.size());
    master.rumble(".");  // Confirm vibration
    
    if (saveToSD) {
        // Check SD card is present before attempting save
        if (!isSDCardInserted()) {
            master.print(1, 0, "NO SD CARD!        ");
            master.rumble("---");
        } else if (this->saveToSD()) {
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
    // Use microseconds for precise timing
    frame.timestamp = pros::micros() - recordStartTime;
    frame.leftStick = static_cast<int8_t>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    frame.rightStick = static_cast<int8_t>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    
    // Record actual motor voltage (get_voltage returns millivolts: -12000 to 12000)
    // Scale to -127 to 127 range to match what move() expects
    frame.intakePower = static_cast<int8_t>(Intake.get_voltage() * 127 / 12000);
    frame.outtakePower = static_cast<int8_t>(Outtake.get_voltage() * 127 / 12000);
    
    frame.heading = imu.get_heading();  // Record heading for drift correction
    frame.buttons = packButtons();
    
    // Safe push_back with error handling
    try {
        recording.push_back(frame);
    } catch (...) {
        // Memory allocation failed - stop recording
        master.print(0, 0, "MEMORY FULL!       ");
        stopRecording(true);
        return;
    }
    
    // Convert to milliseconds for display
    uint32_t elapsedMs = frame.timestamp / 1000;
    
    // Blink indicator every 500ms
    if ((elapsedMs / 500) % 2 == 0) {
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
    _abortRequested = false;  // Reset abort flag
    prevButtons = 0;
    
    // Reset toggle states for pneumatics (motor velocities are recorded directly now)
    bool midScoring = false;
    bool descore = false;
    bool unloader = false;
    
    // Reset IMU heading to match the recording start
    imu.set_heading(0);
    pros::delay(50);  // Brief delay to let IMU settle
    
    // Boost task priority during playback for consistent timing
    pros::Task::current().set_priority(TASK_PRIORITY_MAX - 1);
    
    // Use microseconds for precision timing
    uint64_t playStartTime = pros::micros();
    size_t frameIndex = 0;
    
    master.print(0, 0, "REPLAYING (<>=STOP)");
    
    // Draw green indicator
    pros::screen::set_pen(pros::c::COLOR_GREEN);
    pros::screen::fill_circle(460, 20, 15);
    
    while (frameIndex < recording.size()) {
        // Check for emergency stop (Y + A buttons)
        if (checkEmergencyStop() || _abortRequested) {
            master.print(0, 0, "PLAYBACK ABORTED!  ");
            master.rumble("--");
            break;
        }
        
        uint64_t elapsed = pros::micros() - playStartTime;
        
        // Process frames up to current time (using microseconds)
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
            
            // Apply recorded motor power directly
            Intake.move(frame.intakePower);
            Outtake.move(frame.outtakePower);
            
            // Handle button presses with edge detection for toggle buttons
            uint8_t currentButtons = frame.buttons;
            
            // Mid-scoring toggle (X) - still need to handle pneumatic
            if (wasPressed(currentButtons, prevButtons, BTN_X)) {
                midScoring = !midScoring;
                MidScoring.set_value(midScoring);
            }
            
            // Pneumatics (A = descore, B = unloader)
            if (wasPressed(currentButtons, prevButtons, BTN_A)) {
                descore = !descore;
                Descore.set_value(descore);
            }
            if (wasPressed(currentButtons, prevButtons, BTN_B)) {
                unloader = !unloader;
                Unloader.set_value(unloader);
            }
            
            prevButtons = currentButtons;
            frameIndex++;
        }
        
        // Convert to milliseconds for display
        uint32_t elapsedMs = elapsed / 1000;
        
        // Blink green indicator
        if ((elapsedMs / 500) % 2 == 0) {
            pros::screen::set_pen(pros::c::COLOR_GREEN);
        } else {
            pros::screen::set_pen(pros::c::COLOR_DARK_GREEN);
        }
        pros::screen::fill_circle(460, 20, 15);
        
        pros::delay(5);  // 5ms polling for smoother playback with microsecond timing
    }
    
    // Stop all motors at end
    left_motors.move(0);
    right_motors.move(0);
    Intake.move(0);
    Outtake.move(0);
    
    // Restore normal task priority
    pros::Task::current().set_priority(TASK_PRIORITY_DEFAULT);
    
    _isPlaying = false;
    _abortRequested = false;
    
    if (!_abortRequested) {
        master.print(0, 0, "REPLAY COMPLETE!   ");
    }
    
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
    // Convert from microseconds to milliseconds
    return recording.back().timestamp / 1000;
}

bool AutonReplay::saveToSD() {
    // Check SD card first
    if (!isSDCardInserted()) {
        return false;
    }
    
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
    // Check SD card first
    if (!isSDCardInserted()) {
        master.print(0, 0, "NO SD CARD!        ");
        return false;
    }
    
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
