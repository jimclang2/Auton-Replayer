#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "main.h"
#include "robot_config.h"
#include "autonomous.h"
#include "auton_replay.h"
#include "subsystems/intake.h"
#include "subsystems/outtake.h"
#include "subsystems/pneumatics.h"

void initialize() {
    initializeRobot();
    
    // Try to load any existing recording from SD card
    if (autonReplay.loadFromSD()) {
        pros::screen::set_pen(pros::c::COLOR_GREEN);
        pros::screen::print(pros::E_TEXT_MEDIUM, 10, 100, "Recording loaded from SD!");
    }
    
    drawReplayMenu();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Play back the recorded autonomous
    autonReplay.playback();
}

// Small deadband to prevent drift (applies to values close to 0)
int applyDeadband(int value, int threshold = 8) {
    return (abs(value) < threshold) ? 0 : value;
}

// Draw the main menu for recording/playback
void drawReplayMenu() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_rect(0, 0, 480, 240);
    
    // Title
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 120, 10, "AUTON RECORDER");
    
    // Draw buttons
    // Record button (left side)
    pros::screen::set_pen(pros::c::COLOR_RED);
    pros::screen::fill_rect(20, 60, 220, 140);
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 70, 90, "RECORD");
    
    // Playback button (right side)
    pros::screen::set_pen(pros::c::COLOR_GREEN);
    pros::screen::fill_rect(260, 60, 460, 140);
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 320, 90, "PLAY");
    
    // Status area
    pros::screen::set_pen(pros::c::COLOR_DARK_GRAY);
    pros::screen::fill_rect(20, 160, 460, 220);
    
    // Show recording info
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    if (autonReplay.getFrameCount() > 0) {
        uint32_t duration = autonReplay.getDuration();
        pros::screen::print(pros::E_TEXT_MEDIUM, 30, 175, 
            "Recording: %d frames (%.1f sec)", 
            autonReplay.getFrameCount(), 
            duration / 1000.0f);
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 30, 175, "No recording loaded");
    }
    
    // Instructions
    pros::screen::set_pen(pros::c::COLOR_YELLOW);
    pros::screen::print(pros::E_TEXT_SMALL, 30, 195, "Touch RECORD, drive, touch STOP when done");
}

// Handle touch input for the menu
void handleMenuTouch() {
    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    
    if (status.touch_status == pros::E_TOUCH_PRESSED) {
        int x = status.x;
        int y = status.y;
        
        // Check if in button area (y between 60 and 140)
        if (y >= 60 && y <= 140) {
            // Record button
            if (x >= 20 && x <= 220) {
                if (!autonReplay.isRecording()) {
                    autonReplay.startRecording();
                    
                    // Change button to STOP
                    pros::screen::set_pen(pros::c::COLOR_ORANGE);
                    pros::screen::fill_rect(20, 60, 220, 140);
                    pros::screen::set_pen(pros::c::COLOR_WHITE);
                    pros::screen::print(pros::E_TEXT_LARGE, 80, 90, "STOP");
                } else {
                    autonReplay.stopRecording(true);  // Save to SD
                    drawReplayMenu();  // Redraw menu
                }
            }
            // Play button
            else if (x >= 260 && x <= 460 && !autonReplay.isRecording()) {
                autonReplay.playback();
                drawReplayMenu();  // Redraw after playback
            }
        }
        
        pros::delay(200);  // Debounce
    }
}

void opcontrol() {
    IntakeControl intake;
    OuttakeControl outtake;
    PneumaticControl pneumatics;

    while (true) {
        // Handle menu touch
        handleMenuTouch();
        
        // Tank Drive with deadband
        int left = applyDeadband(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int right = applyDeadband(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        left_motors.move(left);
        right_motors.move(right);

        // Update subsystems
        outtake.update();
        intake.update(outtake.isMidScoring());
        pneumatics.update();
        
        // Record frame if recording is active
        autonReplay.recordFrame();
        
        // Controller shortcut: UP to start recording, DOWN to stop
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            if (!autonReplay.isRecording()) {
                autonReplay.startRecording();
                
                // Update screen
                pros::screen::set_pen(pros::c::COLOR_ORANGE);
                pros::screen::fill_rect(20, 60, 220, 140);
                pros::screen::set_pen(pros::c::COLOR_WHITE);
                pros::screen::print(pros::E_TEXT_LARGE, 80, 90, "STOP");
            }
        }
        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            if (autonReplay.isRecording()) {
                autonReplay.stopRecording(true);
                drawReplayMenu();
            }
        }
        
        // LEFT button to test playback
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            if (!autonReplay.isRecording() && !autonReplay.isPlaying()) {
                autonReplay.playback();
                drawReplayMenu();
            }
        }

        pros::delay(20);
    }
}