#pragma once
#include "main.h"
#include <vector>
#include <string>

// Single frame of recorded data - captures all driver inputs at a moment in time
struct RecordedFrame {
    uint64_t timestamp;     // Time since recording started (microseconds for precision)
    int8_t leftStick;       // Left joystick Y value (-127 to 127)
    int8_t rightStick;      // Right joystick Y value (-127 to 127)
    int8_t intakePower;     // Actual intake motor power (-127 to 127, from voltage)
    int8_t outtakePower;    // Actual outtake motor power (-127 to 127, from voltage)
    float heading;          // IMU heading at this frame (for drift correction)
    
    // Button states packed into bitflags for memory efficiency
    // Bit 0: R1 (intake forward toggle) - kept for reference
    // Bit 1: R2 (intake reverse toggle) - kept for reference
    // Bit 2: L1 (outtake forward toggle) - kept for reference
    // Bit 3: L2 (outtake reverse toggle) - kept for reference
    // Bit 4: X (mid-scoring toggle)
    // Bit 5: A (descore toggle)
    // Bit 6: B (unloader toggle)
    uint8_t buttons;
};

// Button bit positions
constexpr uint8_t BTN_R1 = 0;
constexpr uint8_t BTN_R2 = 1;
constexpr uint8_t BTN_L1 = 2;
constexpr uint8_t BTN_L2 = 3;
constexpr uint8_t BTN_X  = 4;
constexpr uint8_t BTN_A  = 5;
constexpr uint8_t BTN_B  = 6;

// Recording/Playback System with IMU correction and SD card persistence
class AutonReplay {
private:
    std::vector<RecordedFrame> recording;
    uint64_t recordStartTime = 0;  // Microseconds for precision timing
    bool _isRecording = false;
    bool _isPlaying = false;
    bool _abortRequested = false;  // For emergency stop during playback
    
    // Previous button states for edge detection during playback
    uint8_t prevButtons = 0;
    
    // IMU correction settings
    float imuCorrectionGain = 2.0f;  // How aggressively to correct heading drift
    
    // Countdown before recording starts (milliseconds)
    uint32_t countdownDuration = 3000;  // 3 second countdown by default
    
    // File path for SD card storage
    std::string filePath = "/usd/auton_recording.bin";
    
    // Helper to apply IMU heading correction
    void applyHeadingCorrection(int& left, int& right, float targetHeading, float currentHeading);
    
    // Helper to display countdown on screen and controller
    void displayCountdown(int secondsRemaining);
    
    // Helper to check for emergency stop button combo (Left + Right arrows)
    bool checkEmergencyStop();
    
public:
    // Start recording driver inputs
    void startRecording();
    
    // Stop recording and optionally save to SD card
    void stopRecording(bool saveToSD = true);
    
    // Record a single frame (call this in opcontrol loop at 20ms intervals)
    void recordFrame();
    
    // Playback the recording in autonomous (with IMU drift correction)
    void playback();
    
    // Clear the current recording
    void clearRecording();
    
    // Save recording to SD card
    bool saveToSD();
    
    // Load recording from SD card
    bool loadFromSD();
    
    // Get recording size (number of frames)
    int getFrameCount() const { return recording.size(); }
    
    // Get recording duration in milliseconds
    uint32_t getDuration() const;
    
    // Is currently recording?
    bool isRecording() const { return _isRecording; }
    
    // Is currently playing?
    bool isPlaying() const { return _isPlaying; }
    
    // Set IMU correction gain (higher = more aggressive correction)
    void setIMUCorrectionGain(float gain) { imuCorrectionGain = gain; }
    
    // Set countdown duration before recording starts (in milliseconds)
    void setCountdownDuration(uint32_t ms) { countdownDuration = ms; }
    
    // Abort playback (call from emergency stop)
    void abortPlayback();
    
    // Check if SD card is present
    bool isSDCardInserted() const;
    
    // Draw status indicator on brain screen
    void drawStatusIndicator();
};

// Global instance
extern AutonReplay autonReplay;
