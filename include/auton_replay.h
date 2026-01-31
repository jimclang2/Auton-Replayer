#pragma once
#include "main.h"
#include <vector>
#include <string>

// Single frame of recorded data - captures all driver inputs at a moment in time
struct RecordedFrame {
    uint32_t timestamp;     // Time since recording started (ms)
    int8_t leftStick;       // Left joystick Y value (-127 to 127)
    int8_t rightStick;      // Right joystick Y value (-127 to 127)
    float heading;          // IMU heading at this frame (for drift correction)
    
    // Button states packed into bitflags for memory efficiency
    // Bit 0: R1 (intake forward toggle)
    // Bit 1: R2 (intake reverse toggle)
    // Bit 2: L1 (outtake forward toggle)
    // Bit 3: L2 (outtake reverse toggle)
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
    uint32_t recordStartTime = 0;
    bool _isRecording = false;
    bool _isPlaying = false;
    
    // Previous button states for edge detection during playback
    uint8_t prevButtons = 0;
    
    // IMU correction settings
    float imuCorrectionGain = 2.0f;  // How aggressively to correct heading drift
    
    // File path for SD card storage
    std::string filePath = "/usd/auton_recording.bin";
    
    // Helper to apply IMU heading correction
    void applyHeadingCorrection(int& left, int& right, float targetHeading, float currentHeading);
    
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
    
    // Draw status indicator on brain screen
    void drawStatusIndicator();
};

// Global instance
extern AutonReplay autonReplay;
