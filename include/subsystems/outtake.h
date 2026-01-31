#pragma once
#include <cstdint>  // For uint32_t

class OuttakeControl {
private:
    bool toggleForward;
    bool toggleReverse;
    bool L1_lastState;
    bool L2_lastState;
    bool midScoringMode;
    bool X_lastState;
    
    uint32_t unjamStartTime;
    bool isUnjamming;

public:
    OuttakeControl();
    void update();
    int getPower();
    bool isMidScoring();
};