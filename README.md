# VEX V5 Auton Replayer (PROS)

Record your driver movements during practice and replay them during autonomous! Simply drive your route, and the robot will repeat it exactly.

## Features

- **Record & Replay** - Drive your autonomous route manually, replay it perfectly
- **Motor Velocity Recording** - Captures exact motor speeds, not just on/off states
- **IMU Drift Correction** - Automatically corrects heading drift during playback
- **SD Card Persistence** - Recordings survive power cycles
- **Touch Screen UI** - Easy record/play buttons on Brain screen
- **Microsecond Precision** - Accurate timing for consistent replays
- **Emergency Stop** - Press Left + Right arrows to abort playback

---

## Quick Start Guide

### 1. Setup
```
1. Upload this program to the Brain
2. Insert SD card into Brain (leave it there forever)
3. Run the program (normal "Run" on brain)
```

> **Note:** You can now DRIVE normally! The controller works just like usual.

### 2. Record Your Autonomous

| Method | Action |
|--------|--------|
| **Brain Screen** | Touch `RECORD` → Drive → Touch `STOP` |
| **Controller** | Press `UP` → Drive → Press `DOWN` |

A 3-second countdown will appear before recording starts. Recording is automatically saved to SD card!

### 3. Test Your Recording

| Method | Action |
|--------|--------|
| **Brain Screen** | Touch `PLAY` |
| **Controller** | Press `LEFT` |

The robot will replay exactly what you drove!

### 4. At Competition

When autonomous period starts, it automatically plays your recorded route.
> The `autonomous()` function calls `autonReplay.playback()`

---

## Controller Button Map

| Button | Function |
|--------|----------|
| `UP` | Start recording |
| `DOWN` | Stop recording (saves to SD) |
| `LEFT` | Test playback |
| `LEFT + RIGHT` | Emergency stop during playback |

---

## Tips for Best Results

1. **Start in the same position** - Place robot identically each time
2. **Wait for IMU calibration** - Let robot sit still for 2-3 seconds on startup
3. **Drive smoothly** - Jerky movements may not replay as well
4. **Keep recordings short** - Longer recordings accumulate more drift
5. **Test before competition** - Always verify playback works as expected

---

## Customization

```cpp
// In your code, you can customize:
autonReplay.setCountdownDuration(5000);  // 5 second countdown (default: 3000)
autonReplay.setCountdownDuration(0);     // No countdown
autonReplay.setIMUCorrectionGain(3.0f);  // More aggressive drift correction (default: 2.0)
```

---

## Technical Details

- **Recording Format:** Binary file at `/usd/auton_recording.bin`
- **Sample Rate:** 50Hz (every 20ms)
- **Max Duration:** ~5 minutes (15000 frames)
- **Data Captured:** Joystick values, motor velocities, button states, IMU heading, timestamps (microseconds)

---

## License

Apache-2.0 - Based on concepts from [Skyluker4/VEX-V5-Replay](https://github.com/Skyluker4/VEX-V5-Replay)
