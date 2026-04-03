# Lockstep Bridge Notes

## Current implemented bridge semantics

- HAL callback work is represented as queue-only event capture via `QueuedHalSimBridge`
- callback-side API: `onPwmChanged(...)` returns immediately with bounded-queue success/failure
- bridge-side capture: `capture(FrameHeader)` drains queued events, keeps the latest value per channel, and emits a step-tagged `ActuatorFrame`
- sensor application: `HalSimSensorFrameApplicator` rejects stale frames and writes deterministic encoder/IMU values in frame order
- scheduler integration: `LockstepControlHost` applies the newest sensor frame for the control step, advances `SimHooks`, then captures actuators

## Why this matters

This preserves the WPILib threading constraint that robot-thread callbacks must not block on IPC or physics.
