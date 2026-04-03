# Native Extension Plan

## What is now present

- source scaffold
- CMake-based build entry point
- exported symbol contract
- inclusion in release packaging

## What is still missing

- actual callback registration against WPILib native HALSIM APIs
- native-to-Java or native-to-IPC runtime wiring
- build matrix for macOS/Linux/Windows
- validation against real WPILib desktop simulation loads

## Why this intermediate step exists

The rebuilt platform now has enough deterministic architecture to justify packaging the native scaffold,
but not enough native implementation to claim a finished HALSIM extension binary.
