# Changelog

## v0.2.0

### Added

- WPILib desktop harness for validating simulator behavior inside real robot-loop execution.
- Multi-robot co-simulation support with richer endpoint and run-result reporting.
- Additional field-contact and drivetrain validation coverage for rebuilt-season simulation.

### Changed

- Improved deterministic co-simulation loop structure and timing control.
- Refined lockstep control host and multi-robot endpoint coordination.
- Updated rebuilt field contact, traction, and terrain modeling for more stable validation runs.
- Expanded module documentation for physics, rendering, and sensor-emulation packages.

### Fixed

- Hardened HALSIM sensor frame application and callback behavior.
- Improved drivetrain support diagnostics and terrain drive authority handling.
- Added regression coverage for multi-emitter co-simulation behavior.

### Migration Notes

- Consumers should prefer the rebuilt platform modules and harness applications over legacy wrapper entry points.
- The legacy compatibility module remains available for existing integrations during migration.
