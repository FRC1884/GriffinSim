package org.griffins1884.griffinsim.frc.scenario;

import java.util.List;

public record HolonomicVelocityPoseFollowerSpec(
    List<PoseSetpoint2d> setpoints,
    double initialXMeters,
    double initialYMeters,
    double initialThetaRadians,
    double kPTranslation,
    double kPRotation,
    double kVTranslation,
    double kVRotation,
    double maxPwm,
    double metersPerTickAtFullPwm,
    double radiansPerTickAtFullPwm,
    double positionToleranceMeters,
    double thetaToleranceRadians) {
  public HolonomicVelocityPoseFollowerSpec {
    setpoints = List.copyOf(setpoints == null ? List.of() : setpoints);
    if (setpoints.size() < 2) {
      throw new IllegalArgumentException("setpoints must contain at least 2 entries");
    }
    if (kPTranslation <= 0.0 || kPRotation <= 0.0) {
      throw new IllegalArgumentException("proportional gains must be positive");
    }
    if (kVTranslation < 0.0 || kVRotation < 0.0) {
      throw new IllegalArgumentException("feedforward gains must be non-negative");
    }
    if (maxPwm < 0.0) {
      throw new IllegalArgumentException("maxPwm must be non-negative");
    }
    if (metersPerTickAtFullPwm <= 0.0 || radiansPerTickAtFullPwm <= 0.0) {
      throw new IllegalArgumentException("response scales must be positive");
    }
    if (positionToleranceMeters < 0.0 || thetaToleranceRadians < 0.0) {
      throw new IllegalArgumentException("tolerances must be non-negative");
    }
  }
}
