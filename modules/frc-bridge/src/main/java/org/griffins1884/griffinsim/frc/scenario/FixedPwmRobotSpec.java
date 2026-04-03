package org.griffins1884.griffinsim.frc.scenario;

import java.util.Objects;
import org.griffins1884.griffinsim.physics.ActuatorCommandMapper;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;

public record FixedPwmRobotSpec(
    String bodyId,
    int pwmChannel,
    double pwmValue,
    long sensorSeed,
    SensorEmissionConfig sensorEmissionConfig,
    ActuatorCommandMapper actuatorCommandMapper) {
  public FixedPwmRobotSpec {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (pwmChannel < 0) {
      throw new IllegalArgumentException("pwmChannel must be non-negative");
    }
    Objects.requireNonNull(sensorEmissionConfig, "sensorEmissionConfig");
    Objects.requireNonNull(actuatorCommandMapper, "actuatorCommandMapper");
  }
}
