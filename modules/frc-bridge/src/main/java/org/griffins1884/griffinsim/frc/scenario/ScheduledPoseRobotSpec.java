package org.griffins1884.griffinsim.frc.scenario;

import java.util.Objects;
import org.griffins1884.griffinsim.physics.ActuatorCommandMapper;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;

public record ScheduledPoseRobotSpec(
    String bodyId,
    int xPwmChannel,
    int yPwmChannel,
    int thetaPwmChannel,
    HolonomicPosePwmCommandPlan commandPlan,
    long sensorSeed,
    SensorEmissionConfig sensorEmissionConfig,
    ActuatorCommandMapper actuatorCommandMapper) {
  public ScheduledPoseRobotSpec {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (xPwmChannel < 0 || yPwmChannel < 0 || thetaPwmChannel < 0) {
      throw new IllegalArgumentException("channels must be non-negative");
    }
    Objects.requireNonNull(commandPlan, "commandPlan");
    Objects.requireNonNull(sensorEmissionConfig, "sensorEmissionConfig");
    Objects.requireNonNull(actuatorCommandMapper, "actuatorCommandMapper");
  }
}
