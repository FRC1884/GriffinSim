package org.griffins1884.griffinsim.frc;

import java.util.Objects;
import org.griffins1884.griffinsim.physics.ActuatorCommandMapper;
import org.griffins1884.griffinsim.sensors.DeterministicSensorEmulator;

public record MultiRobotEndpoint(
    String bodyId,
    LockstepControlHost controlHost,
    DeterministicSensorEmulator sensorEmulator,
    ActuatorCommandMapper actuatorCommandMapper) {
  public MultiRobotEndpoint {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    Objects.requireNonNull(controlHost, "controlHost");
    Objects.requireNonNull(sensorEmulator, "sensorEmulator");
    Objects.requireNonNull(actuatorCommandMapper, "actuatorCommandMapper");
  }
}
