package org.griffins1884.griffinsim.frc;

import java.util.Objects;
import org.griffins1884.griffinsim.physics.ActuatorCommandMapper;
import org.griffins1884.griffinsim.sensors.DeterministicSensorEmulator;

public record MultiRobotEndpoint(
    String bodyId,
    LockstepControlHost controlHost,
    java.util.List<DeterministicSensorEmulator> sensorEmitters,
    ActuatorCommandMapper actuatorCommandMapper) {
  public MultiRobotEndpoint {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    Objects.requireNonNull(controlHost, "controlHost");
    sensorEmitters = java.util.List.copyOf(sensorEmitters == null ? java.util.List.of() : sensorEmitters);
    if (sensorEmitters.isEmpty()) {
      throw new IllegalArgumentException("sensorEmitters must not be empty");
    }
    Objects.requireNonNull(actuatorCommandMapper, "actuatorCommandMapper");
  }

  public MultiRobotEndpoint(
      String bodyId,
      LockstepControlHost controlHost,
      DeterministicSensorEmulator sensorEmitter,
      ActuatorCommandMapper actuatorCommandMapper) {
    this(bodyId, controlHost, java.util.List.of(sensorEmitter), actuatorCommandMapper);
  }
}
