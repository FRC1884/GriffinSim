package org.griffins1884.griffinsim.frc.scenario;

import java.util.List;
import java.util.Objects;
import org.griffins1884.griffinsim.frc.ControlHostConfig;
import org.griffins1884.griffinsim.frc.MultiRobotEndpoint;
import org.griffins1884.griffinsim.physics.ContactGenerator;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;

public record MultiRobotScenario(
    String name,
    ControlHostConfig controlHostConfig,
    ImmutableWorldState initialWorldState,
    List<MultiRobotEndpoint> endpoints,
    ContactGenerator contactGenerator) {
  public MultiRobotScenario {
    if (name == null || name.isBlank()) {
      throw new IllegalArgumentException("name must be present");
    }
    Objects.requireNonNull(controlHostConfig, "controlHostConfig");
    Objects.requireNonNull(initialWorldState, "initialWorldState");
    endpoints = List.copyOf(endpoints == null ? List.of() : endpoints);
    Objects.requireNonNull(contactGenerator, "contactGenerator");
  }
}
