package org.griffins1884.griffinsim.physics;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;

public final class AugmentedActuatorCommandMapper implements ActuatorCommandMapper {
  private final ActuatorCommandMapper delegate;
  private final Map<String, BodyMotionProfile> passiveProfiles;

  public AugmentedActuatorCommandMapper(
      ActuatorCommandMapper delegate, List<BodyMotionProfile> passiveProfiles) {
    this.delegate = Objects.requireNonNull(delegate);
    this.passiveProfiles =
        (passiveProfiles == null ? List.<BodyMotionProfile>of() : passiveProfiles).stream()
            .collect(java.util.stream.Collectors.toUnmodifiableMap(BodyMotionProfile::bodyId, profile -> profile));
  }

  @Override
  public List<BodyCommand> map(ActuatorFrame frame, ImmutableWorldState currentWorld) {
    Map<String, BodyCommand> commands = new LinkedHashMap<>();
    for (BodyCommand command : delegate.map(frame, currentWorld)) {
      commands.put(command.bodyId(), command);
    }
    for (BodyMotionProfile profile : passiveProfiles.values()) {
      commands.putIfAbsent(
          profile.bodyId(),
          new BodyCommand(profile.bodyId(), profile.massKg(), profile.momentOfInertiaKgM2(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }
    return List.copyOf(commands.values());
  }
}
