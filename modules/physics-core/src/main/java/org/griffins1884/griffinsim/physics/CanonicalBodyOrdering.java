package org.griffins1884.griffinsim.physics;

import java.util.Comparator;
import java.util.List;
import org.griffins1884.griffinsim.contracts.RigidBodyState;

public final class CanonicalBodyOrdering {
  private CanonicalBodyOrdering() {}

  public static List<RigidBodyState> sort(List<RigidBodyState> bodies) {
    return List.copyOf((bodies == null ? List.<RigidBodyState>of() : bodies).stream().sorted(Comparator.comparing(RigidBodyState::bodyId)).toList());
  }
}
