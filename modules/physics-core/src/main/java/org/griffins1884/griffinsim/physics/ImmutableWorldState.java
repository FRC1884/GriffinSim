package org.griffins1884.griffinsim.physics;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;

public record ImmutableWorldState(FrameHeader header, List<RigidBodyState> bodies) {
  public ImmutableWorldState {
    if (header == null) {
      throw new NullPointerException("header");
    }
    bodies = CanonicalBodyOrdering.sort(bodies);
  }
}
