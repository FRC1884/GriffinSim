package org.griffins1884.griffinsim.contracts;

import java.util.List;

public record WorldSnapshot(FrameHeader header, List<RigidBodyState> bodies) {
  public WorldSnapshot {
    if (header == null) {
      throw new NullPointerException("header");
    }
    bodies = List.copyOf(bodies == null ? List.of() : bodies);
  }
}
