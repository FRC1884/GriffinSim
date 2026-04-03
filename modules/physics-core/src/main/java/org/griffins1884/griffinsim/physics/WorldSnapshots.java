package org.griffins1884.griffinsim.physics;

import org.griffins1884.griffinsim.contracts.WorldSnapshot;

public final class WorldSnapshots {
  private WorldSnapshots() {}

  public static WorldSnapshot fromWorldState(ImmutableWorldState worldState) {
    return new WorldSnapshot(worldState.header(), worldState.bodies());
  }
}
