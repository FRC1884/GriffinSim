package org.griffins1884.griffinsim.rendering;

import org.griffins1884.griffinsim.contracts.WorldSnapshot;

public interface WorldSnapshotSubscriber {
  void onWorldSnapshot(WorldSnapshot snapshot);
}
