package org.griffins1884.griffinsim.rendering;

import java.util.concurrent.atomic.AtomicInteger;
import org.griffins1884.griffinsim.contracts.WorldSnapshot;

public final class HeadlessRendererSession implements WorldSnapshotSubscriber {
  private final AtomicInteger snapshotCount = new AtomicInteger();

  @Override
  public void onWorldSnapshot(WorldSnapshot snapshot) {
    snapshotCount.incrementAndGet();
  }

  public int snapshotCount() {
    return snapshotCount.get();
  }
}
