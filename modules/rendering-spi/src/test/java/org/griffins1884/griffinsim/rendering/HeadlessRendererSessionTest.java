package org.griffins1884.griffinsim.rendering;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.WorldSnapshot;
import org.junit.jupiter.api.Test;

class HeadlessRendererSessionTest {
  @Test
  void countsSnapshotsWithoutMutatingThem() {
    HeadlessRendererSession session = new HeadlessRendererSession();
    session.onWorldSnapshot(new WorldSnapshot(FrameHeader.current(5_000_000L, 1), List.of()));
    session.onWorldSnapshot(new WorldSnapshot(FrameHeader.current(10_000_000L, 2), List.of()));

    assertEquals(2, session.snapshotCount());
  }
}
