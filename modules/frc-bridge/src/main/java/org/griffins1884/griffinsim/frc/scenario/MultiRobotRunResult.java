package org.griffins1884.griffinsim.frc.scenario;

import java.util.List;
import org.griffins1884.griffinsim.contracts.ContactTelemetryFrame;
import org.griffins1884.griffinsim.contracts.WorldSnapshot;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;

public record MultiRobotRunResult(
    ImmutableWorldState finalWorldState,
    List<ContactTelemetryFrame> contactTelemetryFrames,
    List<WorldSnapshot> worldSnapshots,
    int renderedSnapshotCount,
    byte[] replayBytes) {
  public MultiRobotRunResult {
    if (finalWorldState == null) {
      throw new NullPointerException("finalWorldState");
    }
    contactTelemetryFrames = List.copyOf(contactTelemetryFrames == null ? List.of() : contactTelemetryFrames);
    worldSnapshots = List.copyOf(worldSnapshots == null ? List.of() : worldSnapshots);
    replayBytes = replayBytes == null ? new byte[0] : replayBytes.clone();
  }
}
