package org.griffins1884.griffinsim.frc.scenario;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.List;
import org.griffins1884.griffinsim.frc.MultiRobotCoSimulationLoop;
import org.griffins1884.griffinsim.physics.DeterministicPhysicsWorld;
import org.griffins1884.griffinsim.runtime.ReplayLogWriter;

public final class MultiRobotScenarioRunner {
  public MultiRobotRunResult run(MultiRobotScenario scenario, int ticks) {
    try (ByteArrayOutputStream output = new ByteArrayOutputStream();
        ReplayLogWriter replayLogWriter = new ReplayLogWriter(output)) {
      MultiRobotCoSimulationLoop loop =
          new MultiRobotCoSimulationLoop(
              scenario.controlHostConfig(),
              scenario.initialWorldState(),
              new DeterministicPhysicsWorld(),
              scenario.endpoints(),
              scenario.contactGenerator(),
              replayLogWriter);
      List<org.griffins1884.griffinsim.contracts.ContactTelemetryFrame> telemetryFrames = new ArrayList<>();
      for (int i = 0; i < ticks; i++) {
        loop.advanceOneTick();
        telemetryFrames.add(loop.lastContactTelemetryFrame());
      }
      replayLogWriter.close();
      return new MultiRobotRunResult(loop.currentWorldState(), telemetryFrames, output.toByteArray());
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}
