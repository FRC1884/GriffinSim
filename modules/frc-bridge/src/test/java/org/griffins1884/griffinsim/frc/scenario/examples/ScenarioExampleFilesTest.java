package org.griffins1884.griffinsim.frc.scenario.examples;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import org.griffins1884.griffinsim.frc.scenario.MultiRobotRunResult;
import org.griffins1884.griffinsim.frc.scenario.MultiRobotScenario;
import org.griffins1884.griffinsim.frc.scenario.MultiRobotScenarioRunner;
import org.griffins1884.griffinsim.frc.scenario.PropertiesMultiRobotScenarioLoader;
import org.junit.jupiter.api.Test;

class ScenarioExampleFilesTest {
  private static Path repoRoot() {
    Path current = Path.of("").toAbsolutePath();
    while (current != null && !java.nio.file.Files.exists(current.resolve("settings.gradle"))) {
      current = current.getParent();
    }
    if (current == null) {
      throw new IllegalStateException("Could not locate repository root");
    }
    return current;
  }
  @Test
  void exampleScenarioFilesLoadAndRunDeterministically() {
    PropertiesMultiRobotScenarioLoader loader = new PropertiesMultiRobotScenarioLoader();
    MultiRobotScenarioRunner runner = new MultiRobotScenarioRunner();

    for (String name : new String[] {
      "two-robot-head-on.properties",
      "holonomic-pose-controller.properties",
      "velocity-pose-controller.properties"
    }) {
      Path path = repoRoot().resolve("scenarios").resolve("examples").resolve(name);
      MultiRobotScenario scenario = loader.load(path);
      MultiRobotRunResult first = runner.run(loader.load(path), 2);
      MultiRobotRunResult second = runner.run(loader.load(path), 2);
      assertEquals(first.finalWorldState(), second.finalWorldState());
      assertTrue(java.util.Arrays.equals(first.replayBytes(), second.replayBytes()));
      assertEquals(2, first.contactTelemetryFrames().size());
      assertTrue(!scenario.name().isBlank());
    }
  }
}
