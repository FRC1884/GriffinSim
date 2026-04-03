package org.griffins1884.griffinsim.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class ScenarioRunnerMainTest {
  private static Path repoRoot() {
    Path current = Path.of("").toAbsolutePath();
    while (current != null && !Files.exists(current.resolve("settings.gradle"))) {
      current = current.getParent();
    }
    if (current == null) {
      throw new IllegalStateException("Could not locate repository root");
    }
    return current;
  }

  @Test
  void runsExampleScenarioAndWritesReplay() throws Exception {
    Path replay = Files.createTempFile("griffinsim-scenario", ".bin");
    int exit =
        ScenarioRunnerMain.run(
            new String[] {
              repoRoot().resolve("scenarios/examples/two-robot-head-on.properties").toString(),
              "2",
              replay.toString()
            });

    assertEquals(0, exit);
    assertTrue(Files.size(replay) > 0L);
  }

  @Test
  void rejectsInvalidArguments() {
    assertEquals(2, ScenarioRunnerMain.run(new String[] {}));
    assertEquals(2, ScenarioRunnerMain.run(new String[] {"scenario.properties", "abc"}));
    assertEquals(2, ScenarioRunnerMain.run(new String[] {"scenario.properties", "0"}));
  }
}
