package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MultiRobotScenarioRunnerTest {
  @Test
  void headOnScenarioRunnerIsDeterministic() {
    MultiRobotScenarioRunner runner = new MultiRobotScenarioRunner();

    MultiRobotRunResult first = runner.run(MultiRobotScenarioFixtures.twoRobotHeadOn(), 2);
    MultiRobotRunResult second = runner.run(MultiRobotScenarioFixtures.twoRobotHeadOn(), 2);

    assertEquals(first.finalWorldState(), second.finalWorldState());
    assertArrayEquals(first.replayBytes(), second.replayBytes());
  }

  @Test
  void pushScenarioMovesGamepieceAndEmitsTelemetry() {
    MultiRobotRunResult result = new MultiRobotScenarioRunner().run(MultiRobotScenarioFixtures.robotPushesGamepiece(), 1);

    var gamepiece = result.finalWorldState().bodies().stream().filter(body -> body.bodyId().equals("gamepiece")).findFirst().orElseThrow();
    assertTrue(gamepiece.x() > 0.3 || gamepiece.y() != 0.0 || gamepiece.z() != 0.0);
    assertTrue(result.contactTelemetryFrames().stream().flatMap(frame -> frame.contacts().stream()).anyMatch(contact -> contact.contactId().contains("vs:gamepiece") || contact.contactId().contains("vs:robot")));
  }
}
