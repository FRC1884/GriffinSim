package org.griffins1884.griffinsim.tools;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.griffins1884.griffinsim.frc.scenario.MultiRobotRunResult;
import org.griffins1884.griffinsim.frc.scenario.MultiRobotScenario;
import org.griffins1884.griffinsim.frc.scenario.MultiRobotScenarioRunner;
import org.griffins1884.griffinsim.frc.scenario.PropertiesMultiRobotScenarioLoader;

public final class ScenarioRunnerMain {
  private ScenarioRunnerMain() {}

  public static void main(String[] args) {
    int exit = run(args);
    if (exit != 0) {
      System.exit(exit);
    }
  }

  static int run(String[] args) {
    if (args.length < 2 || args.length > 3) {
      System.err.println("usage: scenario-runner <scenario.properties> <ticks> [replay-output]");
      return 2;
    }

    Path scenarioPath = Path.of(args[0]);
    int ticks;
    try {
      ticks = Integer.parseInt(args[1]);
    } catch (NumberFormatException e) {
      System.err.println("ticks must be an integer: " + args[1]);
      return 2;
    }
    if (ticks <= 0) {
      System.err.println("ticks must be positive");
      return 2;
    }

    try {
      PropertiesMultiRobotScenarioLoader loader = new PropertiesMultiRobotScenarioLoader();
      MultiRobotScenario scenario = loader.load(scenarioPath);
      MultiRobotRunResult result = new MultiRobotScenarioRunner().run(scenario, ticks);
      if (args.length == 3) {
        Files.write(Path.of(args[2]), result.replayBytes());
      }
      System.out.println("scenario=" + scenario.name());
      System.out.println("ticks=" + ticks);
      System.out.println("bodies=" + result.finalWorldState().bodies().size());
      System.out.println("contact_frames=" + result.contactTelemetryFrames().size());
      System.out.println("world_snapshots=" + result.worldSnapshots().size());
      System.out.println("rendered_snapshots=" + result.renderedSnapshotCount());
      for (var body : result.finalWorldState().bodies()) {
        System.out.println(
            body.bodyId()
                + ": x=" + body.x()
                + ", y=" + body.y()
                + ", z=" + body.z()
                + ", wz=" + body.wz());
      }
      return 0;
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    } catch (RuntimeException e) {
      System.err.println(e.getMessage());
      return 1;
    }
  }
}
