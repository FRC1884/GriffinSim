package org.griffins1884.griffinsim.frc.scenario;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class PathFollower1dCompiler {
  private PathFollower1dCompiler() {}

  public static PwmCommandTimeline compile(PathFollower1dSpec spec) {
    List<Waypoint1d> waypoints =
        spec.waypoints().stream().sorted(Comparator.comparingInt(Waypoint1d::tick)).toList();
    int lastTick = waypoints.get(waypoints.size() - 1).tick();
    List<ScheduledPwmCommand> commands = new ArrayList<>();
    double estimatedX = spec.initialXMeters();
    Double previousPwm = null;
    int segmentStart = 0;

    for (int tick = 0; tick < lastTick; tick++) {
      double targetX = targetXAt(waypoints, tick);
      double error = targetX - estimatedX;
      double pwm = Math.abs(error) <= spec.positionToleranceMeters() ? 0.0 : clamp(spec.kP() * error, -spec.maxPwm(), spec.maxPwm());
      estimatedX += pwm * spec.metersPerTickAtFullPwm();

      if (previousPwm == null) {
        previousPwm = pwm;
        segmentStart = tick;
      } else if (Double.compare(previousPwm, pwm) != 0) {
        commands.add(new ScheduledPwmCommand(segmentStart, tick - 1, previousPwm));
        previousPwm = pwm;
        segmentStart = tick;
      }
    }

    if (previousPwm != null) {
      commands.add(new ScheduledPwmCommand(segmentStart, lastTick - 1, previousPwm));
    }

    return new PwmCommandTimeline(commands, 0.0);
  }

  private static double targetXAt(List<Waypoint1d> waypoints, int tick) {
    Waypoint1d current = waypoints.get(0);
    for (int i = 1; i < waypoints.size(); i++) {
      Waypoint1d next = waypoints.get(i);
      if (tick <= next.tick()) {
        int span = Math.max(1, next.tick() - current.tick());
        double alpha = Math.max(0.0, Math.min(1.0, (tick - current.tick()) / (double) span));
        return current.xMeters() + ((next.xMeters() - current.xMeters()) * alpha);
      }
      current = next;
    }
    return waypoints.get(waypoints.size() - 1).xMeters();
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
