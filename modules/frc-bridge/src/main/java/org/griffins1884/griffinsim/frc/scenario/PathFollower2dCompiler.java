package org.griffins1884.griffinsim.frc.scenario;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class PathFollower2dCompiler {
  private PathFollower2dCompiler() {}

  public static HolonomicPwmCommandPlan compile(PathFollower2dSpec spec) {
    List<Waypoint2d> waypoints =
        spec.waypoints().stream().sorted(Comparator.comparingInt(Waypoint2d::tick)).toList();
    int lastTick = waypoints.get(waypoints.size() - 1).tick();
    List<ScheduledPwmCommand> xCommands = new ArrayList<>();
    List<ScheduledPwmCommand> yCommands = new ArrayList<>();
    double estimatedX = spec.initialXMeters();
    double estimatedY = spec.initialYMeters();
    Double previousXPwm = null;
    Double previousYPwm = null;
    int xSegmentStart = 0;
    int ySegmentStart = 0;

    for (int tick = 0; tick < lastTick; tick++) {
      Waypoint2d target = targetAt(waypoints, tick);
      double xError = target.xMeters() - estimatedX;
      double yError = target.yMeters() - estimatedY;
      double xPwm = Math.abs(xError) <= spec.positionToleranceMeters() ? 0.0 : clamp(spec.kP() * xError, -spec.maxPwm(), spec.maxPwm());
      double yPwm = Math.abs(yError) <= spec.positionToleranceMeters() ? 0.0 : clamp(spec.kP() * yError, -spec.maxPwm(), spec.maxPwm());
      estimatedX += xPwm * spec.metersPerTickAtFullPwm();
      estimatedY += yPwm * spec.metersPerTickAtFullPwm();

      if (previousXPwm == null) {
        previousXPwm = xPwm;
        xSegmentStart = tick;
      } else if (Double.compare(previousXPwm, xPwm) != 0) {
        xCommands.add(new ScheduledPwmCommand(xSegmentStart, tick - 1, previousXPwm));
        previousXPwm = xPwm;
        xSegmentStart = tick;
      }

      if (previousYPwm == null) {
        previousYPwm = yPwm;
        ySegmentStart = tick;
      } else if (Double.compare(previousYPwm, yPwm) != 0) {
        yCommands.add(new ScheduledPwmCommand(ySegmentStart, tick - 1, previousYPwm));
        previousYPwm = yPwm;
        ySegmentStart = tick;
      }
    }

    if (previousXPwm != null) {
      xCommands.add(new ScheduledPwmCommand(xSegmentStart, lastTick - 1, previousXPwm));
    }
    if (previousYPwm != null) {
      yCommands.add(new ScheduledPwmCommand(ySegmentStart, lastTick - 1, previousYPwm));
    }

    return new HolonomicPwmCommandPlan(new PwmCommandTimeline(xCommands, 0.0), new PwmCommandTimeline(yCommands, 0.0));
  }

  private static Waypoint2d targetAt(List<Waypoint2d> waypoints, int tick) {
    Waypoint2d current = waypoints.get(0);
    for (int i = 1; i < waypoints.size(); i++) {
      Waypoint2d next = waypoints.get(i);
      if (tick <= next.tick()) {
        int span = Math.max(1, next.tick() - current.tick());
        double alpha = Math.max(0.0, Math.min(1.0, (tick - current.tick()) / (double) span));
        return new Waypoint2d(
            tick,
            current.xMeters() + ((next.xMeters() - current.xMeters()) * alpha),
            current.yMeters() + ((next.yMeters() - current.yMeters()) * alpha),
            next.pwmMagnitude());
      }
      current = next;
    }
    return waypoints.get(waypoints.size() - 1);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
