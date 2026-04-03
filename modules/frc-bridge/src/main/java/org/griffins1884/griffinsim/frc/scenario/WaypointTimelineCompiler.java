package org.griffins1884.griffinsim.frc.scenario;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class WaypointTimelineCompiler {
  private WaypointTimelineCompiler() {}

  public static PwmCommandTimeline compile1d(List<Waypoint1d> waypoints) {
    List<Waypoint1d> sorted =
        List.copyOf((waypoints == null ? List.<Waypoint1d>of() : waypoints).stream()
            .sorted(Comparator.comparingInt(Waypoint1d::tick))
            .toList());
    if (sorted.size() < 2) {
      return new PwmCommandTimeline(List.of(), 0.0);
    }

    List<ScheduledPwmCommand> commands = new ArrayList<>();
    for (int i = 0; i < sorted.size() - 1; i++) {
      Waypoint1d current = sorted.get(i);
      Waypoint1d next = sorted.get(i + 1);
      if (next.tick() <= current.tick()) {
        continue;
      }
      double deltaX = next.xMeters() - current.xMeters();
      double pwm = deltaX == 0.0 ? 0.0 : Math.copySign(next.pwmMagnitude(), deltaX);
      commands.add(new ScheduledPwmCommand(current.tick(), next.tick() - 1, pwm));
    }
    return new PwmCommandTimeline(commands, 0.0);
  }
}
