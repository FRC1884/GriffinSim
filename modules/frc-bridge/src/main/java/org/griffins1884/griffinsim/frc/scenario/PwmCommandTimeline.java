package org.griffins1884.griffinsim.frc.scenario;

import java.util.List;

public final class PwmCommandTimeline {
  private final List<ScheduledPwmCommand> commands;
  private final double defaultValue;

  public PwmCommandTimeline(List<ScheduledPwmCommand> commands, double defaultValue) {
    this.commands = List.copyOf(commands == null ? List.of() : commands);
    this.defaultValue = defaultValue;
  }

  public double valueAtTick(int tick) {
    double value = defaultValue;
    for (ScheduledPwmCommand command : commands) {
      if (tick >= command.startTick() && tick <= command.endTickInclusive()) {
        value = command.value();
      }
    }
    return value;
  }

  public static PwmCommandTimeline constant(double value) {
    return new PwmCommandTimeline(List.of(new ScheduledPwmCommand(0, Integer.MAX_VALUE, value)), 0.0);
  }
}
