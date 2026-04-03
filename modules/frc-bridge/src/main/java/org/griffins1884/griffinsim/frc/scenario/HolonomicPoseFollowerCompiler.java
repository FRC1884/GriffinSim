package org.griffins1884.griffinsim.frc.scenario;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class HolonomicPoseFollowerCompiler {
  private HolonomicPoseFollowerCompiler() {}

  public static HolonomicPosePwmCommandPlan compile(HolonomicPoseFollowerSpec spec) {
    List<PoseWaypoint2d> waypoints =
        spec.waypoints().stream().sorted(Comparator.comparingInt(PoseWaypoint2d::tick)).toList();
    int lastTick = waypoints.get(waypoints.size() - 1).tick();
    List<ScheduledPwmCommand> xCommands = new ArrayList<>();
    List<ScheduledPwmCommand> yCommands = new ArrayList<>();
    List<ScheduledPwmCommand> thetaCommands = new ArrayList<>();
    double estimatedX = spec.initialXMeters();
    double estimatedY = spec.initialYMeters();
    double estimatedTheta = spec.initialThetaRadians();
    Double previousXPwm = null;
    Double previousYPwm = null;
    Double previousThetaPwm = null;
    int xStart = 0;
    int yStart = 0;
    int thetaStart = 0;

    for (int tick = 0; tick < lastTick; tick++) {
      PoseWaypoint2d target = targetAt(waypoints, tick);
      double xPwm = computeAxisPwm(target.xMeters() - estimatedX, spec.kPTranslation(), spec.maxPwm(), spec.positionToleranceMeters());
      double yPwm = computeAxisPwm(target.yMeters() - estimatedY, spec.kPTranslation(), spec.maxPwm(), spec.positionToleranceMeters());
      double thetaPwm = computeAxisPwm(normalizeAngle(target.thetaRadians() - estimatedTheta), spec.kPRotation(), spec.maxPwm(), spec.thetaToleranceRadians());
      estimatedX += xPwm * spec.metersPerTickAtFullPwm();
      estimatedY += yPwm * spec.metersPerTickAtFullPwm();
      estimatedTheta = normalizeAngle(estimatedTheta + (thetaPwm * spec.radiansPerTickAtFullPwm()));

      if (previousXPwm == null) { previousXPwm = xPwm; xStart = tick; }
      else if (Double.compare(previousXPwm, xPwm) != 0) { xCommands.add(new ScheduledPwmCommand(xStart, tick - 1, previousXPwm)); previousXPwm = xPwm; xStart = tick; }
      if (previousYPwm == null) { previousYPwm = yPwm; yStart = tick; }
      else if (Double.compare(previousYPwm, yPwm) != 0) { yCommands.add(new ScheduledPwmCommand(yStart, tick - 1, previousYPwm)); previousYPwm = yPwm; yStart = tick; }
      if (previousThetaPwm == null) { previousThetaPwm = thetaPwm; thetaStart = tick; }
      else if (Double.compare(previousThetaPwm, thetaPwm) != 0) { thetaCommands.add(new ScheduledPwmCommand(thetaStart, tick - 1, previousThetaPwm)); previousThetaPwm = thetaPwm; thetaStart = tick; }
    }

    if (previousXPwm != null) xCommands.add(new ScheduledPwmCommand(xStart, lastTick - 1, previousXPwm));
    if (previousYPwm != null) yCommands.add(new ScheduledPwmCommand(yStart, lastTick - 1, previousYPwm));
    if (previousThetaPwm != null) thetaCommands.add(new ScheduledPwmCommand(thetaStart, lastTick - 1, previousThetaPwm));

    return new HolonomicPosePwmCommandPlan(
        new PwmCommandTimeline(xCommands, 0.0),
        new PwmCommandTimeline(yCommands, 0.0),
        new PwmCommandTimeline(thetaCommands, 0.0));
  }

  private static PoseWaypoint2d targetAt(List<PoseWaypoint2d> waypoints, int tick) {
    PoseWaypoint2d current = waypoints.get(0);
    for (int i = 1; i < waypoints.size(); i++) {
      PoseWaypoint2d next = waypoints.get(i);
      if (tick <= next.tick()) {
        int span = Math.max(1, next.tick() - current.tick());
        double alpha = Math.max(0.0, Math.min(1.0, (tick - current.tick()) / (double) span));
        return new PoseWaypoint2d(
            tick,
            current.xMeters() + ((next.xMeters() - current.xMeters()) * alpha),
            current.yMeters() + ((next.yMeters() - current.yMeters()) * alpha),
            normalizeAngle(current.thetaRadians() + normalizeAngle(next.thetaRadians() - current.thetaRadians()) * alpha),
            next.pwmMagnitude());
      }
      current = next;
    }
    return waypoints.get(waypoints.size() - 1);
  }

  private static double computeAxisPwm(double error, double kP, double maxPwm, double tolerance) {
    return Math.abs(error) <= tolerance ? 0.0 : clamp(kP * error, -maxPwm, maxPwm);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static double normalizeAngle(double radians) {
    double value = radians;
    while (value > Math.PI) value -= 2.0 * Math.PI;
    while (value < -Math.PI) value += 2.0 * Math.PI;
    return value;
  }
}
