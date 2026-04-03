package org.griffins1884.griffinsim.frc.scenario;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class HolonomicVelocityPoseFollowerCompiler {
  private HolonomicVelocityPoseFollowerCompiler() {}

  public static HolonomicPosePwmCommandPlan compile(HolonomicVelocityPoseFollowerSpec spec) {
    List<PoseSetpoint2d> setpoints =
        spec.setpoints().stream().sorted(Comparator.comparingInt(PoseSetpoint2d::tick)).toList();
    int lastTick = setpoints.get(setpoints.size() - 1).tick();
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
      PoseSetpoint2d target = targetAt(setpoints, tick);
      double xError = target.xMeters() - estimatedX;
      double yError = target.yMeters() - estimatedY;
      double thetaError = normalizeAngle(target.thetaRadians() - estimatedTheta);

      double xPwm = computePwm(xError, target.vxMetersPerTick(), spec.kPTranslation(), spec.kVTranslation(), spec.maxPwm(), spec.positionToleranceMeters(), spec.metersPerTickAtFullPwm(), target.pwmMagnitude());
      double yPwm = computePwm(yError, target.vyMetersPerTick(), spec.kPTranslation(), spec.kVTranslation(), spec.maxPwm(), spec.positionToleranceMeters(), spec.metersPerTickAtFullPwm(), target.pwmMagnitude());
      double thetaPwm = computePwm(thetaError, target.omegaRadiansPerTick(), spec.kPRotation(), spec.kVRotation(), spec.maxPwm(), spec.thetaToleranceRadians(), spec.radiansPerTickAtFullPwm(), target.pwmMagnitude());

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

  private static double computePwm(
      double error,
      double desiredPerTick,
      double kP,
      double kV,
      double maxPwm,
      double tolerance,
      double unitsPerTickAtFullPwm,
      double pwmMagnitude) {
    if (Math.abs(error) <= tolerance && Math.abs(desiredPerTick) <= 1e-9) {
      return 0.0;
    }
    double feedforward = unitsPerTickAtFullPwm <= 1e-9 ? 0.0 : (desiredPerTick / unitsPerTickAtFullPwm) * kV;
    double feedback = error * kP;
    return clamp(feedforward + feedback, -Math.min(maxPwm, pwmMagnitude), Math.min(maxPwm, pwmMagnitude));
  }

  private static PoseSetpoint2d targetAt(List<PoseSetpoint2d> setpoints, int tick) {
    PoseSetpoint2d current = setpoints.get(0);
    for (int i = 1; i < setpoints.size(); i++) {
      PoseSetpoint2d next = setpoints.get(i);
      if (tick <= next.tick()) {
        int span = Math.max(1, next.tick() - current.tick());
        double alpha = Math.max(0.0, Math.min(1.0, (tick - current.tick()) / (double) span));
        return new PoseSetpoint2d(
            tick,
            current.xMeters() + ((next.xMeters() - current.xMeters()) * alpha),
            current.yMeters() + ((next.yMeters() - current.yMeters()) * alpha),
            normalizeAngle(current.thetaRadians() + normalizeAngle(next.thetaRadians() - current.thetaRadians()) * alpha),
            current.vxMetersPerTick() + ((next.vxMetersPerTick() - current.vxMetersPerTick()) * alpha),
            current.vyMetersPerTick() + ((next.vyMetersPerTick() - current.vyMetersPerTick()) * alpha),
            current.omegaRadiansPerTick() + ((next.omegaRadiansPerTick() - current.omegaRadiansPerTick()) * alpha),
            next.pwmMagnitude());
      }
      current = next;
    }
    return setpoints.get(setpoints.size() - 1);
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
