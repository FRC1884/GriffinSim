package org.griffins1884.griffinsim.tools;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import java.util.ArrayList;
import java.util.List;
import org.griffins1884.griffinsim.frc.HalSimValueSink;
import org.griffins1884.griffinsim.frc.WpilibHalSimValueSink;

public final class FixtureTimedRobot extends TimedRobot implements WpilibSimulationHarness.BoundRobot {
  private final PWMSparkMax xMotor = new PWMSparkMax(0);
  private final PWMSparkMax yMotor = new PWMSparkMax(1);
  private final PWMSparkMax thetaMotor = new PWMSparkMax(2);
  private final Encoder driveEncoder = new Encoder(0, 1);
  private final AnalogGyro headingGyro = new AnalogGyro(0);
  private final List<String> lifecycleEvents = new ArrayList<>();
  private final List<Double> encoderSamples = new ArrayList<>();
  private final List<Double> gyroSamplesDegrees = new ArrayList<>();
  private final List<CommandSample> commandSamples = new ArrayList<>();
  private int teleopPeriodicCalls;
  private int disabledPeriodicCalls;

  @Override
  public void robotInit() {
    lifecycleEvents.add("robotInit");
    driveEncoder.reset();
    headingGyro.reset();
  }

  @Override
  public void driverStationConnected() {
    lifecycleEvents.add("driverStationConnected");
  }

  @Override
  public void simulationInit() {
    lifecycleEvents.add("simulationInit");
  }

  @Override
  public void disabledInit() {
    lifecycleEvents.add("disabledInit");
    stopAll();
  }

  @Override
  public void disabledPeriodic() {
    disabledPeriodicCalls++;
    stopAll();
  }

  @Override
  public void teleopInit() {
    lifecycleEvents.add("teleopInit");
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public void teleopPeriodic() {
    double distance = driveEncoder.getDistance();
    CommandSample command = scriptedCommand(teleopPeriodicCalls);
    xMotor.set(command.x());
    yMotor.set(command.y());
    thetaMotor.set(command.theta());
    encoderSamples.add(distance);
    gyroSamplesDegrees.add(headingGyro.getAngle());
    commandSamples.add(command);
    teleopPeriodicCalls++;
  }

  @Override
  public HalSimValueSink halSimValueSink() {
    return WpilibHalSimValueSink.fromDevices(
        List.of(new WpilibHalSimValueSink.EncoderBinding(0, driveEncoder)), headingGyro);
  }

  public List<String> lifecycleEvents() {
    return List.copyOf(lifecycleEvents);
  }

  public List<Double> encoderSamples() {
    return List.copyOf(encoderSamples);
  }

  public List<Double> gyroSamplesDegrees() {
    return List.copyOf(gyroSamplesDegrees);
  }

  public List<Double> motorCommandSamples() {
    return commandSamples.stream().map(CommandSample::x).toList();
  }

  public List<CommandSample> commandSamples() {
    return List.copyOf(commandSamples);
  }

  public int teleopPeriodicCalls() {
    return teleopPeriodicCalls;
  }

  public int disabledPeriodicCalls() {
    return disabledPeriodicCalls;
  }

  public double lastEncoderDistance() {
    return encoderSamples.isEmpty() ? driveEncoder.getDistance() : encoderSamples.get(encoderSamples.size() - 1);
  }

  public double lastGyroAngleDegrees() {
    return gyroSamplesDegrees.isEmpty() ? headingGyro.getAngle() : gyroSamplesDegrees.get(gyroSamplesDegrees.size() - 1);
  }

  public double firstMotorCommand() {
    return commandSamples.isEmpty() ? 0.0 : commandSamples.get(0).x();
  }

  public double lastMotorCommand() {
    return commandSamples.isEmpty() ? 0.0 : commandSamples.get(commandSamples.size() - 1).x();
  }

  @Override
  public void close() {
    xMotor.close();
    yMotor.close();
    thetaMotor.close();
    driveEncoder.close();
    headingGyro.close();
    super.close();
  }

  private void stopAll() {
    xMotor.stopMotor();
    yMotor.stopMotor();
    thetaMotor.stopMotor();
  }

  private CommandSample scriptedCommand(int teleopTick) {
    if (teleopTick < 50) {
      return new CommandSample(0.8, 0.0, 0.0);
    }
    if (teleopTick < 100) {
      return new CommandSample(0.0, 0.0, 0.8);
    }
    if (teleopTick < 150) {
      return new CommandSample(0.6, 0.3, 0.0);
    }
    return new CommandSample(0.0, 0.0, 0.0);
  }

  public record CommandSample(double x, double y, double theta) {}
}
