package org.griffins1884.griffinsim.frc;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Collectors;
import org.griffins1884.griffinsim.contracts.ImuInput;

public final class WpilibHalSimValueSink implements HalSimValueSink {
  private final Map<Integer, EncoderSim> encoderSims;
  private final AnalogGyroSim analogGyroSim;

  public WpilibHalSimValueSink(Map<Integer, EncoderSim> encoderSims, AnalogGyroSim analogGyroSim) {
    this.encoderSims = Map.copyOf(encoderSims == null ? Map.of() : encoderSims);
    this.analogGyroSim = analogGyroSim;
  }

  public static WpilibHalSimValueSink fromDevices(List<EncoderBinding> encoders, AnalogGyro analogGyro) {
    Map<Integer, EncoderSim> encoderSimMap =
        (encoders == null ? List.<EncoderBinding>of() : encoders).stream()
            .collect(Collectors.toMap(EncoderBinding::channel, binding -> new EncoderSim(binding.encoder())));
    return new WpilibHalSimValueSink(
        encoderSimMap,
        analogGyro == null ? null : new AnalogGyroSim(analogGyro));
  }

  @Override
  public void setEncoder(int channel, double position, double velocity) {
    EncoderSim encoderSim = encoderSims.get(channel);
    if (encoderSim == null) {
      return;
    }
    encoderSim.setDistance(position);
    encoderSim.setRate(velocity);
  }

  @Override
  public void setImu(ImuInput imuInput) {
    if (analogGyroSim == null) {
      return;
    }
    analogGyroSim.setAngle(Math.toDegrees(imuInput.yaw()));
    analogGyroSim.setRate(Math.toDegrees(imuInput.yawRate()));
  }

  public record EncoderBinding(int channel, Encoder encoder) {
    public EncoderBinding {
      if (channel < 0) {
        throw new IllegalArgumentException("channel must be non-negative");
      }
      Objects.requireNonNull(encoder, "encoder");
    }
  }
}
