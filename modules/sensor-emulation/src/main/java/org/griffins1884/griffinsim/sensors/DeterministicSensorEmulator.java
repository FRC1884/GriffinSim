package org.griffins1884.griffinsim.sensors;

import java.util.List;
import java.util.Objects;
import org.griffins1884.griffinsim.contracts.EncoderInput;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.ImuInput;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.griffins1884.griffinsim.contracts.WorldSnapshot;

public final class DeterministicSensorEmulator {
  private final SensorEmissionConfig config;
  private final LatencyQueue<SensorFrame> latencyQueue = new LatencyQueue<>();
  private final SeededGaussianNoise encoderNoise;
  private final SeededGaussianNoise yawNoise;

  public DeterministicSensorEmulator(SensorEmissionConfig config, long seed) {
    this.config = Objects.requireNonNull(config);
    this.encoderNoise = new SeededGaussianNoise(seed, config.encoderNoiseStdDev());
    this.yawNoise = new SeededGaussianNoise(seed ^ 0x5DEECE66DL, config.yawNoiseStdDev());
  }

  public void observe(WorldSnapshot snapshot) {
    RigidBodyState body =
        snapshot.bodies().stream()
            .filter(candidate -> candidate.bodyId().equals(config.bodyId()))
            .findFirst()
            .orElseThrow(() -> new IllegalArgumentException("Missing body for sensor emission: " + config.bodyId()));
    SensorFrame frame =
        new SensorFrame(
            snapshot.header(),
            List.of(
                new EncoderInput(
                    config.encoderChannel(),
                    (body.x() * config.positionScale()) + encoderNoise.sample(),
                    (body.vx() * config.velocityScale()) + encoderNoise.sample())),
            new ImuInput(
                quaternionYaw(body) + yawNoise.sample(),
                0.0,
                0.0,
                body.wz(),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0));
    latencyQueue.enqueue(snapshot.header().simTimeNanos() + config.latencyNanos(), frame);
  }

  public List<SensorFrame> releaseReady(long nowNanos) {
    return latencyQueue.releaseReady(nowNanos);
  }

  private static double quaternionYaw(RigidBodyState body) {
    double sinyCosp = 2.0 * ((body.qw() * body.qz()) + (body.qx() * body.qy()));
    double cosyCosp = 1.0 - (2.0 * ((body.qy() * body.qy()) + (body.qz() * body.qz())));
    return Math.atan2(sinyCosp, cosyCosp);
  }
}
