package org.griffins1884.griffinsim.frc;

import java.util.Objects;
import org.griffins1884.griffinsim.contracts.EncoderInput;
import org.griffins1884.griffinsim.contracts.SensorFrame;

public final class HalSimSensorFrameApplicator implements SensorFrameApplicator {
  private final HalSimValueSink valueSink;
  private int lastAppliedStep = -1;

  public HalSimSensorFrameApplicator(HalSimValueSink valueSink) {
    this.valueSink = Objects.requireNonNull(valueSink);
  }

  @Override
  public void apply(SensorFrame frame) {
    if (frame.header().stepId() < lastAppliedStep) {
      return;
    }
    for (EncoderInput encoder : frame.encoders()) {
      valueSink.setEncoder(encoder.channel(), encoder.position(), encoder.velocity());
    }
    valueSink.setImu(frame.imu());
    lastAppliedStep = frame.header().stepId();
  }
}
