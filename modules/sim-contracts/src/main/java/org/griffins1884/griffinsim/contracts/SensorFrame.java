package org.griffins1884.griffinsim.contracts;

import java.util.List;

public record SensorFrame(FrameHeader header, List<EncoderInput> encoders, ImuInput imu) {
  public SensorFrame {
    if (header == null) {
      throw new NullPointerException("header");
    }
    encoders = List.copyOf(encoders == null ? List.of() : encoders);
    imu = imu == null ? ImuInput.zero() : imu;
  }
}
