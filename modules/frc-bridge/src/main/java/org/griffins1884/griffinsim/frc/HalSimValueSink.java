package org.griffins1884.griffinsim.frc;

import org.griffins1884.griffinsim.contracts.ImuInput;

public interface HalSimValueSink {
  void setEncoder(int channel, double position, double velocity);

  void setImu(ImuInput imuInput);
}
