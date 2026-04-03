package org.griffins1884.griffinsim.frc;

import org.griffins1884.griffinsim.contracts.ImuInput;

public final class NoOpHalSimValueSink implements HalSimValueSink {
  @Override
  public void setEncoder(int channel, double position, double velocity) {}

  @Override
  public void setImu(ImuInput imuInput) {}
}
