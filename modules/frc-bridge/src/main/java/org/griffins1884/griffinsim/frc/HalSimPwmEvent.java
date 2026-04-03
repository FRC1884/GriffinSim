package org.griffins1884.griffinsim.frc;

public record HalSimPwmEvent(int channel, double value, long sequenceNumber) {
  public HalSimPwmEvent {
    if (channel < 0) {
      throw new IllegalArgumentException("channel must be non-negative");
    }
    if (sequenceNumber < 0) {
      throw new IllegalArgumentException("sequenceNumber must be non-negative");
    }
  }
}
