package org.griffins1884.griffinsim.contracts;

public record PwmOutput(int channel, double value) {
  public PwmOutput {
    if (channel < 0) {
      throw new IllegalArgumentException("channel must be non-negative");
    }
  }
}
