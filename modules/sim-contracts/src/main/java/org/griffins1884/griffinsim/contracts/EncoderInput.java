package org.griffins1884.griffinsim.contracts;

public record EncoderInput(int channel, double position, double velocity) {
  public EncoderInput {
    if (channel < 0) {
      throw new IllegalArgumentException("channel must be non-negative");
    }
  }
}
