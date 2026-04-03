package org.griffins1884.griffinsim.contracts;

public record FrameHeader(ProtocolVersion protocolVersion, long simTimeNanos, int stepId) {
  public FrameHeader {
    if (protocolVersion == null) {
      throw new NullPointerException("protocolVersion");
    }
    if (simTimeNanos < 0) {
      throw new IllegalArgumentException("simTimeNanos must be non-negative");
    }
    if (stepId < 0) {
      throw new IllegalArgumentException("stepId must be non-negative");
    }
  }

  public static FrameHeader current(long simTimeNanos, int stepId) {
    return new FrameHeader(ProtocolVersion.current(), simTimeNanos, stepId);
  }
}
