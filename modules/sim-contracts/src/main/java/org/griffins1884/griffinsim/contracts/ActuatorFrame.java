package org.griffins1884.griffinsim.contracts;

import java.util.List;

public record ActuatorFrame(FrameHeader header, List<PwmOutput> pwmOutputs) {
  public ActuatorFrame {
    if (header == null) {
      throw new NullPointerException("header");
    }
    pwmOutputs = List.copyOf(pwmOutputs == null ? List.of() : pwmOutputs);
  }
}
