package org.griffins1884.griffinsim.contracts;

import java.util.List;

public record ContactTelemetryFrame(FrameHeader header, List<ContactTelemetry> contacts) {
  public ContactTelemetryFrame {
    if (header == null) {
      throw new NullPointerException("header");
    }
    contacts = List.copyOf(contacts == null ? List.of() : contacts);
  }
}
