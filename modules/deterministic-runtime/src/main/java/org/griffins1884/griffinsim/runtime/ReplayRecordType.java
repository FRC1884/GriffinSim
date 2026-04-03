package org.griffins1884.griffinsim.runtime;

public enum ReplayRecordType {
  ACTUATOR(1),
  SENSOR(2),
  WORLD(3),
  CONTACT_TELEMETRY(4);

  private final int wireValue;

  ReplayRecordType(int wireValue) {
    this.wireValue = wireValue;
  }

  public int wireValue() {
    return wireValue;
  }

  public static ReplayRecordType fromWireValue(int wireValue) {
    for (ReplayRecordType type : values()) {
      if (type.wireValue == wireValue) {
        return type;
      }
    }
    throw new IllegalArgumentException("Unsupported replay record type: " + wireValue);
  }
}
