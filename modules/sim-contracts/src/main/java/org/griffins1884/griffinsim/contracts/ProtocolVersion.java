package org.griffins1884.griffinsim.contracts;

public enum ProtocolVersion {
  V1(1);

  private final int wireValue;

  ProtocolVersion(int wireValue) {
    this.wireValue = wireValue;
  }

  public int wireValue() {
    return wireValue;
  }

  public static ProtocolVersion fromWireValue(int wireValue) {
    for (ProtocolVersion version : values()) {
      if (version.wireValue == wireValue) {
        return version;
      }
    }
    throw new IllegalArgumentException("Unsupported protocol version: " + wireValue);
  }

  public static ProtocolVersion current() {
    return V1;
  }
}
