package org.griffins1884.griffinsim.runtime;

import java.util.Arrays;

public final class ReplayRecord {
  private final ReplayRecordType type;
  private final byte[] payload;

  public ReplayRecord(ReplayRecordType type, byte[] payload) {
    if (type == null) {
      throw new NullPointerException("type");
    }
    if (payload == null) {
      throw new NullPointerException("payload");
    }
    this.type = type;
    this.payload = payload.clone();
  }

  public ReplayRecordType type() {
    return type;
  }

  public byte[] payload() {
    return payload.clone();
  }

  @Override
  public boolean equals(Object other) {
    if (this == other) {
      return true;
    }
    if (!(other instanceof ReplayRecord that)) {
      return false;
    }
    return type == that.type && Arrays.equals(payload, that.payload);
  }

  @Override
  public int hashCode() {
    return 31 * type.hashCode() + Arrays.hashCode(payload);
  }
}
