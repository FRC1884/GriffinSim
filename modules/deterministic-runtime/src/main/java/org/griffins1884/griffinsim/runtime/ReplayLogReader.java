package org.griffins1884.griffinsim.runtime;

import java.io.DataInputStream;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.List;

public final class ReplayLogReader {
  private static final int MAGIC = 0x4753494D;

  private ReplayLogReader() {}

  public static List<ReplayRecord> readAll(InputStream inputStream) {
    try {
      DataInputStream in = new DataInputStream(inputStream);
      List<ReplayRecord> records = new ArrayList<>();
      while (true) {
        try {
          int magic = in.readInt();
          if (magic != MAGIC) {
            throw new IllegalArgumentException("Unexpected replay magic: " + magic);
          }
          ReplayRecordType type = ReplayRecordType.fromWireValue(in.readInt());
          int length = in.readInt();
          byte[] payload = in.readNBytes(length);
          if (payload.length != length) {
            throw new IOException("Unexpected replay EOF");
          }
          records.add(new ReplayRecord(type, payload));
        } catch (EOFException eof) {
          return List.copyOf(records);
        }
      }
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}
