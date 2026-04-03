package org.griffins1884.griffinsim.runtime;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public final class ReplayDiff {
  private ReplayDiff() {}

  public static ReplayDiffResult compare(Path left, Path right) throws IOException {
    byte[] leftBytes = Files.readAllBytes(left);
    byte[] rightBytes = Files.readAllBytes(right);
    int min = Math.min(leftBytes.length, rightBytes.length);
    for (int i = 0; i < min; i++) {
      if (leftBytes[i] != rightBytes[i]) {
        return new ReplayDiffResult(
            false,
            i,
            leftBytes.length,
            rightBytes.length,
            "Replay logs differ at byte " + i);
      }
    }
    if (leftBytes.length != rightBytes.length) {
      return new ReplayDiffResult(
          false,
          min,
          leftBytes.length,
          rightBytes.length,
          "Replay logs differ in length after shared prefix");
    }
    return new ReplayDiffResult(true, -1, leftBytes.length, rightBytes.length, "Replay logs are identical");
  }
}
