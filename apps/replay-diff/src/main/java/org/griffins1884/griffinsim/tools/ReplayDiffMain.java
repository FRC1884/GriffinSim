package org.griffins1884.griffinsim.tools;

import java.nio.file.Path;
import org.griffins1884.griffinsim.runtime.ReplayDiff;
import org.griffins1884.griffinsim.runtime.ReplayDiffResult;

public final class ReplayDiffMain {
  private ReplayDiffMain() {}

  public static void main(String[] args) throws Exception {
    if (args.length != 2) {
      throw new IllegalArgumentException("usage: replay-diff <left-log> <right-log>");
    }
    ReplayDiffResult result = ReplayDiff.compare(Path.of(args[0]), Path.of(args[1]));
    System.out.println(result.message());
    if (!result.identical()) {
      System.exit(1);
    }
  }
}
