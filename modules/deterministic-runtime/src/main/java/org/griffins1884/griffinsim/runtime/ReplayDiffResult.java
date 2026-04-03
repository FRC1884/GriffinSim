package org.griffins1884.griffinsim.runtime;

public record ReplayDiffResult(
    boolean identical, long firstDifferentByte, long leftSize, long rightSize, String message) {}
