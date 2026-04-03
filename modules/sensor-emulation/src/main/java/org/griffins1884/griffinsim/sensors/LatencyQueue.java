package org.griffins1884.griffinsim.sensors;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public final class LatencyQueue<T> {
  private final List<Entry<T>> queue = new ArrayList<>();

  public void enqueue(long releaseAtNanos, T value) {
    if (releaseAtNanos < 0) {
      throw new IllegalArgumentException("releaseAtNanos must be non-negative");
    }
    Entry<T> entry = new Entry<>(releaseAtNanos, Objects.requireNonNull(value));
    int insertAt = 0;
    while (insertAt < queue.size() && queue.get(insertAt).releaseAtNanos() <= releaseAtNanos) {
      insertAt++;
    }
    queue.add(insertAt, entry);
  }

  public List<T> releaseReady(long nowNanos) {
    List<T> ready = new ArrayList<>();
    while (!queue.isEmpty() && queue.get(0).releaseAtNanos() <= nowNanos) {
      ready.add(queue.remove(0).value());
    }
    return List.copyOf(ready);
  }

  private record Entry<T>(long releaseAtNanos, T value) {}
}
