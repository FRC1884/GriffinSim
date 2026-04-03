package org.griffins1884.griffinsim.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;

class LatencyQueueTest {
  @Test
  void releasesOnlyReadyItemsInReleaseOrder() {
    LatencyQueue<String> queue = new LatencyQueue<>();
    queue.enqueue(30, "late");
    queue.enqueue(10, "first");
    queue.enqueue(20, "second");

    assertEquals(List.of(), queue.releaseReady(5));
    assertEquals(List.of("first", "second", "late"), queue.releaseReady(30));
  }
}
