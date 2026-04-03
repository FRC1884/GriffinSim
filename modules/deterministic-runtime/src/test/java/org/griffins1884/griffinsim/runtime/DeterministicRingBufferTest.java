package org.griffins1884.griffinsim.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class DeterministicRingBufferTest {
  @Test
  void rejectsWritesWhenFullWithoutBlocking() {
    DeterministicRingBuffer<Integer> buffer = new DeterministicRingBuffer<>(2);

    assertTrue(buffer.offer(1));
    assertTrue(buffer.offer(2));
    assertFalse(buffer.offer(3));

    List<Integer> drained = new ArrayList<>();
    buffer.drainTo(drained::add);
    assertEquals(List.of(1, 2), drained);
  }
}
