package org.griffins1884.griffinsim.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SeededGaussianNoiseTest {
  @Test
  void identicalSeedsProduceIdenticalNoise() {
    SeededGaussianNoise first = new SeededGaussianNoise(1884L, 0.5);
    SeededGaussianNoise second = new SeededGaussianNoise(1884L, 0.5);

    assertEquals(first.sample(), second.sample());
    assertEquals(first.sample(), second.sample());
  }
}
