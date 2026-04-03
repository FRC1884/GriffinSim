package org.griffins1884.griffinsim.sensors;

import java.util.Random;

public final class SeededGaussianNoise {
  private final Random random;
  private final double standardDeviation;

  public SeededGaussianNoise(long seed, double standardDeviation) {
    this.random = new Random(seed);
    this.standardDeviation = standardDeviation;
  }

  public double sample() {
    return random.nextGaussian() * standardDeviation;
  }
}
