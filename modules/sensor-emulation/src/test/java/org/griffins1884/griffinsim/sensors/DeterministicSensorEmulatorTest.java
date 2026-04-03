package org.griffins1884.griffinsim.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.contracts.WorldSnapshot;
import org.junit.jupiter.api.Test;

class DeterministicSensorEmulatorTest {
  @Test
  void emitsLatencyAwareDeterministicSensorFrames() {
    DeterministicSensorEmulator emulator =
        new DeterministicSensorEmulator(
            new SensorEmissionConfig("robot", 0, 10_000_000L, 2.0, 3.0, 0.0, 0.0),
            1884L);
    WorldSnapshot snapshot =
        new WorldSnapshot(
            FrameHeader.current(20_000_000L, 1),
            List.of(new RigidBodyState("robot", 1.5, 0, 0, 1, 0, 0, 0, 4.0, 0, 0, 0, 0, 0.2)));

    emulator.observe(snapshot);

    assertEquals(List.of(), emulator.releaseReady(25_000_000L));
    var ready = emulator.releaseReady(30_000_000L);
    assertEquals(1, ready.size());
    assertEquals(1, ready.get(0).header().stepId());
    assertEquals(3.0, ready.get(0).encoders().get(0).position(), 1e-9);
    assertEquals(12.0, ready.get(0).encoders().get(0).velocity(), 1e-9);
    assertEquals(0.2, ready.get(0).imu().yawRate(), 1e-9);
  }
}
