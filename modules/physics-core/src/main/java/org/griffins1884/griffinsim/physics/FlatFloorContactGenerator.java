package org.griffins1884.griffinsim.physics;

import java.util.List;
import java.util.Map;
import org.griffins1884.griffinsim.contracts.RigidBodyState;

public final class FlatFloorContactGenerator implements ContactGenerator {
  private final double floorZ;
  private final MaterialProfile floorMaterial;
  private final Map<String, MaterialProfile> bodyMaterials;

  public FlatFloorContactGenerator(double floorZ) {
    this(floorZ, MaterialProfiles.CARPET, Map.of());
  }

  public FlatFloorContactGenerator(double floorZ, MaterialProfile floorMaterial, Map<String, MaterialProfile> bodyMaterials) {
    this.floorZ = floorZ;
    this.floorMaterial = floorMaterial == null ? MaterialProfiles.CARPET : floorMaterial;
    this.bodyMaterials = Map.copyOf(bodyMaterials == null ? Map.of() : bodyMaterials);
  }

  @Override
  public List<ContactConstraint> generateContacts(ImmutableWorldState worldState) {
    return worldState.bodies().stream()
        .filter(body -> body.z() < floorZ)
        .map(this::toContact)
        .toList();
  }

  private ContactConstraint toContact(RigidBodyState body) {
    MaterialProfile bodyMaterial = bodyMaterials.getOrDefault(body.bodyId(), MaterialProfiles.DEFAULT_BODY);
    return new ContactConstraint(
        body.bodyId(),
        body.bodyId() + ":floor",
        0.0,
        0.0,
        1.0,
        floorZ - body.z(),
        MaterialResolver.combinedFriction(bodyMaterial, floorMaterial),
        MaterialResolver.combinedRestitution(bodyMaterial, floorMaterial));
  }
}
