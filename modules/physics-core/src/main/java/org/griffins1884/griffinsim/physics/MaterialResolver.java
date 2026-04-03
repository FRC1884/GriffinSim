package org.griffins1884.griffinsim.physics;

public final class MaterialResolver {
  private MaterialResolver() {}

  public static double combinedFriction(MaterialProfile bodyMaterial, MaterialProfile surfaceMaterial) {
    return Math.sqrt(bodyMaterial.frictionCoefficient() * surfaceMaterial.frictionCoefficient());
  }

  public static double combinedRestitution(MaterialProfile bodyMaterial, MaterialProfile surfaceMaterial) {
    return Math.max(bodyMaterial.restitutionCoefficient(), surfaceMaterial.restitutionCoefficient());
  }

  public static double combinedRollingFriction(
      MaterialProfile bodyMaterial, MaterialProfile surfaceMaterial) {
    return Math.sqrt(
        bodyMaterial.rollingFrictionCoefficient() * surfaceMaterial.rollingFrictionCoefficient());
  }

  public static double combinedTorsionalFriction(
      MaterialProfile bodyMaterial, MaterialProfile surfaceMaterial) {
    return Math.sqrt(
        bodyMaterial.torsionalFrictionCoefficient()
            * surfaceMaterial.torsionalFrictionCoefficient());
  }
}
