package org.griffins1884.griffinsim.physics;

public final class MaterialProfiles {
  public static final MaterialProfile DEFAULT_BODY =
      MaterialProfile.of("default-body", 0.8, 0.0, 0.05, 0.03);
  public static final MaterialProfile DEFAULT_FIELD =
      MaterialProfile.of("default-field", 0.9, 0.0, 0.04, 0.02);
  public static final MaterialProfile PAINTED_STEEL =
      MaterialProfile.of("painted-steel", 0.7, 0.05, 0.03, 0.04);
  public static final MaterialProfile HDPE =
      MaterialProfile.of("hdpe", 0.35, 0.1, 0.02, 0.015);
  public static final MaterialProfile CARPET =
      MaterialProfile.of("carpet", 0.95, 0.0, 0.08, 0.02);
  public static final MaterialProfile FOAM_BUMP =
      MaterialProfile.of("foam-bump", 0.85, 0.02, 0.10, 0.03);

  private MaterialProfiles() {}
}
