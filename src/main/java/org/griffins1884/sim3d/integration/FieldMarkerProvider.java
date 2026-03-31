package org.griffins1884.sim3d.integration;

/** Supplies static field markers for visualization tools such as AdvantageScope field overlays. */
public interface FieldMarkerProvider {
  FieldMarkerSample[] getFieldMarkers();
}
