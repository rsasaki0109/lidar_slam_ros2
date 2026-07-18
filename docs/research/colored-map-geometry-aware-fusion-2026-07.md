# Geometry-aware camera-colour fusion (2026-07)

## Problem

Robust RGB medoids, exposure balancing, and view confidence improve the colour
chosen from valid camera observations. They cannot repair a candidate set that
already contains the wrong surface: a background LiDAR point projected beside
a foreground silhouette, either side of a depth discontinuity, or a moving
object observed in only part of the sequence. Calibration uncertainty also
makes a fixed pixel guard inconsistent across range and vehicle motion.

## Architecture

The robust projector now constructs a full-resolution per-view z-buffer and
applies four opt-in guards before colour sampling:

1. **Occlusion silhouette:** compare projected depth with the minimum finite
   depth in a circular pixel neighbourhood.
2. **Depth edge:** reject both sides when local finite depth range exceeds an
   absolute plus range-relative threshold.
3. **Dynamic image mask:** reject a pixel when any excluded mask pixel lies in
   its circular neighbourhood.
4. **Calibration uncertainty:** convert accepted 7DoF uncertainty into a
   per-observation pixel radius and add it to enabled geometry guards. The
   translation term scales by focal length over range. Rotation is range
   independent. Residual time uncertainty is multiplied by the camera's local
   linear and angular speeds before projection.

The final radius is configurable and clipped. Larger uncertainty also lowers
the observation quality used by bounded sample selection. The previous robust
fusion path is byte-compatible at default values because all new margins and
the uncertainty multiplier are zero at the library boundary.

## Dynamic-mask contract

Segmentation inference is deliberately outside the mapping core. The adapter
accepts PNG files named after each posed image stem, validates their dimensions,
and writes `dynamic_mask_path` into each frame. Non-zero means excluded. It also
records `dynamic-image-mask-v1` provenance with frame coverage, masked-pixel
fraction, and SHA-256 for every attached file.

The production pipeline stages data as:

```text
posed images -> validated dynamic-mask manifest -> optional 7DoF calibration
             -> geometry-aware robust RGB map -> quality reports
```

Calibration consumes the mask-attached transforms document and preserves the
frame metadata in its corrected output. Cache invalidation includes the source
transforms, mask directory, and individual PNG modification times. Dynamic
exclusion requires a complete mask set; partial manifests are allowed only for
non-exclusion dataset preparation.

## Safety and diagnostics

- Geometry-aware fusion requires a one-pixel z-buffer; coarse bins are rejected.
- Negative radii, thresholds, and uncertainty multipliers are rejected.
- Uncertainty propagation requires finite, strictly increasing frame timestamps
  and an accepted calibration with seven finite non-negative uncertainty values.
- Rejected calibration metadata is never silently used.
- Per-run diagnostics count projected samples, occlusion rejects, depth-edge
  rejects, dynamic-mask rejects, and accepted samples.
- Existing colour options and output selection are unchanged unless explicitly
  enabled.

## Validation status

Unit and integration coverage includes variable-radius depth neighbourhoods,
silhouette rejection, two-sided edge rejection, mask dilation, timestamped
camera motion, uncertainty expansion, manifest hashes and completeness, loader
metadata retention, end-to-end mask consumption, pipeline ordering, cache
invalidation, and legacy opt-in behavior. Production threshold selection and
real-data A/B promotion are intentionally the next dataset-level stage.
