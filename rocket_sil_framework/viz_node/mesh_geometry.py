"""Visual transform for the bundled rocket_z_up.obj in the mesh viewer.

Length includes the nose; radius describes the cylindrical body, not the fins.
Scaling preserves the asset origin (the rocket datum), without recentering it.
"""
import math

ASSET_LENGTH_M = 2.5
ASSET_BODY_RADIUS_M = 0.15


def rocket_mesh_transform(length_m, radius_m):
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or v <= 0 for v in (length_m, radius_m)):
        raise ValueError('Visual rocket length and radius must be finite positive numbers')
    radial = radius_m / ASSET_BODY_RADIUS_M
    # The viewer imports this Y-up OBJ into Z-up coordinates before the marker
    # transform. Keep the original identity pose: another X rotation tips the nose.
    scale = (radial, radial, length_m / ASSET_LENGTH_M)
    orientation_xyzw = (0.0, 0.0, 0.0, 1.0)
    return scale, orientation_xyzw
