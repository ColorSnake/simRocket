"""Check visual dimensions against the actual bundled OBJ, without ROS."""
import math
from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'rocket_sil_framework/viz_node'))
from mesh_geometry import rocket_mesh_transform

class MeshGeometry(unittest.TestCase):
    def test_transformed_asset_dimensions_and_axis(self):
        vertices = [tuple(map(float, line.split()[1:4])) for line in
                    (ROOT / 'rocket_sil_framework/meshes/rocket_z_up.obj').read_text().splitlines()
                    if line.startswith('v ')]
        for length, radius in [(3.0, .15), (2.0, .3)]:
            scale, (qx, qy, qz, qw) = rocket_mesh_transform(length, radius)
            # Viewer convention: imported OBJ +Y becomes displayed +Z before
            # marker scaling. This models that convention, not a renderer test.
            self.assertEqual((qx, qy, qz, qw), (0.0, 0.0, 0.0, 1.0))
            transformed = [(x*scale[0], -z*scale[1], y*scale[2])
                           for x,y,z in vertices]
            self.assertGreater(transformed[32][2], 0)  # nose points toward +Z
            self.assertAlmostEqual(max(v[2] for v in transformed)-min(v[2] for v in transformed), length)
            # First ring is the cylindrical body; fins must not define body radius.
            self.assertAlmostEqual(max(math.hypot(x,y) for x,y,z in transformed[:16]), radius, delta=radius * 0.001)  # OBJ coordinates are rounded to 4 decimals
            self.assertAlmostEqual(sum(v*v for v in (qx,qy,qz,qw)), 1)

    def test_invalid_dimensions(self):
        for value in [0, -1, float('nan'), float('inf'), True, '3']:
            with self.assertRaises(ValueError): rocket_mesh_transform(value, .15)
            with self.assertRaises(ValueError): rocket_mesh_transform(3, value)

if __name__ == '__main__': unittest.main()
