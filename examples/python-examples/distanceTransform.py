"""Display the L1 and L2 distance transforms and Voronoi diagrams of a vol object.
"""

from pathlib import Path

from dgtal import SH3, kernel
import numpy as np
import polyscope as ps


params = SH3.defaultParameters()
bunny_path = Path(__file__).resolve().parents[1] / "samples" / "bunny-128.vol"
bunny = SH3.makeBinaryImage(str(bunny_path), params)

# Background voxels are the sites: each foreground voxel receives its
# distance to the nearest background voxel, using the selected metric.
dt_l1 = SH3.makeL1VoronoiMap(bunny, params)
dt_l2 = SH3.makeL2VoronoiMap(bunny, params)
p = kernel.Point3D(45, 45, 45)
print("L1 distance at", p, "=", dt_l1.distanceToClosestSite(p))
print("L2 distance at", p, "=", dt_l2.distanceToClosestSite(p))

# Keep only nonzero distances, so background voxels do not occupy the grid.
occupied_cells, l1, l2 = [], [], []
voronoi_l1, voronoi_l2 = [], []
site_ids = {}
for p in bunny.domain:
    distance = dt_l1.distanceToClosestSite(p)
    if distance > 0:
        occupied_cells.append((p[0], p[1], p[2]))
        l1.append(distance)
        l2.append(dt_l2.distanceToClosestSite(p))
        # Despite its name, directionToClosestSite returns site coordinates.
        # Give each site a unique ID, shared by both metrics.
        for vmap, values in ((dt_l1, voronoi_l1), (dt_l2, voronoi_l2)):
            site = vmap.directionToClosestSite(p)
            key = (site[0], site[1], site[2])
            values.append(site_ids.setdefault(key, len(site_ids)))
occupied_cells = np.asarray(occupied_cells, dtype=np.int32)
l1, l2 = np.asarray(l1), np.asarray(l2)
# Shuffle unique site values reproducibly, using the same mapping for L1 and L2.
site_values = np.random.default_rng(42).permutation(len(site_ids))
voronoi_l1 = site_values[np.asarray(voronoi_l1, dtype=np.intp)]
voronoi_l2 = site_values[np.asarray(voronoi_l2, dtype=np.intp)]

ps.init()
# The origin is a cell corner: offset it to center voxels at integer coordinates.
grid = ps.register_sparse_volume_grid(
    "Bunny distance transform", origin=(-0.5, -0.5, -0.5),
    grid_cell_width=(1.0, 1.0, 1.0), occupied_cells=occupied_cells,
)
# Sparse grids have no add_distance_quantity; use scalar fields with isolines.
value_range = (0.0, float(max(l1.max(), l2.max())))
grid.add_scalar_quantity("L1 distance to background", l1, defined_on="cells",
                         isolines_enabled=True, isoline_period=2.0,
                         isoline_period_relative=False,
                         vminmax=value_range, enabled=False)
grid.add_scalar_quantity("L2 distance to background", l2, defined_on="cells",
                         isolines_enabled=True, isoline_period=2.0,
                         isoline_period_relative=False,
                         vminmax=value_range, enabled=True)
# Cells with the same ID belong to the same Voronoi region.
grid.add_scalar_quantity("L1 Voronoi site ID", voronoi_l1,
                         defined_on="cells", enabled=False)
grid.add_scalar_quantity("L2 Voronoi site ID", voronoi_l2,
                         defined_on="cells", enabled=False)
ps.show()
