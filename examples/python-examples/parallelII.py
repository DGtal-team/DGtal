from dgtal import SH3
from pathlib import Path
import polyscope as ps
import numpy as np
import dgtal

params = SH3.defaultParameters()
params.set("r-radius", 5.0)
# Use the OpenMP runtime's thread count (1 selects sequential II estimation).
params.set("ii-thread-number", 4)
bunny_path = Path(__file__).resolve().parents[1] / "samples" / "bunny-128.vol"
bunny = SH3.makeBinaryImage(str(bunny_path), params)

K = SH3.getKSpace(bunny, params)
surface = SH3.makeLightDigitalSurface(bunny, K, params)
surfels = SH3.getSurfelRange(surface, params)


normals = SH3.getIINormalVectors(bunny, surfels, params)
meanCurvs  = SH3.getIIMeanCurvatures(bunny, surfels, params)

pos = SH3.getEmbeddedPositions(K, surfels)


# Define coordinates of the unit cube
cubes = np.asarray([
    [-0.5, -0.5, -0.5],
    [-0.5,  0.5, -0.5],
    [ 0.5,  0.5, -0.5],
    [ 0.5, -0.5, -0.5],
    [-0.5, -0.5,  0.5],
    [-0.5,  0.5,  0.5],
    [ 0.5,  0.5,  0.5],
    [ 0.5, -0.5,  0.5]
])
# Create a volume mesh of hex cell (cubes)
points  = pos[:, np.newaxis, :] + cubes
points  = points.reshape(len(pos) * 8 , 3)
indices = np.arange(len(pos) * 8).reshape(len(pos), 8)

ps.init()
vmesh = ps.register_volume_mesh("Bunny", points, hexes=indices)
vmesh.add_scalar_quantity("Mean Curvature", np.asarray(meanCurvs), defined_on='cells', enabled=False)
vmesh.add_vector_quantity("Normals", np.asarray(normals), defined_on='cells', enabled=True)
ps.show()
