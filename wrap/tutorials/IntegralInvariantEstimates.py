from dgtal import SH3
from dgtal import helpers
import polyscope as ps
import numpy as np

params = SH3.defaultParameters()
al_capone = SH3.makeBinaryImage(helpers.SAMPLES_PATH + "/Al.100.vol", params)
K = SH3.getKSpace(al_capone, params)
surface = SH3.makeLightDigitalSurface(al_capone, K, params)
surfels = SH3.getSurfelRange(surface, params)

normals = SH3.getIINormalVectors(al_capone, surfels, params)
meanCurvs  = SH3.getIIMeanCurvatures(al_capone, surfels, params)
gaussCurvs = SH3.getIIGaussianCurvatures(al_capone, surfels, params)



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
vmesh = ps.register_volume_mesh("Al Capone", points, hexes=indices)
vmesh.add_scalar_quantity("Mean Curvature", np.asarray(meanCurvs), defined_on='cells', enabled=False)
vmesh.add_scalar_quantity("Gaussian Curvature", np.asarray(gaussCurvs), defined_on='cells', enabled=True)
vmesh.add_vector_quantity("Normals", np.asarray(normals), defined_on='cells', enabled=True)
ps.show()
