from dgtal import SH3
from dgtal import helpers
import numpy as np

params = SH3.defaultParameters()
al_capone = SH3.makeBinaryImage(helpers.SAMPLES_PATH + "/Al.100.vol", params)
K = SH3.getKSpace(al_capone, params)
surface = SH3.makeLightDigitalSurface(al_capone, K, params)
surfels = SH3.getSurfelRange(surface, params)
colors = SH3.applyColorMap(np.arange(len(surfels)), params)
SH3.saveOBJ(surface, [], colors, "al.obj")
