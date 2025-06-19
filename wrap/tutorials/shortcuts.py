import dgtal._dgtal.helpers as SH3 # TODO: Fix ._dgtal 
import numpy as np

params = SH3.defaultParameters()
al_capone = SH3.makeBinaryImage(SH3.SAMPLES_PATH + "/Al.100.vol", params)
K = SH3.getKSpace(al_capone, params)
surface = SH3.makeLightDigitalSurface(al_capone, K, params)
surfels = SH3.getSurfelRange(surface, params)
colors = SH3.applyColorMap(np.arange(len(surfels)).astype(float), params)
SH3.saveOBJ(surface, [], colors, "al.obj")
