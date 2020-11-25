from ._dgtal import *

# dgtal.tables_folder is used in Object and VoxelComplex.
from pathlib import Path as _Path
tables_folder = str(_Path(__file__).parent.absolute() / "tables")
