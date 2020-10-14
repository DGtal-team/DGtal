from ._dgtal import *

# dgtal.table_folder is used in thin functions
from pathlib import Path as _Path
tables_folder = str(_Path(__file__).parent.absolute() / "tables")
