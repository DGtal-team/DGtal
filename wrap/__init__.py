from ._dgtal import *

# dgtal.tables_folder is used in Object and VoxelComplex.
from pathlib import Path as _Path
tables_folder = str(_Path(__file__).parent.absolute() / "tables")

def Point(dim, dtype='int32', data=()):
    """
    Factory helper function for DGtal::PointVector.

    Parameters
    ----------
    dim: Int
        Dimension of the returning point (2D or 3D)
    dtype: String. 'int32' or 'float' ['int32']
        If dtype='int32' and dim=2 a Point2D will be returned
    data: Tuple, List or any iterable
        It will be passed as positional arguments to the constructor of DGtal::PointVector

    Options:
        Point2D: dim=2, dtype='int32'
        Point3D: dim=3, dtype='int32'
        RealPoint2D: dim=2, dtype='float'
        RealPoint3D: dim=3, dtype='float'

    Use help(Point(dim=2)) for more info.
    """
    if dim != 2 and dim != 3:
        raise ValueError("Dimension (dim = " + dim + ") is not valid. Should be 2 or 3.")
    if dtype != 'int32' and dtype != 'float':
        raise ValueError("Unhandled dtype (" + dtype + "). Valid: 'int32', and 'float'.")

    if dtype == 'int32':
        if dim == 2:
            return kernel.Point2D(*data)
        elif dim == 3:
            return kernel.Point3D(*data)
    elif dtype == 'float':
        if dim == 2:
            return kernel.RealPoint2D(*data)
        elif dim == 3:
            return kernel.RealPoint3D(*data)
