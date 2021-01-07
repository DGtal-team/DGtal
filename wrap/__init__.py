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

def Domain(lower_bound, upper_bound):
    """
    Factory helper function for DGtal::HyperRectDomain.

    Parameters
    ----------
    lower_bound: Point
        lower point of the domain (2D or 3D)
    upper_bound: Point
        upper point of the domain (2D or 3D)

    Example:
      dgtal.Domain(lower_bound=dgtal.Point(dim=2).zero,
                   upper_bound=dgtal.Point(dim=2).diagonal(2))
    """
    if lower_bound.dimension != upper_bound.dimension:
        raise ValueError("lower_bound and upper_bound have different dimension")

    dim = lower_bound.dimension
    if dim == 2:
        return kernel.DomainZ2i(lower_bound=lower_bound, upper_bound=upper_bound)
    elif dim == 3:
        return kernel.DomainZ3i(lower_bound=lower_bound, upper_bound=upper_bound)
