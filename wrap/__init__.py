from ._dgtal import *

# dgtal.tables_folder is used in Object and VoxelComplex.
from pathlib import Path as _Path
tables_folder = str(_Path(__file__).parent.absolute() / "tables")

def _check_dim(dim):
    """
    Internal function to raise error if dimension is not valid.
    Used in the factory helper functions.
    """
    if dim != 2 and dim != 3:
        raise ValueError("Dimension (dim = " + str(dim) + ") is not valid. Should be 2 or 3.")

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
    _check_dim(dim)

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
    _check_dim(dim)

    if dim == 2:
        return kernel.DomainZ2i(lower_bound=lower_bound, upper_bound=upper_bound)
    elif dim == 3:
        return kernel.DomainZ3i(lower_bound=lower_bound, upper_bound=upper_bound)

def DigitalSet(domain):
    """
    Factory helper function for DGtal::DigitalSetBySTLVector

    Parameters
    ----------
    domain: Domain
        A domain of the desired dimension (2 or 3)

    Example:
    domain = dgtal.Domain(lower_bound=dgtal.Point(dim=2).zero,
                          upper_bound=dgtal.Point(dim=2).diagonal(2))
    digital_set = dgtal.DigitalSet(domain=domain)
    """
    dim = domain.dimension
    _check_dim(dim)

    if dim == 2:
        return kernel.DigitalSetZ2i(domain=domain)
    elif dim == 3:
        return kernel.DigitalSetZ3i(domain=domain)

def KPreSpace(dim):
    """
    Factory helper function for DGtal::KhalimskyPreSpace

    Parameters
    ----------
    dim: Int
        Dimension of the space (2D or 3D)

    Example:
    pre_ks = dgtal.KPreSpace(dim=2)
    """
    _check_dim(dim)

    if dim == 2:
        return topology.KPreSpace2D()
    elif dim == 3:
        return topology.KPreSpace3D()

def PreCell(dim=(), point=()):
    """
    Factory helper function for DGtal::PreCell

    Parameters
    ----------
    dim: Int
        Dimension of the space (2D or 3D)
    point: dgtal.Point
        If no point provided, return a PreCell of the input dimension.

    Example:
    pre_cell_empty = dgtal.PreCell(dim=2)
    pre_cell = dgtal.PreCell(point=dgtal.Point(dim=2).diagonal(2))
    """
    if not dim and not point:
        raise ValueError("Provide at least one of the following parameters: dim or point")

    if point:
        dim_point = point.dimension
        if dim:
            if dim != dim_point:
                raise ValueError("dim and point dimension are different. Provide only a valid point.")

        _check_dim(dim_point)
        if dim_point == 2:
            return topology.PreCell2D(point=point)
        elif dim_point == 3:
            return topology.PreCell3D(point=point)
    else:
        _check_dim(dim)
        if dim == 2:
            return topology.PreCell2D()
        elif dim == 3:
            return topology.PreCell3D()

def SPreCell(dim=(), point=(), positive=True):
    """
    Factory helper function for DGtal::PreCell

    Parameters
    ----------
    dim: Int
        Dimension of the space (2D or 3D)
    point: dgtal.Point of the same dimension
        If empty (default) returns a default constructed PreCell
    positive: Bool [True by default]
        Sign of the cell.

    Example:
    spre_cell_empty = dgtal.SPreCell(dim=2)
    spre_cell_pos = dgtal.SPreCell(point=dgtal.Point(dim=2).diagonal(2))
    spre_cell_neg = dgtal.SPreCell(point=dgtal.Point(dim=2).diagonal(2),
                                   positive=False)
    """
    if not dim and not point:
        raise ValueError("Provide at least one of the following parameters: dim or point")

    if point:
        dim_point = point.dimension
        if dim:
            if dim != dim_point:
                raise ValueError("dim and point dimension are different. Provide only a valid point.")

        _check_dim(dim_point)
        if dim_point == 2:
            return topology.SPreCell2D(point=point, positive=positive)
        elif dim_point == 3:
            return topology.SPreCell3D(point=point, positive=positive)
    else:
        _check_dim(dim)
        if dim == 2:
            pre_cell = topology.SPreCell2D()
        elif dim == 3:
            pre_cell = topology.SPreCell3D()
        pre_cell.positive = positive
        return pre_cell


def KSpace(dim):
    """
    Factory helper function for DGtal::KhalimskySpace
    Call:
        kspace.init(lower=dgtal.Point, upper=dgtal.Point , is_closed=True|False)
    to initialize it to a valid state.

    Parameters
    ----------
    dim: Int
        Dimension of the space (2D or 3D)

    Example:
    ks = dgtal.KSpace(dim=2)
    ks.init(lower=dgtal.Point(dim=2).zero,
            upper=dgtal.Point(dim=2).diagonal(2),
            is_closed=True)
    """
    _check_dim(dim)

    if dim == 2:
        return topology.KSpace2D()
    elif dim == 3:
        return topology.KSpace3D()

def MetricAdjacency(dim, max_norm):
    """
    Factory helper function for DGtal::MetricAdjacency

    Parameters
    ----------
    dim: Int
        Dimension of the space (2D or 3D)
    max_norm: Int
        Defines which point are adjacents. Valid: [1, ... , dim]

    Example:
        # 3D
        adj26 = dgtal.MetricAdjacency(dim=3, max_norm=3)
        adj18 = dgtal.MetricAdjacency(dim=3, max_norm=2)
        adj6 = dgtal.MetricAdjacency(dim=3, max_norm=1)
        # 2D
        adj8 = dgtal.MetricAdjacency(dim=2, max_norm=2)
        adj4 = dgtal.MetricAdjacency(dim=2, max_norm=1)

    These metrics are also available using:
        adj26 = dgtal.kernel.Adj26()
    """
    _check_dim(dim)
    if max_norm < 1 or max_norm > dim:
        raise ValueError("max_norm must be greater than 0, and not greater than input dimension.")

    if dim == 2:
        if max_norm == 1:
            return topology.Adj4()
        elif max_norm == 2:
            return topology.Adj8()
    elif dim == 3:
        if max_norm == 1:
            return topology.Adj6()
        elif max_norm == 2:
            return topology.Adj18()
        elif max_norm == 3:
            return topology.Adj26()
