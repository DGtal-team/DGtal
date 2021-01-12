from ._dgtal import *

# dgtal.tables_folder is used in Object and VoxelComplex.
from pathlib import Path as _Path
import sys as _sys
tables_folder = str(_Path(__file__).parent.absolute() / "tables")

def _check_dim(dim):
    """
    Internal function to raise error if dimension is not valid.
    Used in the factory helper functions.
    """
    if dim != 2 and dim != 3:
        raise ValueError("Dimension (dim = " + str(dim) + ") is not valid. Should be 2 or 3.")

def Point(dim=None, dtype='int32', data=None):
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

    Example:
        point2D_empty = dgtal.Point(dim=2)
        point2D = dgtal.Point(data=[2,3])
        real_point3D = dgtal.Point(dtype='float', data=(2,3,4))
        # With numpy.arrays
        real_point3D = dgtal.Point(dtype='float',
                                   data=numpy.array([2,3,4], dtype='float')
                                   )
    Use help(Point(dim=2)) for more info.
    """
    if not dim:
        if data is None:
            raise ValueError("At least provide parameter dim or data")
        else:
            dim = len(data)

    _check_dim(dim)

    if dtype != 'int32' and dtype != 'float':
        raise ValueError("Unhandled dtype (" + dtype + "). Valid: 'int32', and 'float'.")

    if dtype == 'int32':
        if dim == 2:
            if data is None:
                return kernel.Point2D()
            else:
                return kernel.Point2D(*data)
        elif dim == 3:
            if data is None:
                return kernel.Point3D()
            else:
                return kernel.Point3D(*data)
    elif dtype == 'float':
        if dim == 2:
            if data is None:
                return kernel.RealPoint2D()
            else:
                return kernel.RealPoint2D(*data)
        elif dim == 3:
            if data is None:
                return kernel.RealPoint3D()
            else:
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

def _check_valid_adjacency_pair(foreground_adjacency, background_adjacency):
    """
    Given a foreground_adjacency and background_adjacency in the format of either:
    a string (i.e. "8") or an instance of MetricAdjacency (i.e dgtal.topology.Adj8())
    Returns a string if the pair of adjacencies is valid: "8_4" or raise a ValueError.
    """
    if type(foreground_adjacency) == str:
        fadj_t = getattr(topology, "Adj" + foreground_adjacency)
    else:
        fadj_t = type(foreground_adjacency)

    if type(background_adjacency) == str:
        badj_t = getattr(topology, "Adj" + background_adjacency)
    else:
        badj_t = type(background_adjacency)

    # Check that foreground_adjacency and background_adjacency
    # are instances, and not types.
    wrong_type = "<class 'pybind11_builtins.pybind11_type'>"
    if (str(fadj_t) == wrong_type) or (str(badj_t) == wrong_type):
        raise ValueError("foreground/background adjacencies need to be an instance, not a type.\n"
        "Use dgtal.topology.Adj8() instead of dgtal.topology.Adj8")
    # 2D
    # 4-8 and 8-4
    if fadj_t == topology.Adj4 and badj_t == topology.Adj8:
        return "4_8"
    if fadj_t == topology.Adj8 and badj_t == topology.Adj4:
        return "8_4"

    # 3D
    # 6-18, 6-26
    if fadj_t == topology.Adj6:
        if badj_t == topology.Adj18:
            return "6_18"
        elif badj_t == topology.Adj26:
            return "6_26"

    # 18-6, 26-6
    if badj_t == topology.Adj6:
        if fadj_t == topology.Adj18:
            return "18_6"
        elif fadj_t == topology.Adj26:
            return "26_6"

    raise ValueError("Invalid pair of foreground and background adjacency. fadj: " + str(fadj_t) + " , badj: " + str(badj_t) + ".")

def DigitalTopology(foreground, background,
                    properties = topology.DigitalTopologyProperties.UNKNOWN_DT):
    """
    Factory helper function for DGtal::DigitalTopology

    The only valid pair of foreground and background adjacencies are:
    # 2D
    "8_4", "4_8"
    # 3D
    "26_6", "6_26", "18_6", "6_18",

    Parameters
    ----------
    foreground: str or dgtal.MetricAdjacency
        A valid dgtal.MetricAdjacency or a string
        represeting it.

        foreground = dgtal.topology.Adj8()
        foreground = dgtal.MetricAdjacency(dim=2, max_norm=2)
        # Or use a str:
        foreground = "8"
    background: str or dgtal.MetricAdjacency
        A valid dgtal.MetricAdjacency or a string
        represeting it.

        background = dgtal.topology.Adj4()
        background = dgtal.MetricAdjacency(dim=2, max_norm=1)
        # Or use a str:
        background = "4"
    properties: Enum from dgtal.topology.DigitalTopologyProperties
        Default to dgtal.topology.DigitalTopologyProperties.UNKNOWN_DT

    Example:
        topo8_4 = dgtal.DigitalTopology(foreground=dgtal.topology.Adj8,
                                        background=dgtal.topology.Adj4)
        # using str adjacencies:
        dgtal.DigitalTopology(
                  foreground="8",
                  background="4",
                  properties=dgtal.topology.DigitalTopologyProperties.JORDAN_DT)
    """
    adjacency_pair_string = _check_valid_adjacency_pair(foreground, background)
    TDigitalTopology = getattr(topology, "DT" + adjacency_pair_string)
    if type(foreground) == str:
        foreground = TDigitalTopology.TForegroundAdjacency()
    if type(background) == str:
        background = TDigitalTopology.TBackgroundAdjacency()

    return TDigitalTopology(foreground=foreground,
                            background=background,
                            properties=properties)

def Object(topology,
           domain = None, point_set = None,
           connectedness = topology.Connectedness.UNKNOWN):
    """
    Factory helper function for DGtal::Object

    Need to provide one of the parameters: domain or point_set

    Parameters
    ----------
    topology: instance of any dgtal.DigitalTopology
        A valid instance dgtal.DigitalTopology
    domain: dgtal.Domain
        A Domain. It will create an Object with an empty point_set in this domain.
    point_set: dgtal.DigitalSet
        A digital set (which has a domain). It will create an
        Object with the input point_set.
    connectedness: Enum from dgtal.topology.Connectedness
        Default to dgtal.topology.Connectedness.UNKNOWN

    Example:
        topo8_4 = dgtal.DigitalTopology(foreground=dgtal.topology.Adj8,
                                        background=dgtal.topology.Adj4)
        domain = dgtal.Domain(lower_bound=dgtal.Point(dim=2),
                              upper_bound=dgtal.Point(dim=2).diagonal(2))
        # empty object in domain
        empty_object = dgtal.Object(topology=topo8_4, domain=domain)
        empty_object.pointSet().insert(dgtal.Point(data=[1,2]))
        # with an input digital_set
        point_set = dgtal.DigitalSet(domain=domain)
        point_set.insert(dgtal.Point(data=[1,2]))
        object = dgtal.Object(topology=topo8_4, point_set=point_set)
    """
    if not domain and not point_set:
        raise ValueError("Provide one of: domain or point_set (none provided)")
    if domain and point_set:
        raise ValueError("Provide one of: domain or point_set (both provided)")

    # Name clash of module with argument, workaround:
    module_topology = _sys.modules['dgtal._dgtal.topology']
    TObject = getattr(module_topology, "Object" + topology.adjacency_pair_string)
    if domain:
        # Empty object, connectedness is ignored.
        return TObject(topology=topology, domain=domain)
    elif point_set:
        return TObject(topology=topology, point_set=point_set,
                       connectedness=connectedness)

