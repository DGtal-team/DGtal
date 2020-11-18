import pytest
import dgtal

import copy
import functools
import math


@pytest.mark.parametrize("Type", [
    ("Point2D"), ("RealPoint2D"),
    ("Point3D"), ("RealPoint3D")])
def test_default_constructor(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    Type = getattr(kernel_submodule, Type)
    p = Type()
    assert p[0] == 0

@pytest.mark.parametrize("Type", [
    ("Point2D"),
    ("RealPoint3D")])
def test_copy_constructor(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    Type = getattr(kernel_submodule, Type)
    p = Type()
    p[0] = 2
    p[1] = 4
    p_copy_constructed = Type(p)
    assert p_copy_constructed[0] == 2
    assert p_copy_constructed[1] == 4

@pytest.mark.parametrize("Type", [
    ("Point2D"),
    ("RealPoint2D")])
def test_2D_constructor(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    Type = getattr(kernel_submodule, Type)
    p = Type(2, 4)
    assert p[0] == 2
    assert p[1] == 4
    with pytest.raises(TypeError):
        Type(2,4,6)

@pytest.mark.parametrize("Type", [
    ("Point3D"),
    ("RealPoint3D")])
def test_3D_constructor(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    Type = getattr(kernel_submodule, Type)
    p = Type(2, 4, 6)
    assert p[0] == 2
    assert p[1] == 4
    assert p[2] == 6
    with pytest.raises(TypeError):
        Type(2,4)

@pytest.mark.parametrize("Self, Other", [
    ("Point2D", "Point2D"),
    ("RealPoint2D", "RealPoint2D"),
    ("Point2D", "RealPoint2D"),
    ("RealPoint2D", "Point2D"),
    ("Point3D", "Point3D"),
    ("RealPoint3D", "RealPoint3D"),
    ("Point3D", "RealPoint3D"),
    ("RealPoint3D", "Point3D")])
def test_mixings_any_type(Self, Other):
    kernel_submodule = getattr(dgtal, "kernel")
    Self = getattr(kernel_submodule, Self)
    Other = getattr(kernel_submodule, Other)
    ps = Self()
    ps[0] = 2
    ps[1] = 4
    po = Other()
    po[0] = 1
    po[1] = 1
    if ps.dimension == 3:
        ps[2] = 6
    if po.dimension == 3:
        po[2] = 1

    ### Arithmetic operators
    add = ps + po
    assert add[0] == 3
    sub = ps - po
    assert sub[0] == 1
    mul = ps * po
    assert mul[0] == 2
    div = ps / po
    assert div[0] == 2


@pytest.mark.parametrize("Self, Other", [
    ("Point2D", "RealPoint2D"),
    ("RealPoint3D", "Point3D")])
def test_mixings_different_type(Self, Other):
    kernel_submodule = getattr(dgtal, "kernel")
    Self = getattr(kernel_submodule, Self)
    Other = getattr(kernel_submodule, Other)
    ps = Self()
    ps[0] = 2
    ps[1] = 4
    po = Other()
    po[0] = 1
    po[1] = 1

    ### Arithmetic in-place operators not possible with any other type
    in_add = copy.deepcopy(ps)
    with pytest.raises(TypeError):
        in_add += po

    in_sub = copy.deepcopy(ps)
    with pytest.raises(TypeError):
        in_sub -= po

    in_mul = copy.deepcopy(ps)
    with pytest.raises(TypeError):
        in_mul *= po

    in_div = copy.deepcopy(ps)
    with pytest.raises(TypeError):
        in_div /= po

@pytest.mark.parametrize("Self, Other", [
    ("Point2D", "Point2D"),
    ("RealPoint3D", "RealPoint3D")])
def test_mixings_same_type(Self, Other):
    kernel_submodule = getattr(dgtal, "kernel")
    Self = getattr(kernel_submodule, Self)
    Other = getattr(kernel_submodule, Other)
    ps = Self()
    ps[0] = 2
    ps[1] = 4
    po = Other()
    po[0] = 1
    po[1] = 1

    ### Arithmetic in-place operators
    in_add = copy.deepcopy(ps)
    in_add += ps
    assert in_add[0] == ps[0] + ps[0]

    in_sub = copy.deepcopy(ps)
    in_sub -= ps
    assert in_sub[0] == ps[0] - ps[0]

    in_mul = copy.deepcopy(ps)
    in_mul *= ps
    assert in_mul[0] == ps[0] * ps[0]

    in_div = copy.deepcopy(ps)
    in_div /= ps
    assert in_div[0] == ps[0] / ps[0]
    assert in_div[0] == 1

    ### Comparison operators
    assert not ps == po
    assert ps != po
    assert not ps < po
    assert ps > po
    assert not ps <= po
    assert ps >= po

@pytest.mark.parametrize("Self", [
    ("Point2D"),
    ("RealPoint3D")])
def test_function_members(Self):
    kernel_submodule = getattr(dgtal, "kernel")
    Self = getattr(kernel_submodule, Self)
    ps = Self()
    ps[0] = 2
    ps[1] = 4
    if ps.dimension == 3:
        ps[2] = 6

    assert ps.max() == ps[ps.dimension -1]
    assert ps.min() == ps[0]
    expected_squared_norm = functools.reduce(
        lambda a,b: a+(b*b), [0] + [ps[i] for i in range(ps.dimension)])
    assert ps.squaredNorm() == expected_squared_norm
    assert ps.norm() == math.sqrt(expected_squared_norm)
    assert ps.norm1()
    assert ps.normInfinity()
    base_k1 = ps.base(1, 14)
    assert base_k1[1] == 14

    # In place functions
    negated = copy.deepcopy(ps)
    negated.negate()
    assert negated == -ps

    cleared = copy.deepcopy(ps)
    cleared.clear()
    assert cleared == ps.zero

    reseted = copy.deepcopy(ps)
    reseted.reset()
    assert reseted == ps.zero


@pytest.mark.parametrize("Self", [
    ("Point2D"),
    ("RealPoint3D")])
def test_bridge_buffer(Self):
    numpy = pytest.importorskip("numpy")
    np = numpy
    kernel_submodule = getattr(dgtal, "kernel")
    Self = getattr(kernel_submodule, Self)
    ps = Self()
    ps[0] = 2
    ps[1] = 4
    dtype = getattr(np, ps.dtype)
    if ps.dimension == 2:
        expected_np_array = np.array([ps[0], ps[1]], dtype=dtype)
    elif ps.dimension == 3:
        ps[2] = 6
        expected_np_array = np.array([ps[0], ps[1], ps[2]], dtype=dtype)
    else:
        raise RuntimeError("Dimension not supported")

    # Test def_buffer
    np_array = np.array(ps, copy = False)
    np.testing.assert_array_almost_equal(np_array, expected_np_array)

    # Test constructor from py::buffer
    ps_from_np_array = Self(np_array)
    assert ps_from_np_array == ps


@pytest.mark.parametrize("Type", [("Point2D"), ("RealPoint3D")])
def test_printers(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    Type = getattr(kernel_submodule, Type)
    ps = Type()
    print("__str__:")
    print(ps)
    print("__repr__:")
    print(ps.__repr__())
