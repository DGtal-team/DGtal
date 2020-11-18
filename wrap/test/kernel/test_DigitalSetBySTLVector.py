import pytest
import dgtal

@pytest.mark.parametrize("Type", [("DigitalSetZ2i"), ("DigitalSetZ3i")])
def test_constructor_with_domain(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    DigitalSet = getattr(kernel_submodule, Type)
    Domain = DigitalSet.TDomain
    Point = DigitalSet.TPoint
    # Conctruct Domain
    lb = Point.zero
    ub = Point.diagonal(4)
    dom = Domain(lb, ub)
    ds = DigitalSet(dom)
    assert ds.domain.lower_bound == dom.lower_bound
    assert ds.domain.upper_bound == dom.upper_bound

@pytest.mark.parametrize("Type", [("DigitalSetZ2i"), ("DigitalSetZ3i")])
def test_function_members(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    DigitalSet = getattr(kernel_submodule, Type)
    Domain = DigitalSet.TDomain
    Point = DigitalSet.TPoint
    # Conctruct Domain
    lb = Point.zero
    ub = Point.diagonal(4)
    dom = Domain(lb, ub)
    ds = DigitalSet(dom)

    # Insert
    ds.insert(Point.diagonal(1))
    ds.insert(Point.diagonal(2))
    assert ds.size() == 2
    assert len(ds) == 2

    # Iter
    point_list = [p for p in ds]
    assert len(point_list) == 2
    assert point_list[0] == Point.diagonal(1)
    assert point_list[1] == Point.diagonal(2)

    # Erase
    ds.erase(Point.diagonal(2))
    assert len(ds) == 1

    # bounding_box
    ds.insert(Point.diagonal(3))
    [lower, upper] = ds.bounding_box()
    assert lower == Point.diagonal(1)
    assert upper == Point.diagonal(3)

    # Complement
    ds_complement = ds.complement()
    expected_number_of_elements_complement = (ub[0] + 1)**Point.dimension - len(ds)
    assert len(ds_complement) == expected_number_of_elements_complement

    # Clear and Empty
    ds.clear()
    assert ds.empty()
