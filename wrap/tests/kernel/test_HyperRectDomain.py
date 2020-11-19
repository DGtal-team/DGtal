import pytest
import dgtal

@pytest.mark.parametrize("Type", [("DomainZ2i"), ("DomainZ3i")])
def test_constructor_default(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    Type = getattr(kernel_submodule, Type)
    dom = Type()
    assert dom.lower_bound == dom.TPoint.zero
    assert dom.upper_bound == dom.TPoint.diagonal(-1)

@pytest.mark.parametrize("Type", [("DomainZ2i"), ("DomainZ3i")])
def test_constructor_with_points(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    Domain = getattr(kernel_submodule, Type)
    # Integer Points
    Point = Domain.TPoint
    lb = Point.zero
    ub = Point.diagonal(4)
    dom = Domain(lb, ub)
    assert dom.lower_bound == lb
    assert dom.upper_bound == ub
    # Real Points
    RealPoint = Domain.TRealPoint
    rlb = RealPoint.zero
    rub = RealPoint.diagonal(8)
    rdom = Domain(rlb, rub)
    assert rdom.lower_bound == Point.zero
    assert rdom.upper_bound == Point.diagonal(8)

@pytest.mark.parametrize("Type", [("DomainZ2i"), ("DomainZ3i")])
def test_member_functions(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    Domain = getattr(kernel_submodule, Type)
    dom = Domain()
    assert dom.is_empty()

@pytest.mark.parametrize("Type", [("DomainZ2i"), ("DomainZ3i")])
def test_printers(Type):
    kernel_submodule = getattr(dgtal, "kernel")
    Domain = getattr(kernel_submodule, Type)
    dom = Domain()
    print("__str__:")
    print(dom)
    print("__repr__:")
    print(dom.__repr__())
