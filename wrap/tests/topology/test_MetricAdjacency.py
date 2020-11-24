import pytest
import dgtal
import copy

def createPoints2D():
    submodule = getattr(dgtal, "kernel")
    Point2D = getattr(submodule, "Point2D")
    pa = Point2D(3, 4)
    pb = Point2D(3, 3)
    return [pa, pb]
def createPoints3D():
    submodule = getattr(dgtal, "kernel")
    Point3D = getattr(submodule, "Point3D")
    pa = Point3D(3, 4, 1)
    pb = Point3D(3, 3, 1)
    return [pa, pb]


@pytest.mark.parametrize("Type", [("Adj4"), ("Adj8"), ("Adj6"), ("Adj18"), ("Adj26")])
def test_MetricAdjacency(Type):
    submodule = getattr(dgtal, "topology")
    Adj = getattr(submodule, Type)
    # All methods are static, but default constructor is also enabled.
    adj = Adj()
    adj = Adj
    print(Adj().__repr__())
    print(Adj())
    if adj.dimension == 2:
        [pa, pb] = createPoints2D()
    elif adj.dimension == 3:
        [pa, pb] = createPoints3D()

    assert adj.isAdjacentTo(pa, pb)
    assert adj.isProperlyAdjacentTo(pa, pb)
    assert not adj.isProperlyAdjacentTo(pa, pa)
    capacity = adj.bestCapacity()
    print("Capacity: ", capacity)
    assert adj.degree(pa) == capacity
    assert adj.degree(pb) == capacity
    neighbors = adj.writeNeighbors(pa)
    print("point pa: ", pa)
    print("point pa neighbors: ", neighbors)
    assert len(neighbors) == capacity

