import pytest
import dgtal
import copy

@pytest.mark.parametrize("Type", [
    ("Object4_8"), ("Object8_4"), ("Object6_18"), ("Object18_6"), ("Object6_26"), ("Object26_6")])
def test_Object(Type):
    submodule = getattr(dgtal, "topology")
    Object = getattr(submodule, Type)
    # Instantitate DigitalTopology
    Topo = Object.TDigitalTopology
    FAdj = Topo.TForegroundAdjacency
    BAdj = Topo.TBackgroundAdjacency
    fadj = FAdj()
    badj = BAdj()
    topo = Topo(fadj, badj)
    # Instantitate DigitalSet
    DigitalSet = Object.TDigitalSet
    Domain = DigitalSet.TDomain
    Point = Object.TPoint
    # Construct Domain
    lb = Point.zero
    ub = Point.diagonal(10)
    dom = Domain(lb, ub)
    ds = DigitalSet(dom)
    p1 = Point.diagonal(1)
    p2 = Point.diagonal(2)
    ds.insert(p1)
    ds.insert(p2)
    # Create object with topo and digital set
    obj = Object(topo, ds)
    # Load table to speed up isSimple computations (this takes some time)
    assert obj.setTable(dgtal.tables_folder)
    assert obj.isSimple(p1) == obj.isSimple(p2)
    assert obj.size()
    assert obj.domain()
    assert obj.pointSet()
    assert obj.topology()
    assert obj.adjacency()
    assert obj.connectedness() == submodule.Connectedness.UNKNOWN
    obj.computeConnectedness()
    assert obj.connectedness() != submodule.Connectedness.UNKNOWN
    assert obj.border()
    assert obj.degree(vertex=p1) == obj.degree(vertex=p2)
    assert obj.bestCapacity()
    assert len(obj.writeNeighbors(vertex=p1)) == len(obj.writeNeighbors(vertex=p2))
    assert len(obj.neighborhood(p1)) == len(obj.neighborhood(p2))
    assert obj.neighborhoodSize(p1) == obj.neighborhoodSize(p2)
    assert len(obj.properNeighborhood(p1)) == len(obj.properNeighborhood(p2))
    assert obj.properNeighborhoodSize(p1) == obj.properNeighborhoodSize(p2)
    assert obj.geodesicNeighborhood(p1, 0).size() == obj.geodesicNeighborhood(p2, 0).size()
    assert obj.geodesicNeighborhoodInComplement(p1, 1).size() == obj.geodesicNeighborhoodInComplement(p2, 1).size()
