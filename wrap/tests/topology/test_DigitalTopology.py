import pytest
import dgtal
import copy

@pytest.mark.parametrize("Type", [
    ("DT4_8"), ("DT8_4"), ("DT6_18"), ("DT18_6"), ("DT6_26"), ("DT26_6")])
def test_DigitalTopology(Type):
    submodule = getattr(dgtal, "topology")
    Topo = getattr(submodule, Type)
    FAdj = Topo.TForegroundAdjacency
    BAdj = Topo.TBackgroundAdjacency

    # We need instances of MetricAdjacencies for topo constructor.
    fadj = FAdj()
    badj = BAdj()
    topo = Topo(fadj, badj)
    assert topo.foreground() == fadj
    assert topo.background() == badj
    assert topo.properties() == submodule.DigitalTopologyProperties.UNKNOWN_DT
    topo = Topo(foreground=fadj, background=badj, props=submodule.DigitalTopologyProperties.JORDAN_DT)
    print(topo.__repr__())
    print(topo)
