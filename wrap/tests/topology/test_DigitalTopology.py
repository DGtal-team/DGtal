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
    topo = Topo(foreground=fadj, background=badj, properties=submodule.DigitalTopologyProperties.JORDAN_DT)
    print(topo.__repr__())
    print(topo)

def test_factory():
    # With strings
    topo8_4 = dgtal.DigitalTopology(
        foreground="8", background="4",
        properties=dgtal.topology.DigitalTopologyProperties.JORDAN_DT)
    # With adjacencies
    topo8_4 = dgtal.DigitalTopology(foreground=dgtal.topology.Adj8(),
                                    background=dgtal.topology.Adj4())
    # assert
    fadj = topo8_4.foreground() # returns reference
    assert topo8_4.foreground() != topo8_4.TForegroundAdjacency()
    assert topo8_4.adjacency_pair_string == "8_4"

    with pytest.raises(ValueError):
        dgtal.DigitalTopology(foreground="8",
                              background="8")

    # exercise all the valid pair of adjacencies
    dgtal.DigitalTopology(foreground="4",
                          background="8")
    dgtal.DigitalTopology(foreground="6",
                          background="18")
    dgtal.DigitalTopology(foreground="6",
                          background="26")
    dgtal.DigitalTopology(foreground="18",
                          background="6")
    dgtal.DigitalTopology(foreground="26",
                          background="6")
