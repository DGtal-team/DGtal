import pytest
import dgtal
import copy

def fixture_PreCell2D():
    Type = "PreCell2D"
    submodule = getattr(dgtal, "topology")
    PreCell = getattr(submodule, Type)
    pre_cell = PreCell()
    pre_cell[0] = 2
    pre_cell[1] = 7
    return pre_cell;

def fixture_PreCell3D():
    Type = "PreCell3D"
    submodule = getattr(dgtal, "topology")
    PreCell = getattr(submodule, Type)
    pre_cell = PreCell()
    pre_cell[0] = 2
    pre_cell[1] = 7
    pre_cell[2] = 2
    return pre_cell;


@pytest.mark.parametrize("Type", [("PreCell2D"), ("PreCell3D")])
def test_PreCell(Type):
    submodule = getattr(dgtal, "topology")
    PreCell = getattr(submodule, Type)
    pre_cell = PreCell()
    pre_cell[0] = 2
    pre_cell[1] = 4
    # Test preCell() returns itself (const)
    assert pre_cell.preCell() == pre_cell
    # Test pickling
    pre_cell_copy = copy.deepcopy(pre_cell)
    assert pre_cell == pre_cell_copy
    assert pre_cell[0] == 2
    assert pre_cell[1] == 4
    print(pre_cell)
    print(pre_cell_copy.__repr__())

@pytest.mark.parametrize("Type", [("PreCell2D"), ("PreCell3D")])
def test_PreCell_bridge_buffer(Type):
    numpy = pytest.importorskip("numpy")
    np = numpy
    submodule = getattr(dgtal, "topology")
    PreCell = getattr(submodule, Type)
    pre_cell = PreCell()
    pre_cell[0] = 2
    pre_cell[1] = 4
    if pre_cell.dimension == 2:
        expected_np_array = np.array([pre_cell[0], pre_cell[1]], dtype='int')
    elif pre_cell.dimension == 3:
        pre_cell[2] = 6
        expected_np_array = np.array([pre_cell[0], pre_cell[1], pre_cell[2]])
    else:
        raise RuntimeError("Dimension not supported")

    # Test def_buffer
    np_array = np.array(pre_cell, copy = False)
    np.testing.assert_array_almost_equal(np_array, expected_np_array)

    # Test constructor from py::buffer
    pre_cell_from_np_array = PreCell(np_array)
    assert pre_cell_from_np_array == pre_cell

@pytest.mark.parametrize("Type", [("SPreCell2D"), ("SPreCell3D")])
def test_SPreCell(Type):
    submodule = getattr(dgtal, "topology")
    PreCell = getattr(submodule, Type)
    pre_cell = PreCell()
    pre_cell[0] = 2
    pre_cell[1] = 4
    # Test preCell() returns itself (const)
    assert pre_cell.preCell() == pre_cell
    # Test pickling
    pre_cell_copy = copy.deepcopy(pre_cell)
    assert pre_cell == pre_cell_copy
    assert pre_cell[0] == 2
    assert pre_cell[1] == 4
    assert pre_cell.positive
    pre_cell.positive = False
    assert not pre_cell.positive
    print(pre_cell)
    print(pre_cell_copy.__repr__())

@pytest.mark.parametrize("Type", [("KPreSpace2D"), ("KPreSpace3D")])
def test_KPreSpace(Type):
    submodule = getattr(dgtal, "topology")
    KPreSpace = getattr(submodule, Type)
    # No need to instantiate the space, all function members are static
    # But instantiation is still valid (default constructor)
    space = KPreSpace()
    space = KPreSpace
    if space.dimension == 2:
        pre_cell = fixture_PreCell2D()
        scell_pos = space.TSCell(pre_cell.coordinates, True)
        scell_neg = space.TSCell(pre_cell.coordinates, False)
        a_point = space.TPoint(10,10)
    elif space.dimension == 3:
        pre_cell = fixture_PreCell3D()
        scell_pos = space.TSCell(pre_cell.coordinates, True)
        scell_neg = space.TSCell(pre_cell.coordinates, False)
        a_point = space.TPoint(10,10,10)

    # Pre-cell creation services
    assert space.uCell(kpoint=a_point)
    assert space.uCell(point=a_point, cell=pre_cell)
    assert space.sCell(kpoint=a_point) # defaults to sign=True
    assert space.sCell(kpoint=a_point, sign=False)
    assert space.sCell(point=a_point, cell=scell_pos)
    assert space.uSpel(point=a_point)
    assert space.sSpel(point=a_point) # defaults to sign=True
    assert space.sSpel(point=a_point, sign=False)
    assert space.uPointel(point=a_point)
    assert space.sPointel(point=a_point) # defaults to sign=True
    assert space.sPointel(point=a_point, sign=False)

    # Read accessors to cells
    assert space.uKCoord(cell=pre_cell, dim=0)
    assert space.uCoord(cell=pre_cell, dim=0)
    assert space.uKCoords(cell=pre_cell)
    assert space.uCoords(cell=pre_cell)
    assert space.sKCoord(cell=scell_pos, dim=0)
    assert space.sCoord(cell=scell_pos, dim=0)
    assert space.sKCoords(cell=scell_pos)
    assert space.sCoords(cell=scell_pos)
    assert space.sSign(cell=scell_pos)
    assert not space.sSign(cell=scell_neg)

    # Write accessors to cells
    modified_pre_cell = copy.deepcopy(pre_cell)
    modified_scell_pos = copy.deepcopy(scell_pos)
    space.uSetKCoord(cell=modified_pre_cell, dim=0, value=8)
    space.sSetKCoord(cell=modified_scell_pos, dim=0, value=8)
    space.uSetKCoords(cell=modified_pre_cell, value=a_point)
    space.sSetKCoords(cell=modified_scell_pos, value=a_point)
    space.uSetCoord(cell=modified_pre_cell, dim=0, value=8)
    space.sSetCoord(cell=modified_scell_pos, dim=0, value=8)
    space.uSetCoords(cell=modified_pre_cell, value=a_point)
    space.sSetCoords(cell=modified_scell_pos, value=a_point)
    space.sSetSign(cell=modified_scell_pos, sign=False)
    assert not space.sSign(cell=modified_scell_pos)

    # Conversion signed/unsigned
    assert space.signs(cell=pre_cell, sign=True) == scell_pos
    assert space.signs(cell=pre_cell, sign=space.NEG) == scell_neg
    assert space.unsigns(cell=scell_neg) == pre_cell
    assert space.sOpp(cell=scell_pos) == scell_neg
    assert space.sOpp(cell=scell_neg) == scell_pos

    # Topology services
    utopo = space.uTopology(cell=pre_cell)
    stopo = space.sTopology(cell=scell_pos)
    assert utopo == stopo
    uDim = space.uDim(cell=pre_cell)
    sDim = space.sDim(cell=scell_pos)
    assert uDim == sDim
    if space.dimension == 2:
        assert space.uIsSurfel(cell=pre_cell)
    elif space.dimension == 3:
        assert not space.uIsSurfel(cell=pre_cell)
    assert space.uIsOpen(cell=pre_cell, dim=1)
    assert space.sIsOpen(cell=scell_pos, dim=1)
    assert not space.uIsOpen(cell=pre_cell, dim=0)

    # Unsigned geometry services
    assert space.uGetIncr(cell=pre_cell, dim=0)
    assert not space.uIsMax(cell=pre_cell, dim=0)
    assert space.uIsInside(cell=pre_cell, dim=0)
    assert space.uIsInside(cell=pre_cell)
    assert space.uGetDecr(cell=pre_cell, dim=0)
    assert not space.uIsMin(cell=pre_cell, dim=0)
    assert space.uGetAdd(cell=pre_cell, dim=0, x=2)
    assert space.uGetSub(cell=pre_cell, dim=0, x=2)

    assert space.uTranslation(cell=pre_cell, vec=a_point)
    assert space.uProjection(cell=pre_cell, bound=pre_cell, dim=1)
    space.uProject(cell=modified_pre_cell, bound=pre_cell, dim=1)
    assert not space.uNext(cell=pre_cell, lower=pre_cell, upper=pre_cell)

    # Signed geometry services
    assert space.sGetIncr(cell=scell_pos, dim=0)
    assert not space.sIsMax(cell=scell_pos, dim=0)
    assert space.sIsInside(cell=scell_pos, dim=0)
    assert space.sIsInside(cell=scell_pos)
    assert space.sGetDecr(cell=scell_pos, dim=0)
    assert not space.sIsMin(cell=scell_pos, dim=0)
    assert space.sGetAdd(cell=scell_pos, dim=0, x=2)
    assert space.sGetSub(cell=scell_pos, dim=0, x=2)

    assert space.sTranslation(cell=scell_pos, vec=a_point)
    assert space.sProjection(cell=scell_pos, bound=scell_pos, dim=1)
    space.sProject(cell=modified_scell_pos, bound=scell_pos, dim=1)
    assert not space.sNext(cell=scell_pos, lower=scell_pos, upper=scell_pos)

    # Neighborhood services
    assert space.uNeighborhood(cell=pre_cell)
    assert space.sNeighborhood(cell=scell_pos)
    assert space.uProperNeighborhood(cell=pre_cell)
    assert space.sProperNeighborhood(cell=scell_pos)
    assert space.uAdjacent(cell=pre_cell, dim=0, up=False)
    assert space.sAdjacent(cell=scell_pos, dim=0, up=False)
    assert space.uIncident(cell=pre_cell, dim=0, up=False)
    assert space.sIncident(cell=scell_pos, dim=0, up= False)
    assert space.uLowerIncident(cell=pre_cell)
    assert space.sLowerIncident(cell=scell_pos)
    assert space.uUpperIncident(cell=pre_cell)
    assert space.sUpperIncident(cell=scell_pos)
    assert space.sDirect(cell=scell_pos, dim=0)
    assert space.sDirectIncident(cell=scell_pos, dim=0)
    assert space.sIndirectIncident(cell=scell_pos, dim=0)
    # faces and cofaces return type AnyCellCollection. It needs to be wrapped.
    # Wrapped using py::detail::type_caster specialization in dgtal_pybind11_common.h
    faces = space.uFaces(cell=pre_cell)
    assert faces
    assert len(faces) == 2
    cofaces = space.uCoFaces(cell=pre_cell)
    assert cofaces
    if space.dimension == 2:
        assert len(cofaces) == 2
    elif space.dimension == 3:
        assert len(cofaces) == 8
