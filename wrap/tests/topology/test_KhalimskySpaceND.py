import pytest
import dgtal
import copy

@pytest.mark.parametrize("Type", [("Cell2D"), ("Cell3D")])
def test_Cell(Type):
    submodule = getattr(dgtal, "topology")
    # KhalimskyCell does not allow modification of its preCell data member, a KhalimskyPreCell
    Cell = getattr(submodule, Type)
    cell = Cell()
    assert cell[1] == 0
    # access readonly pre_cell, with readonly property or a function
    assert cell.pre_cell[1] == 0
    assert cell.preCell()[1] == 0
    cell_copy_constructed = Cell(cell)
    assert cell == cell_copy_constructed
    print(cell)
    print(cell_copy_constructed.__repr__())

@pytest.mark.parametrize("Type", [("KSpace2D"), ("KSpace3D")])
def test_KSpace(Type):
    submodule = getattr(dgtal, "topology")
    KSpace = getattr(submodule, Type)
    space = KSpace()
    lower = space.TPoint.diagonal(0)
    upper = space.TPoint.diagonal(20)
    space.init(lower=lower, upper=upper, is_closed=True)
    assert space.Closure.CLOSED == 0
    space.init(lower=lower, upper=upper, closure=space.Closure.CLOSED)

    if space.dimension == 2:
        PreCell = getattr(submodule, "PreCell2D")
        pre_cell = PreCell()
        pre_cell[0] = 2;
        pre_cell[1] = 7;
        SPreCell = getattr(submodule, "SPreCell2D")
        spre_cell_pos = SPreCell(pre_cell.coordinates, True)
        spre_cell_neg = SPreCell(pre_cell.coordinates, False)
        cell = space.uCell(pre_cell)
        scell_pos = space.sCell(spre_cell_pos)
        scell_neg = space.sCell(spre_cell_neg)
        a_point = space.TPoint(10,10)
    elif space.dimension == 3:
        PreCell = getattr(submodule, "PreCell3D")
        pre_cell = PreCell()
        pre_cell[0] = 2;
        pre_cell[1] = 7;
        pre_cell[2] = 2;
        SPreCell = getattr(submodule, "SPreCell3D")
        spre_cell_pos = SPreCell(pre_cell.coordinates, True)
        spre_cell_neg = SPreCell(pre_cell.coordinates, False)
        cell = space.uCell(pre_cell)
        scell_pos = space.sCell(spre_cell_pos)
        scell_neg = space.sCell(spre_cell_neg)
        a_point = space.TPoint(10,10,10)
    print("pre_cell: ", pre_cell)
    print("spre_cell_pos: ", spre_cell_pos)
    print("spre_cell_neg: ", spre_cell_neg)
    print("cell: ", cell)
    print("scell_pos: ", scell_pos)
    print("scell_neg: ", scell_neg)

    # Cell creation services
    assert space.uCell(kpoint=a_point)
    assert space.uCell(point=a_point, cell=pre_cell)
    assert space.sCell(kpoint=a_point) # defaults to sign=True
    assert space.sCell(kpoint=a_point, sign=False)
    assert space.sCell(point=a_point, cell=spre_cell_pos)
    assert space.uSpel(point=a_point)
    assert space.sSpel(point=a_point) # defaults to sign=True
    assert space.sSpel(point=a_point, sign=False)
    assert space.uPointel(point=a_point)
    assert space.sPointel(point=a_point) # defaults to sign=True
    assert space.sPointel(point=a_point, sign=False)

    # Read accessors to cells
    assert space.uKCoord(cell=cell, dim=0)
    assert space.uCoord(cell=cell, dim=0)
    assert space.uKCoords(cell=cell)
    assert space.uCoords(cell=cell)
    assert space.sKCoord(cell=scell_pos, dim=0)
    assert space.sCoord(cell=scell_pos, dim=0)
    assert space.sKCoords(cell=scell_pos)
    assert space.sCoords(cell=scell_pos)
    assert space.sSign(cell=scell_pos)
    assert not space.sSign(cell=scell_neg)

    # Write accessors to cells
    modified_cell = space.uCell(pre_cell)
    modified_scell_pos = space.sCell(spre_cell_pos)
    space.uSetKCoord(cell=modified_cell, dim=0, value=8)
    space.sSetKCoord(cell=modified_scell_pos, dim=0, value=8)
    space.uSetKCoords(cell=modified_cell, value=a_point)
    space.sSetKCoords(cell=modified_scell_pos, value=a_point)
    space.uSetCoord(cell=modified_cell, dim=0, value=8)
    space.sSetCoord(cell=modified_scell_pos, dim=0, value=8)
    space.uSetCoords(cell=modified_cell, value=a_point)
    space.sSetCoords(cell=modified_scell_pos, value=a_point)
    space.sSetSign(cell=modified_scell_pos, sign=False)
    assert not space.sSign(cell=modified_scell_pos)

    # Conversion signed/unsigned
    assert space.signs(cell=cell, sign=True) == scell_pos
    assert space.signs(cell=cell, sign=space.NEG) == scell_neg
    assert space.unsigns(cell=scell_neg) == cell
    assert space.sOpp(cell=scell_pos) == scell_neg
    assert space.sOpp(cell=scell_neg) == scell_pos

    # Topology services
    utopo = space.uTopology(cell=cell)
    stopo = space.sTopology(cell=scell_pos)
    assert utopo == stopo
    uDim = space.uDim(cell=cell)
    sDim = space.sDim(cell=scell_pos)
    assert uDim == sDim
    if space.dimension == 2:
        assert space.uIsSurfel(cell=cell)
    elif space.dimension == 3:
        assert not space.uIsSurfel(cell=cell)
    assert space.uIsOpen(cell=cell, dim=1)
    assert space.sIsOpen(cell=scell_pos, dim=1)
    assert not space.uIsOpen(cell=cell, dim=0)

    # Unsigned geometry services
    assert space.uGetIncr(cell=cell, dim=0)
    assert not space.uIsMax(cell=cell, dim=0)
    assert space.uIsInside(cell=cell, dim=0)
    assert space.uIsInside(cell=cell)
    assert space.uIsInside(cell=pre_cell, dim=0)
    assert space.uIsInside(cell=pre_cell)
    assert space.uGetDecr(cell=cell, dim=0)
    assert not space.uIsMin(cell=cell, dim=0)
    assert space.uGetAdd(cell=cell, dim=0, x=2)
    assert space.uGetSub(cell=cell, dim=0, x=1)

    assert space.uTranslation(cell=cell, vec=a_point)
    assert space.uProjection(cell=cell, bound=cell, dim=1)
    assert space.uIsOpen(cell=cell, dim=1) == space.uIsOpen(cell=cell, dim=1)
    space.uProject(cell=cell, bound=cell, dim=1)
    assert not space.uNext(cell=cell, lower=cell, upper=cell)

    # Signed geometry services
    assert space.sGetIncr(cell=scell_pos, dim=0)
    assert not space.sIsMax(cell=scell_pos, dim=0)
    assert space.sIsInside(cell=scell_pos, dim=0)
    assert space.sIsInside(cell=scell_pos)
    assert space.sIsInside(cell=spre_cell_pos, dim=0)
    assert space.sIsInside(cell=spre_cell_pos)
    assert space.sGetDecr(cell=scell_pos, dim=0)
    assert not space.sIsMin(cell=scell_pos, dim=0)
    assert space.sGetAdd(cell=scell_pos, dim=0, x=2)
    assert space.sGetSub(cell=scell_pos, dim=0, x=1)

    assert space.sTranslation(cell=scell_pos, vec=a_point)
    assert space.sProjection(cell=scell_pos, bound=scell_pos, dim=1)
    assert space.sIsOpen(cell=scell_pos, dim=1) == space.sIsOpen(cell=scell_pos, dim=1)
    space.sProject(cell=scell_pos, bound=scell_pos, dim=1)
    assert not space.sNext(cell=scell_pos, lower=scell_pos, upper=scell_pos)

    # Neighborhood services
    assert space.uNeighborhood(cell=cell)
    assert space.sNeighborhood(cell=scell_pos)
    assert space.uProperNeighborhood(cell=cell)
    assert space.sProperNeighborhood(cell=scell_pos)
    assert space.uAdjacent(cell=cell, dim=0, up=False)
    assert space.sAdjacent(cell=scell_pos, dim=0, up=False)
    assert space.uIncident(cell=cell, dim=0, up=False)
    assert space.sIncident(cell=scell_pos, dim=0, up= False)
    assert space.uLowerIncident(cell=cell)
    assert space.sLowerIncident(cell=scell_pos)
    assert space.uUpperIncident(cell=cell)
    assert space.sUpperIncident(cell=scell_pos)
    assert space.sDirect(cell=scell_pos, dim=0)
    assert space.sDirectIncident(cell=scell_pos, dim=0)
    assert space.sIndirectIncident(cell=scell_pos, dim=0)
    # faces and cofaces return type AnyCellCollection. It needs to be wrapped.
    # Wrapped using py::detail::type_caster specialization in dgtal_pybind11_common.h
    faces = space.uFaces(cell=cell)
    assert faces
    assert len(faces) == 2
    cofaces = space.uCoFaces(cell=cell)
    assert cofaces
    if space.dimension == 2:
        assert len(cofaces) == 2
    elif space.dimension == 3:
        assert len(cofaces) == 8
