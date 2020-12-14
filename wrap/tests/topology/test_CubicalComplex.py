import pytest
import dgtal
import copy

def test_CubicalCellData():
    submodule = getattr(dgtal, "topology")
    Data = getattr(submodule, "CubicalCellData")
    data = Data()
    assert data.data == 0
    data = Data(12)
    assert data.data == 12

@pytest.mark.parametrize("Type", [
    ("CubicalComplex2D"), ("CubicalComplex3D")])
def test_DigitalTopology(Type):
    submodule = getattr(dgtal, "topology")
    CComplex = getattr(submodule, Type)
    KSpace = CComplex.TKSpace
    space = KSpace()
    lower = space.TPoint.diagonal(0)
    upper = space.TPoint.diagonal(5)
    space.init(lower=lower, upper=upper, is_closed=True)
    ccomplex = CComplex(space)
    print(ccomplex.__repr__())
    print(ccomplex)

    assert ccomplex.REMOVED != 0
    assert ccomplex.COLLAPSIBLE != 0
    assert ccomplex.FIXED != 0
    assert ccomplex.USER1 != 0
    assert ccomplex.VALUE != 0
    if ccomplex.dimension == 2:
        assert ccomplex.dimension == 2
        assert ccomplex.max_size() == 24
    elif ccomplex.dimension == 3:
        assert ccomplex.dimension == 3
        assert ccomplex.max_size() == 36
    ccomplex.clear()
    assert len(ccomplex) == 0
    assert ccomplex.space() == space

    # With Cells
    cell1 = space.uCell(KSpace.TPoint.diagonal(1))
    ccomplex.insertCell(cell1)
    assert ccomplex.size() == 1
    assert not ccomplex.empty()
    cell2 = space.uCell(KSpace.TPoint.diagonal(2))
    ccomplex.insert(cell2)
    assert ccomplex.size() == 2
    if ccomplex.dimension == 2:
        assert ccomplex.nbCells(0) == 1
        assert ccomplex.nbCells(1) == 0
        assert ccomplex.nbCells(2) == 1
    elif ccomplex.dimension == 3:
        assert ccomplex.nbCells(0) == 1
        assert ccomplex.nbCells(1) == 0
        assert ccomplex.nbCells(2) == 0
        assert ccomplex.nbCells(3) == 1

    cells = ccomplex.getCells(dim=0)
    assert len(cells) == 1
    print("cells: ", cells)

    # Test find
    # Cell exists in complex. Return reference.
    cell1_found = ccomplex.find(cell1)
    assert cell1_found == cell1
    # Cell does not exist in complex. Return None.
    point_at_border = KSpace.TPoint.diagonal(5)
    assert space.cIsInside(point_at_border)
    cell_not_in_complex = space.uCell(point_at_border)
    not_found = ccomplex.find(cell_not_in_complex)
    assert not_found == None

    assert ccomplex.belongs(cell1)
    assert not ccomplex.belongs(cell_not_in_complex)

    faces = ccomplex.faces(cell1)
    print("faces: ", faces)
    directFaces = ccomplex.directFaces(cell1)
    print("directFaces: ", directFaces)
    coFaces = ccomplex.coFaces(cell1)
    print("coFaces: ", coFaces)
    directCoFaces = ccomplex.directCoFaces(cell1)
    print("directCoFaces: ", directCoFaces)

    print("test iter(d) for d = 0: ")
    for cell in ccomplex.iter(0):
        print(cell)

    cellBoundary = ccomplex.cellBoundary(cell1)
    print("cellBoundary: ", cellBoundary)
    cellCoBoundary = ccomplex.cellCoBoundary(cell1)
    print("cellCoBoundary: ", cellCoBoundary)

    isCellInterior = ccomplex.isCellInterior(cell1)
    print("isCellInterior: ", isCellInterior)
    isCellBoundary = ccomplex.isCellBoundary(cell1)
    print("isCellBoundary: ", isCellBoundary)

    ccinterior = ccomplex.interior()
    print("ccinterior: ", ccinterior)
    ccboundary = ccomplex.boundary()
    print("ccboundary: ", ccboundary)
    ccclosure = ccomplex.closure(ccomplex=ccinterior)
    print("ccclosure: ", ccclosure)
    ccstar = ccomplex.star(ccomplex=ccinterior)
    print("ccstar: ", ccstar)
    cclink = ccomplex.link(ccomplex=ccinterior)
    print("cclink: ", cclink)

    ccomplex.close()
    ccomplex.open()



