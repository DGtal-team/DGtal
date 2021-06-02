import pytest
import dgtal
import copy

def test_VoxelComplexMethodsFromCubicalComplex():
    submodule = getattr(dgtal, "topology")
    VComplex = getattr(submodule, "VoxelComplex")
    KSpace = VComplex.TKSpace
    space = KSpace()
    lower = space.TPoint.diagonal(0)
    upper = space.TPoint.diagonal(5)
    space.init(lower=lower, upper=upper, is_closed=True)
    vcomplex = VComplex(space)
    print(vcomplex.__repr__())
    print(vcomplex)

    #inherited from CubicalComplex
    assert vcomplex.REMOVED != 0
    assert vcomplex.COLLAPSIBLE != 0
    assert vcomplex.FIXED != 0
    assert vcomplex.USER1 != 0
    assert vcomplex.VALUE != 0
    if vcomplex.dimension == 2:
        assert vcomplex.dimension == 2
        assert vcomplex.max_size() == 24
    elif vcomplex.dimension == 3:
        assert vcomplex.dimension == 3
        assert vcomplex.max_size() == 36
    vcomplex.clear()
    assert len(vcomplex) == 0
    assert vcomplex.space() == space

    # With Cells
    cell1 = space.uCell(KSpace.TPoint.diagonal(1))
    vcomplex.insertCell(cell1)
    assert vcomplex.size() == 1
    assert not vcomplex.empty()
    cell2 = space.uCell(KSpace.TPoint.diagonal(2))
    vcomplex.insert(cell2)
    assert vcomplex.size() == 2
    if vcomplex.dimension == 2:
        assert vcomplex.nbCells(0) == 1
        assert vcomplex.nbCells(1) == 0
        assert vcomplex.nbCells(2) == 1
    elif vcomplex.dimension == 3:
        assert vcomplex.nbCells(0) == 1
        assert vcomplex.nbCells(1) == 0
        assert vcomplex.nbCells(2) == 0
        assert vcomplex.nbCells(3) == 1

    cells = vcomplex.getCells(dim=0)
    assert len(cells) == 1
    print("cells: ", cells)

    # Test find
    # Cell exists in complex. Return reference.
    cell1_found = vcomplex.find(cell1)
    assert cell1_found == cell1
    # Cell does not exist in complex. Return None.
    point_at_border = KSpace.TPoint.diagonal(5)
    assert space.cIsInside(point_at_border)
    cell_not_in_complex = space.uCell(point_at_border)
    not_found = vcomplex.find(cell_not_in_complex)
    assert not_found == None

    assert vcomplex.belongs(cell1)
    assert not vcomplex.belongs(cell_not_in_complex)

    faces = vcomplex.faces(cell1)
    print("faces: ", faces)
    directFaces = vcomplex.directFaces(cell1)
    print("directFaces: ", directFaces)
    coFaces = vcomplex.coFaces(cell1)
    print("coFaces: ", coFaces)
    directCoFaces = vcomplex.directCoFaces(cell1)
    print("directCoFaces: ", directCoFaces)

    print("test iter(d) for d = 0: ")
    for cell in vcomplex.iter(0):
        print(cell)

    cellBoundary = vcomplex.cellBoundary(cell1)
    print("cellBoundary: ", cellBoundary)
    cellCoBoundary = vcomplex.cellCoBoundary(cell1)
    print("cellCoBoundary: ", cellCoBoundary)

    isCellInterior = vcomplex.isCellInterior(cell1)
    print("isCellInterior: ", isCellInterior)
    isCellBoundary = vcomplex.isCellBoundary(cell1)
    print("isCellBoundary: ", isCellBoundary)

    ccinterior = vcomplex.interior()
    print("ccinterior: ", ccinterior)
    ccboundary = vcomplex.boundary()
    print("ccboundary: ", ccboundary)
    ccclosure = vcomplex.closure(ccomplex=ccinterior)
    print("ccclosure: ", ccclosure)
    ccstar = vcomplex.star(ccomplex=ccinterior)
    print("ccstar: ", ccstar)
    cclink = vcomplex.link(ccomplex=ccinterior)
    print("cclink: ", cclink)

    vcomplex.close()
    vcomplex.open()

    # Operators
    vcother = vcomplex | vcomplex
    print("operator |: ", vcother)
    vcother = vcomplex & vcomplex
    print("operator &: ", vcother)
    vcother = vcomplex - vcomplex
    print("operator -: ", vcother)
    vcother = vcomplex ^ vcomplex
    print("operator ^: ", vcother)
    vcother = ~vcomplex
    print("Close ~: ", vcother)

    # construct
    vc_from_set = VComplex(space)
    DigitalSet = VComplex.TDigitalSet
    Domain = DigitalSet.TDomain
    dom = Domain(lower, upper)
    digital_set = DigitalSet(dom)
    digital_set.insert(Domain.TPoint.diagonal(1))
    digital_set.insert(Domain.TPoint.diagonal(2))
    digital_set.insert(Domain.TPoint.diagonal(3))
    vc_from_set.construct(digital_set)
    print("construct from set: #cells(dim=3)", vc_from_set.nbCells(3))


def test_VoxelComplexThinning():
    submodule = getattr(dgtal, "topology")
    VComplex = getattr(submodule, "VoxelComplex")
    KSpace = VComplex.TKSpace
    space = KSpace()
    lower = space.TPoint.diagonal(0)
    upper = space.TPoint.diagonal(5)
    space.init(lower=lower, upper=upper, is_closed=True)
    vcomplex = VComplex(space)
    cell1 = space.uCell(KSpace.TPoint.diagonal(1))
    vcomplex.insertCell(cell1)
    cell2 = space.uCell(KSpace.TPoint.diagonal(2))
    vcomplex.insertCell(cell2)
    vcomplex.close()
    # Make a copy
    vcomplex_copy = copy.deepcopy(vcomplex)
    vcomplex_copy.thinning()
    print("Thinned: ", vcomplex_copy)
    assert len(vcomplex_copy) == 1
    vcomplex_copy = copy.deepcopy(vcomplex)
    vcomplex_thinned = vcomplex_copy.thinningVoxelComplex(
        skel_type="end", select_type="dmax",
        tables_folder=dgtal.tables_folder)

def test_VoxelComplexSpecificMethods():
    submodule = getattr(dgtal, "topology")
    VComplex = getattr(submodule, "VoxelComplex")
    KSpace = VComplex.TKSpace
    space = KSpace()
    lower = space.TPoint.diagonal(0)
    upper = space.TPoint.diagonal(5)
    space.init(lower=lower, upper=upper, is_closed=True)
    vcomplex = VComplex(space)
    print(vcomplex.__repr__())
    print(vcomplex)

    vcomplex.setSimplicityTable(dgtal.tables_folder)
    assert vcomplex.isTableLoaded()
    # TODO Some functions are yet to be exercised
    # vcomplex.voxelClose(cell=)

