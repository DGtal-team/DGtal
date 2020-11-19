import pytest
import dgtal
import copy

@pytest.mark.parametrize("Type", [("Cell2D"), ("Cell3D")])
def test_Cell(Type):
    # KhalimskyCell does not allow modification of its preCell data member, a KhalimskyPreCell
    submodule = getattr(dgtal, "topology")
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
