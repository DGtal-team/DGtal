import pytest
import dgtal
import copy

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
