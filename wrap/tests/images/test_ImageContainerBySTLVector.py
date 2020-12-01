import pytest
import dgtal

@pytest.mark.parametrize("Type", [
    ("ImageContainerByVector2DInteger"),
    ("ImageContainerByVector2DReal"),
    ("ImageContainerByVector2DFloat"),
    ("ImageContainerByVector3DInteger"),
    ("ImageContainerByVector3DReal"),
    ("ImageContainerByVector3DFloat")])
def test_ImageContainerWithSingleValues(Type):
    submodule = getattr(dgtal, "images")
    ImageContainer = getattr(submodule, Type)
    Domain = ImageContainer.TDomain
    Point = ImageContainer.TPoint
    # Construct Domain
    lb = Point.zero
    ub = Point.diagonal(4)
    dom = Domain(lb, ub)
    img = ImageContainer(dom)
    assert img.domain.lower_bound == dom.lower_bound
    assert img.domain.upper_bound == dom.upper_bound
    assert img.extent
    p1 = Point.diagonal(1)
    p2 = Point.diagonal(2)
    print(img)
    print(img.__repr__())
    assert len(img) == 5 ** Point.dimension
    # Access/Set by point
    assert img[p1] == img[p2]
    img[p1] = 2;
    img[p2] = 4;
    assert img[p1] == 2
    assert img[p2] == 4
    img.translateDomain(shift=p1)
    assert img.domain.lower_bound == dom.lower_bound + p1
    assert img.domain.upper_bound == dom.upper_bound + p1
    # Access/Set by linear index
    assert img[0] == 0
    img[0] = 2
    assert img[0] == 2
    print("TValue: ", img.TValue)

@pytest.mark.parametrize("Type", [
    ("ImageContainerByVector2DColor"),
    ("ImageContainerByVector3DColor"),
    ("ImageContainerByVector2DPoint2D"),
    ("ImageContainerByVector2DRealPoint2D"),
    ("ImageContainerByVector2DPoint3D"),
    ("ImageContainerByVector2DRealPoint3D"),
    ("ImageContainerByVector3DPoint2D"),
    ("ImageContainerByVector3DRealPoint2D"),
    ("ImageContainerByVector3DPoint3D"),
    ("ImageContainerByVector3DRealPoint3D")
])
def test_ImageContainerWithDGtalTypes(Type):
    submodule = getattr(dgtal, "images")
    ImageContainer = getattr(submodule, Type)
    Domain = ImageContainer.TDomain
    Point = ImageContainer.TPoint
    # Construct Domain
    lb = Point.zero
    ub = Point.diagonal(4)
    dom = Domain(lb, ub)
    img = ImageContainer(dom)
    assert img.domain.lower_bound == dom.lower_bound
    assert img.domain.upper_bound == dom.upper_bound
    assert img.extent
    p1 = Point.diagonal(1)
    p2 = Point.diagonal(2)
    assert len(img) == 5 ** Point.dimension
    # Access/Set by point
    assert img[p1] == img[p2]
    print("TValue: ", img.TValue)
    ValueType = img.TValue
    v1 = ValueType()
    img[p1] = v1;
    assert img[p1] == v1


@pytest.mark.parametrize("Type", [
    ("ImageContainerByVector2DInteger"),
    ("ImageContainerByVector2DReal"),
    ("ImageContainerByVector2DFloat"),
    ("ImageContainerByVector3DInteger"),
    ("ImageContainerByVector3DReal"),
    ("ImageContainerByVector3DFloat")])
def test_bridge_buffer(Type):
    numpy = pytest.importorskip("numpy")
    np = numpy
    submodule = getattr(dgtal, "images")
    ImageContainer = getattr(submodule, Type)
    Domain = ImageContainer.TDomain
    Point = ImageContainer.TPoint
    # Construct Domain
    lb = Point.zero
    p1 = Point.diagonal(1)
    if Point.dimension == 2:
        ub = Point(2,3)
        dom = Domain(lb, ub)
        img = ImageContainer(dom)
        dtype = img.TValue
        img[Point(1,0)] = 20
        img[Point(0,1)] = 40
        # Test def_buffer
        np_array = np.array(img, copy=False)
        if Type == "ImageContainerByVector2DInteger":
            print(np_array)
        # Check number of elements
        assert np_array.size == img.domain.size()
        # Check shape
        assert np_array.shape[0] == img.domain.upper_bound[0] + 1
        assert np_array.shape[1] == img.domain.upper_bound[1] + 1
        # f_contiguous (colum major)
        assert np_array[1, 0] == dtype(20)
        assert np_array[0, 1] == dtype(40)
        # modify image through np_array
        np_array[1,1] = dtype(15)
        assert img[Point(1,1)] == dtype(15)
        # Test constructor via array
        img_from_array = ImageContainer(np_array, lower_bound=dom.lower_bound)
        print("img_from_array: ", img_from_array)
        assert img_from_array.domain.lower_bound == dom.lower_bound
        assert img_from_array.domain.upper_bound == dom.upper_bound
        np_array_from_array = np.array(img_from_array)
        # Check number of elements
        assert np_array_from_array.size == img.domain.size()
        # Check shape
        assert np_array_from_array.shape[0] == img.domain.upper_bound[0] + 1
        assert np_array_from_array.shape[1] == img.domain.upper_bound[1] + 1
        if Type == "ImageContainerByVector2DInteger":
            print("np_array_from_array: ", np_array_from_array)
        assert np_array_from_array[1, 0] == dtype(20)
        assert np_array_from_array[0, 1] == dtype(40)
        # assert img_from_array == img
    elif Point.dimension == 3:
        ub = Point(2,3,2)
        dom = Domain(lb, ub)
        img = ImageContainer(dom)
        dtype = img.TValue
        img[Point(1,0,0)] = 20
        img[Point(0,1,0)] = 40
        img[Point(0,0,2)] = 50
        img[Point(0,3,2)] = 100
        np_array = np.array(img, copy=False)
        if Type == "ImageContainerByVector3DInteger":
            print(np_array)
        # Check number of elements
        assert np_array.size == img.domain.size()
        # Check shape
        assert np_array.shape[0] == img.domain.upper_bound[0] + 1
        assert np_array.shape[1] == img.domain.upper_bound[1] + 1
        assert np_array.shape[2] == img.domain.upper_bound[2] + 1
        # f_contiguous (colum major)
        assert np_array[1, 0, 0] == dtype(20)
        assert np_array[0, 1, 0] == dtype(40)
        assert np_array[0, 0, 2] == dtype(50)
        assert np_array[0, 3, 2] == dtype(100)
        # modify image through np_array
        np_array[1,1,1] = dtype(15)
        assert img[Point(1,1,1)] == dtype(15)
        # Test constructor via array
        img_from_array = ImageContainer(np_array, lower_bound=dom.lower_bound)
        print("img_from_array: ", img_from_array)
        assert img_from_array.domain.lower_bound == dom.lower_bound
        assert img_from_array.domain.upper_bound == dom.upper_bound
        np_array_from_array = np.array(img_from_array)
    else:
        raise RuntimeError("Dimension not supported")
