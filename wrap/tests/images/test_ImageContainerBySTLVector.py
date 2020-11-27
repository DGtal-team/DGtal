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
    ("ImageContainerByVector3DColor")])
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


