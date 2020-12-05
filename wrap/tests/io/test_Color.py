import pytest
import dgtal
import copy

def test_Color():
    submodule = getattr(dgtal, "io")
    Color = getattr(submodule, "Color")
    # Default Constructor
    color = Color()
    assert color.red == 0
    assert color.r()  == 0.0
    # Constructor rgba
    color = Color(10, 11, 12, 200)
    assert color.alpha == 200
    color = Color(red=10, green=11, blue=12, alpha=200)
    # Constructor with gray value
    color = Color(gray_value=200)
    assert color.green == 200
    assert color.alpha == 255
    # Constructor unsigned int
    color = Color(rgb=12390, alpha=10) # Why not number
    assert color.red == 0
    assert color.green == 48
    assert color.blue == 102
    assert color.alpha == 10

    other_color = copy.deepcopy(color)
    other_color.red = 10
    other_color.green = 10
    other_color.blue = 10
    other_color.alpha = 20

    sum_c = color + other_color
    minus_c = color - other_color
    assert minus_c < color
    minus_c += color.Red
    assert minus_c.red == 255
    # Cannot use None in python, rename None to Invalid
    assert color.Invalid == Color(gray_value=0, alpha=0)
    assert not color.Invalid.valid()
    assert other_color.valid()

    assert other_color.tikz()
    assert other_color.svg()
    assert other_color.svgAlpha("foo")
    assert other_color.postscript()

    assert other_color.getRGB()
    assert other_color.getRGBA()
    # This spotted a bug in getRGBA
    assert other_color.setRGBA(sum_c.getRGBA())
    assert other_color == sum_c

    assert other_color.setRGBf(red=minus_c.r(),
                               green=minus_c.g(),
                               blue=minus_c.b(),
                               alpha=minus_c.a())
    assert other_color == minus_c

def test_Color_buffer():
    numpy = pytest.importorskip("numpy")
    np = numpy
    submodule = getattr(dgtal, "io")
    Color = getattr(submodule, "Color")
    color = Color(10, 11, 12, 200)
    # def_buffer
    np_color = np.array(color)
    print("np_color: ", np_color.__repr__())
    np.testing.assert_array_equal(np_color, np.array([10, 11, 12, 200], dtype='uint8'))
    # constructor from buffer
    color_from_np_array = Color(np_color)
    print("color_from_np_array: ", color_from_np_array)
    assert color_from_np_array == color


