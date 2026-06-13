import pytest

from dgtal import SH3
from dgtal import helpers


def _make_surface_data():
    params = SH3.defaultParameters()
    params.set("verbose", 0)
    bimage = SH3.makeBinaryImage(helpers.SAMPLES_PATH + "/Al.100.vol", params)
    K = SH3.getKSpace(bimage, params)
    surface = SH3.makeLightDigitalSurface(bimage, K, params)
    surfels = SH3.getSurfelRange(surface, params)
    return params, bimage, surfels


def _make_parallel_params():
    params = SH3.defaultParameters()
    params.set("verbose", 0)
    params.set("ii-thread-number", 4)
    params.set("ii-split-axis", 2)
    return params


def test_geometry_estimation_parameters_expose_ii_controls():
    ii_params = SH3.parametersGeometryEstimation()

    assert "ii-thread-number" in ii_params
    assert "ii-split-axis" in ii_params
    assert ii_params.count("ii-thread-number") == 1
    assert ii_params.count("ii-split-axis") == 1
    assert ii_params.isValid()

    merged = SH3.defaultParameters() | ii_params
    merged.set("ii-thread-number", 2)
    merged.set("ii-split-axis", 1)

    assert "ii-thread-number" in merged
    assert "ii-split-axis" in merged
    assert helpers.DGTAL_WITH_OPENMP in (True, False)


def test_ii_mean_gaussian_and_normal_estimators_accept_parallel_parameters():
    params, bimage, surfels = _make_surface_data()
    parallel_params = _make_parallel_params()

    mean_curvatures = SH3.getIIMeanCurvatures(bimage, surfels, params)
    mean_curvatures_parallel = SH3.getIIMeanCurvatures(bimage, surfels, parallel_params)
    gaussian_curvatures = SH3.getIIGaussianCurvatures(bimage, surfels, params)
    gaussian_curvatures_parallel = SH3.getIIGaussianCurvatures(bimage, surfels, parallel_params)
    normal_vectors = SH3.getIINormalVectors(bimage, surfels, params)
    normal_vectors_parallel = SH3.getIINormalVectors(bimage, surfels, parallel_params)

    assert len(mean_curvatures_parallel) == len(mean_curvatures)
    assert len(gaussian_curvatures_parallel) == len(gaussian_curvatures)
    assert len(normal_vectors_parallel) == len(normal_vectors)

    for parallel_value, sequential_value in zip(mean_curvatures_parallel, mean_curvatures):
        assert parallel_value == pytest.approx(sequential_value)

    for parallel_value, sequential_value in zip(gaussian_curvatures_parallel, gaussian_curvatures):
        assert parallel_value == pytest.approx(sequential_value)

    for parallel_vector, sequential_vector in zip(normal_vectors_parallel, normal_vectors):
        for axis in range(3):
            assert parallel_vector[axis] == pytest.approx(sequential_vector[axis])


def test_ii_principal_curvatures_and_directions_are_bound():
    params, bimage, surfels = _make_surface_data()
    parallel_params = _make_parallel_params()

    principal_curvatures = SH3.getIIPrincipalCurvaturesAndDirections(bimage, surfels, params)
    principal_curvatures_parallel = SH3.getIIPrincipalCurvaturesAndDirections(
        bimage, surfels, parallel_params
    )

    assert len(principal_curvatures_parallel) == len(principal_curvatures)

    for parallel_value, sequential_value in zip(principal_curvatures_parallel, principal_curvatures):
        assert parallel_value[0] == pytest.approx(sequential_value[0])
        assert parallel_value[1] == pytest.approx(sequential_value[1])
