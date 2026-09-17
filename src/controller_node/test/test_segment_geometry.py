import numpy as np

from controller_node.adapters._segment_geometry import segment_segment_distance


def test_parallel_segments():
    distance = segment_segment_distance(
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([1.0, 1.0, 0.0]),
    )
    assert distance == 1.0


def test_perpendicular_skew_segments():
    # Clásico: uno a lo largo de X en z=1, el otro a lo largo de Y en z=0 --
    # los puntos más cercanos son (0,0,1) y (0,0,0), distancia 1.
    distance = segment_segment_distance(
        np.array([-1.0, 0.0, 1.0]),
        np.array([1.0, 0.0, 1.0]),
        np.array([0.0, -1.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
    )
    assert distance == 1.0


def test_crossing_segments_have_zero_distance():
    distance = segment_segment_distance(
        np.array([0.0, 0.0, 0.0]),
        np.array([2.0, 0.0, 0.0]),
        np.array([1.0, -1.0, 0.0]),
        np.array([1.0, 1.0, 0.0]),
    )
    assert distance == 0.0


def test_segments_sharing_an_endpoint_have_zero_distance():
    # El caso real de dos eslabones consecutivos, unidos por su
    # articulación -- 0.0 aquí no significa autocolisión, ver
    # self_collision_planning_adapter.py sobre por qué se excluyen.
    distance = segment_segment_distance(
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([1.0, 1.0, 0.0]),
    )
    assert distance == 0.0


def test_point_versus_segment():
    distance = segment_segment_distance(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([1.0, 1.0, 0.0]),
    )
    assert distance == 1.0


def test_point_versus_point():
    distance = segment_segment_distance(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.array([3.0, 4.0, 0.0]),
        np.array([3.0, 4.0, 0.0]),
    )
    assert distance == 5.0
