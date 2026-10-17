import math

import pytest

from safe_drive_safety.route_geometry import (
    extended_path, point_to_path_distance, vehicle_corners,
)


def test_extended_path_and_corner_distance():
    path = extended_path([(0.0, 0.0), (10.0, 0.0)])
    assert path[0] == (-20.0, 0.0)
    assert path[-1] == (16.0, 0.0)
    assert point_to_path_distance((4.0, 1.5), path) == pytest.approx(1.5)
    corners = vehicle_corners(0.0, 0.0, math.pi / 2, 4.0, 2.0)
    assert max(point_to_path_distance(corner, path) for corner in corners) == pytest.approx(2.0)
