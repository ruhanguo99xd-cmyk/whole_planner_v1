import numpy as np

from lite_slam.slam.occupancy_grid import GridParams, OccupancyGrid2D


def test_integrate_points_increases_occupancy():
    p = GridParams(
        resolution_m=1.0,
        width_cells=10,
        height_cells=10,
        origin_x_m=0.0,
        origin_y_m=0.0,
        logodds_hit=1.0,
        logodds_miss=-0.2,
        logodds_min=-4.0,
        logodds_max=4.0,
        occupied_threshold=0.65,
    )
    g = OccupancyGrid2D(p)
    g.integrate_points(np.array([1.0]), np.array([2.0]))
    prob = g.to_probability()
    assert prob[2, 1] > 0.5
