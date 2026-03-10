"""
ROS2 适配层占位。

建议后续实现：
- 发布 nav_msgs/OccupancyGrid 到 /map
- 可选发布 tf(map->odom / odom->base_link)
- 提供 map 保存服务
"""


class ROS2MapPublisher:
    def __init__(self, cfg: dict):
        self.cfg = cfg

    def publish(self, grid_prob):
        _ = grid_prob
        # TODO: rclpy + nav_msgs/OccupancyGrid
