# 只执行ros2动作逻辑，不直接指挥PLC，适合与ROS2系统联调验证动作流程
# 执行完后在ui界面执行动作
try:
    import snap7  # noqa: F401
    from snap7 import util  # noqa: F401
except ImportError:
    snap7 = None
    util = None
import time
import numpy as np
# ros2通讯相关
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool  # 用Bool类型传递启动/完成信号，这是ROS2中自带标准通讯消息类型
from shovel_interfaces.action import Dig

# ======================== ROS2动作服务端节点 ========================
class DigActionServerNode(Node):
    def __init__(self):
        super().__init__('plc_control')
        self.callback_group = ReentrantCallbackGroup()
        # 1. 创建发布者：向规划系统发送"准许感知启动计算"信号（话题名可根据实际ROS2系统修改）
        self.start_pub = self.create_publisher(
            Bool, 'digging/perception_start', 10, callback_group=self.callback_group
        )
        self.excavation_start_pub = self.create_publisher(
            Bool, 'digging/excavation_start', 10, callback_group=self.callback_group
        )
        # 2. 创建订阅者：接收规划系统"感知计算完成"信号（话题名需与感知系统一致）
        self.finish_sub = self.create_subscription(
            Bool, 'digging/perception_finish', self.finish_callback, 10,
            callback_group=self.callback_group
        )
        # 3. 标志位：标记是否收到感知完成信号
        self.perception_finished = False
        # 4. 动作服务端：接收dig.action的start并执行流程
        self.action_server = ActionServer(
            self,
            Dig,
            '/dig',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info("Dig action服务端已启动，等待客户端请求")

    # 感知完成信号回调函数
    def finish_callback(self, msg):
        if msg.data:  # 若收到True信号，说明感知计算完成
            self.perception_finished = True
            self.get_logger().info("感知系统计算完成")

    # 发布"启动感知"信号的函数
    def publish_start_signal(self):
        start_msg = Bool()
        start_msg.data = True
        self.start_pub.publish(start_msg)
        self.get_logger().info("向挖掘节点发送允许感知计算信号")

    def publish_excavation_start(self):
        start_msg = Bool()
        start_msg.data = True
        self.excavation_start_pub.publish(start_msg)
        self.get_logger().info("向挖掘/装载节点发送开始执行挖掘动作信号")

    def goal_callback(self, goal_request):
        if not goal_request.start:
            self.get_logger().warn("收到start=False，拒绝执行")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("收到取消请求")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("开始执行dig.action流程")
        feedback = Dig.Feedback()
        feedback.dummy = 1
        goal_handle.publish_feedback(feedback)

        try:
            # 3. 步骤1：回转90°（扫描物料）
            self.get_logger().info("步骤1：回转90°扫描物料")
            rotate_angle = 90.0
            # rotate_planning_function(plc, rotate_angle)

            # 4. 步骤2：通知启动感知计算 + 等待感知完成
            self.get_logger().info("步骤2：启动感知，扫描物料")
            self.publish_start_signal()
            self.get_logger().info("等待规划系统计算完成...")
            while not self.perception_finished:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = Dig.Result()
                    result.code = Dig.Result.CANCELED
                    result.message = "action canceled"
                    return result
                time.sleep(0.5)
            self.perception_finished = False

            # 5. 步骤3：回转0°（复位到挖掘位置）
            self.get_logger().info("步骤3：回转0°准备复位")
            reset_angle = 0.0
            # rotate_planning_function(plc, reset_angle)

            # 6. 步骤4：复位规划
            self.get_logger().info("步骤4：复位规划")
            # reset_planning_function(plc)
            time.sleep(0.5)

            # 7. 步骤5：挖掘动作
            self.get_logger().info("步骤5：执行挖掘动作")
            # dig_function(plc)
            time.sleep(5.0)
            self.publish_excavation_start()

            feedback.dummy = 2
            goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result = Dig.Result()
            result.code = Dig.Result.SUCCESS
            result.message = "dig sequence finished"
            return result
        except Exception as exc:
            goal_handle.abort()
            result = Dig.Result()
            result.code = Dig.Result.FAILED
            result.message = f"exception: {exc}"
            return result

# =====================PLC通讯相关================================
# 通用函数：写入REAL数组


# ===========================================================
# ====================== 主程序（核心修改环节） ======================
def main():
    rclpy.init()
    node = DigActionServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error(f"操作失败：{str(e)}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("已销毁ROS2动作服务端节点")

if __name__ == "__main__":
    main()
