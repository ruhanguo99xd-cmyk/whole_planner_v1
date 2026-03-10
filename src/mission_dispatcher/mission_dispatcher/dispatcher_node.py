import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from integrated_mission_interfaces.action import DigMission, WalkMission
from integrated_mission_interfaces.msg import PlannerMode as PlannerModeMsg
from integrated_mission_interfaces.msg import PlcSnapshot
from integrated_mission_interfaces.srv import SubmitMission
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from mission_dispatcher.state_machine import (
    Decision,
    DispatchContext,
    DispatcherStateMachine,
    PlannerMode,
    PlcState,
)


ERROR_TIMEOUT = 3
ERROR_FAILED = 5


@dataclass
class MissionRequest:
    mission_id: str
    walk_target: PoseStamped
    dig_target_zone: str
    walk_constraints_json: str
    dig_parameters_json: str
    priority: int


class MissionDispatcherNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_dispatcher')
        self.declare_parameter('walk_action_name', '/mobility/execute')
        self.declare_parameter('dig_action_name', '/excavation/execute')
        self.declare_parameter('plc_topic', '/plc/status')
        self.declare_parameter('loop_period_sec', 0.1)
        self.declare_parameter('walk_timeout_sec', 30.0)
        self.declare_parameter('dig_timeout_sec', 60.0)
        self.declare_parameter('max_retry_count', 1)
        self.declare_parameter('auto_start', True)

        self.walk_timeout_sec = float(self.get_parameter('walk_timeout_sec').value)
        self.dig_timeout_sec = float(self.get_parameter('dig_timeout_sec').value)
        self.max_retry_count = int(self.get_parameter('max_retry_count').value)
        self.auto_enabled = bool(self.get_parameter('auto_start').value)

        self.state_machine = DispatcherStateMachine()
        self.mode = PlannerMode.IDLE
        self.mode_pub = self.create_publisher(PlannerModeMsg, '/mission_dispatcher/mode', 10)
        self.plc_snapshot: Optional[PlcSnapshot] = None
        self.current_mission: Optional[MissionRequest] = None
        self.next_phase: Optional[str] = None
        self.active_phase: Optional[str] = None
        self.active_goal_handle = None
        self.goal_request_in_flight = False
        self.active_goal_started_at = 0.0
        self.active_goal_timeout = 0.0
        self.retry_count = {'walk': 0, 'dig': 0}
        self.last_transition_reason = 'startup'

        walk_name = str(self.get_parameter('walk_action_name').value)
        dig_name = str(self.get_parameter('dig_action_name').value)
        plc_topic = str(self.get_parameter('plc_topic').value)

        self.walk_client = ActionClient(self, WalkMission, walk_name)
        self.dig_client = ActionClient(self, DigMission, dig_name)
        self.create_subscription(PlcSnapshot, plc_topic, self._on_plc_snapshot, 10)
        self.create_service(Trigger, '/mission_dispatcher/start', self._on_start)
        self.create_service(Trigger, '/mission_dispatcher/stop', self._on_stop)
        self.create_service(SubmitMission, '/mission_dispatcher/submit_mission', self._on_submit_mission)

        loop_period = float(self.get_parameter('loop_period_sec').value)
        self.timer = self.create_timer(loop_period, self._on_loop)
        self._publish_mode('startup')
        self.get_logger().info(
            f'mission_dispatcher started: walk_action={walk_name} '
            f'dig_action={dig_name} auto={self.auto_enabled}'
        )

    def _on_start(self, request, response):
        del request
        self.auto_enabled = True
        response.success = True
        response.message = 'dispatcher auto mode enabled'
        self.get_logger().info('Auto mode enabled')
        return response

    def _on_stop(self, request, response):
        del request
        self.auto_enabled = False
        self._cancel_active_goal('stop_service')
        self.current_mission = None
        self.next_phase = None
        self.active_phase = None
        self._transition(PlannerMode.IDLE, 'stop_service')
        response.success = True
        response.message = 'dispatcher stopped'
        self.get_logger().info('Auto mode disabled')
        return response

    def _on_submit_mission(self, request, response):
        mission_id = request.mission_id or f'mission-{int(time.time())}'
        self.current_mission = MissionRequest(
            mission_id=mission_id,
            walk_target=request.walk_target,
            dig_target_zone=request.dig_target_zone,
            walk_constraints_json=request.walk_constraints_json,
            dig_parameters_json=request.dig_parameters_json,
            priority=int(request.priority),
        )
        self.next_phase = 'walk'
        self.retry_count = {'walk': 0, 'dig': 0}
        response.accepted = True
        response.message = f'{mission_id} queued'
        self.get_logger().info(f'Mission queued: {mission_id}')
        return response

    def _on_plc_snapshot(self, msg: PlcSnapshot) -> None:
        self.plc_snapshot = msg

    def _on_loop(self) -> None:
        if self.plc_snapshot is None:
            return
        plc = PlcState(
            machine_ready=self.plc_snapshot.machine_ready,
            safe_to_walk=self.plc_snapshot.safe_to_walk,
            safe_to_dig=self.plc_snapshot.safe_to_dig,
            fault_active=self.plc_snapshot.fault_active,
            manual_override=self.plc_snapshot.manual_override,
        )
        if self._active_goal_timed_out():
            self._handle_action_failure(self.active_phase or 'unknown', ERROR_TIMEOUT, 'action timeout', retryable=True)
            return

        context = DispatchContext(
            mode=self.mode,
            auto_enabled=self.auto_enabled,
            mission_present=self.current_mission is not None,
            next_phase=self.next_phase,
            active_goal=self.active_goal_handle is not None or self.goal_request_in_flight,
        )
        decision = self.state_machine.decide(context, plc)
        self._apply_decision(decision)

    def _apply_decision(self, decision: Decision) -> None:
        if decision.mode != self.mode:
            self._transition(decision.mode, decision.reason)
        if decision.command == 'dispatch_walk':
            self._send_walk_goal()
        elif decision.command == 'dispatch_dig':
            self._send_dig_goal()

    def _send_walk_goal(self) -> None:
        if self.current_mission is None:
            return
        if self.goal_request_in_flight:
            return
        if not self.walk_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warning('Walk action server unavailable')
            return
        goal = WalkMission.Goal()
        goal.mission_id = self.current_mission.mission_id
        goal.target_pose = self.current_mission.walk_target
        goal.constraints_json = self.current_mission.walk_constraints_json
        goal.priority = self.current_mission.priority
        goal.timeout_sec = float(self.walk_timeout_sec)
        self.goal_request_in_flight = True
        future = self.walk_client.send_goal_async(goal, feedback_callback=self._on_walk_feedback)
        future.add_done_callback(self._on_walk_goal_response)

    def _send_dig_goal(self) -> None:
        if self.current_mission is None:
            return
        if self.goal_request_in_flight:
            return
        if not self.dig_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warning('Dig action server unavailable')
            return
        goal = DigMission.Goal()
        goal.mission_id = self.current_mission.mission_id
        goal.target_zone = self.current_mission.dig_target_zone
        goal.process_parameters_json = self.current_mission.dig_parameters_json
        goal.safety_boundary_json = '{}'
        goal.priority = self.current_mission.priority
        goal.timeout_sec = float(self.dig_timeout_sec)
        self.goal_request_in_flight = True
        future = self.dig_client.send_goal_async(goal, feedback_callback=self._on_dig_feedback)
        future.add_done_callback(self._on_dig_goal_response)

    def _on_walk_goal_response(self, future) -> None:
        self.goal_request_in_flight = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._handle_action_failure('walk', ERROR_FAILED, 'walk goal rejected', retryable=False)
            return
        self.active_goal_handle = goal_handle
        self.active_phase = 'walk'
        self.active_goal_started_at = time.monotonic()
        self.active_goal_timeout = self.walk_timeout_sec
        self._transition(PlannerMode.WALKING, 'walk_goal_accepted')
        goal_handle.get_result_async().add_done_callback(self._on_walk_result)

    def _on_dig_goal_response(self, future) -> None:
        self.goal_request_in_flight = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._handle_action_failure('dig', ERROR_FAILED, 'dig goal rejected', retryable=False)
            return
        self.active_goal_handle = goal_handle
        self.active_phase = 'dig'
        self.active_goal_started_at = time.monotonic()
        self.active_goal_timeout = self.dig_timeout_sec
        self._transition(PlannerMode.DIGGING, 'dig_goal_accepted')
        goal_handle.get_result_async().add_done_callback(self._on_dig_result)

    def _on_walk_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Walk feedback: {feedback.phase} {feedback.progress:.2f}')

    def _on_dig_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Dig feedback: {feedback.phase} {feedback.progress:.2f}')

    def _on_walk_result(self, future) -> None:
        wrapped = future.result()
        result = wrapped.result
        self.active_goal_handle = None
        self.active_phase = None
        if wrapped.status == GoalStatus.STATUS_ABORTED:
            self._handle_action_failure('walk', ERROR_FAILED, 'walk action aborted', retryable=True)
            return
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            self._handle_action_failure('walk', ERROR_FAILED, 'walk action canceled', retryable=True)
            return
        if result.success:
            self.next_phase = 'dig'
            self._transition(PlannerMode.TRANSITION, 'walk_complete')
            return
        self._handle_action_failure('walk', result.error_code or ERROR_FAILED, result.message, retryable=result.retryable)

    def _on_dig_result(self, future) -> None:
        wrapped = future.result()
        result = wrapped.result
        self.active_goal_handle = None
        self.active_phase = None
        if wrapped.status == GoalStatus.STATUS_ABORTED:
            self._handle_action_failure('dig', ERROR_FAILED, 'dig action aborted', retryable=True)
            return
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            self._handle_action_failure('dig', ERROR_FAILED, 'dig action canceled', retryable=True)
            return
        if result.success:
            mission_id = self.current_mission.mission_id if self.current_mission else 'unknown'
            self.current_mission = None
            self.next_phase = None
            self.retry_count = {'walk': 0, 'dig': 0}
            self._transition(PlannerMode.IDLE, f'mission_complete:{mission_id}')
            return
        self._handle_action_failure('dig', result.error_code or ERROR_FAILED, result.message, retryable=result.retryable)

    def _handle_action_failure(self, phase: str, code: int, message: str, retryable: bool) -> None:
        self.get_logger().error(
            f'Phase {phase} failed: code={code} message={message} retryable={retryable}'
        )
        self._cancel_active_goal('failure')
        if phase in self.retry_count and retryable and self.retry_count[phase] < self.max_retry_count:
            self.retry_count[phase] += 1
            self.next_phase = phase
            self._transition(PlannerMode.TRANSITION, f'retry_{phase}_{self.retry_count[phase]}')
            return
        self._transition(PlannerMode.FAULT, f'{phase}_failed:{code}')

    def _cancel_active_goal(self, reason: str) -> None:
        if self.active_goal_handle is not None:
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover
                self.get_logger().warning(f'Cancel goal failed: {exc}')
        self.active_goal_handle = None
        self.active_phase = None
        self.active_goal_started_at = 0.0
        self.active_goal_timeout = 0.0
        self.goal_request_in_flight = False
        self.get_logger().info(f'Active goal cleared: {reason}')

    def _active_goal_timed_out(self) -> bool:
        if self.active_goal_handle is None:
            return False
        return (time.monotonic() - self.active_goal_started_at) > self.active_goal_timeout

    def _transition(self, mode: PlannerMode, reason: str) -> None:
        if mode != self.mode:
            self.get_logger().info(f'State transition: {self.mode.name} -> {mode.name} ({reason})')
        self.mode = mode
        self.last_transition_reason = reason
        self._publish_mode(reason)

    def _publish_mode(self, reason: str) -> None:
        msg = PlannerModeMsg()
        msg.mode = int(self.mode)
        msg.mission_id = self.current_mission.mission_id if self.current_mission else ''
        msg.reason = reason
        msg.stamp = self.get_clock().now().to_msg()
        self.mode_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MissionDispatcherNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
