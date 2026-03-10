import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from integrated_mission_interfaces.action import DigMission, WalkMission
from integrated_mission_interfaces.msg import PlannerMode as PlannerModeMsg
from integrated_mission_interfaces.msg import PlcSnapshot, SubsystemStatus
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

COMMAND_START = 1
COMMAND_STOP = 2
ERROR_TIMEOUT = 3
ERROR_FAILED = 5
STOPPED_PHASES = {'idle', 'stopped'}
COMPLETED_PHASES = {'completed'}


@dataclass
class MissionRequest:
    mission_id: str
    walk_target: PoseStamped
    dig_target_zone: str
    walk_constraints_json: str
    dig_parameters_json: str
    priority: int


@dataclass
class PendingFailure:
    phase: str
    code: int
    message: str
    retryable: bool


class MissionDispatcherNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_dispatcher')
        self.declare_parameter('walk_action_name', '/mobility/execute')
        self.declare_parameter('dig_action_name', '/excavation/execute')
        self.declare_parameter('walk_status_topic', '/mobility/status')
        self.declare_parameter('dig_status_topic', '/excavation/status')
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
        self.walk_status: Optional[SubsystemStatus] = None
        self.dig_status: Optional[SubsystemStatus] = None
        self.current_mission: Optional[MissionRequest] = None
        self.next_phase: Optional[str] = None
        self.active_phase: Optional[str] = None
        self.phase_command_state: Optional[str] = None
        self.phase_started_at = 0.0
        self.command_request_in_flight = False
        self.pending_command_phase: Optional[str] = None
        self.pending_command_type: Optional[str] = None
        self.pending_failure: Optional[PendingFailure] = None
        self.retry_count = {'walk': 0, 'dig': 0}
        self.last_transition_reason = 'startup'

        walk_name = str(self.get_parameter('walk_action_name').value)
        dig_name = str(self.get_parameter('dig_action_name').value)
        plc_topic = str(self.get_parameter('plc_topic').value)
        walk_status_topic = str(self.get_parameter('walk_status_topic').value)
        dig_status_topic = str(self.get_parameter('dig_status_topic').value)

        self.walk_client = ActionClient(self, WalkMission, walk_name)
        self.dig_client = ActionClient(self, DigMission, dig_name)
        self.create_subscription(PlcSnapshot, plc_topic, self._on_plc_snapshot, 10)
        self.create_subscription(SubsystemStatus, walk_status_topic, self._on_walk_status, 10)
        self.create_subscription(SubsystemStatus, dig_status_topic, self._on_dig_status, 10)
        self.create_service(Trigger, '/mission_dispatcher/start', self._on_start)
        self.create_service(Trigger, '/mission_dispatcher/stop', self._on_stop)
        self.create_service(Trigger, '/mission_dispatcher/recover', self._on_recover)
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
        self.next_phase = None
        self.pending_failure = None
        if self.active_phase is not None and self.phase_command_state != 'stop_requested' and not self.command_request_in_flight:
            self._send_phase_command(self.active_phase, COMMAND_STOP)
        else:
            self._clear_active_phase('stop_service')
        self.current_mission = None
        self._transition(PlannerMode.IDLE, 'stop_service')
        response.success = True
        response.message = 'received'
        self.get_logger().info('Auto mode disabled')
        return response

    def _on_submit_mission(self, request, response):
        if self.current_mission is not None or self.active_phase is not None:
            response.accepted = False
            response.message = 'dispatcher busy'
            return response
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
        self.pending_failure = None
        self.retry_count = {'walk': 0, 'dig': 0}
        response.accepted = True
        response.message = f'{mission_id} queued'
        self.get_logger().info(f'Mission queued: {mission_id}')
        return response

    def _on_recover(self, request, response):
        del request
        if self.mode != PlannerMode.FAULT:
            response.success = True
            response.message = 'dispatcher not in fault'
            return response
        if self.plc_snapshot and (self.plc_snapshot.fault_active or self.plc_snapshot.manual_override):
            response.success = False
            response.message = 'plc fault/manual override still active'
            return response
        self.pending_failure = None
        self.current_mission = None
        self.next_phase = None
        self.retry_count = {'walk': 0, 'dig': 0}
        self._clear_active_phase('recover_service')
        self._transition(PlannerMode.IDLE, 'recover_service')
        response.success = True
        response.message = 'dispatcher recovered to idle'
        return response

    def _on_plc_snapshot(self, msg: PlcSnapshot) -> None:
        self.plc_snapshot = msg

    def _on_walk_status(self, msg: SubsystemStatus) -> None:
        self.walk_status = msg

    def _on_dig_status(self, msg: SubsystemStatus) -> None:
        self.dig_status = msg

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

        context = DispatchContext(
            mode=self.mode,
            auto_enabled=self.auto_enabled,
            mission_present=self.current_mission is not None,
            next_phase=self.next_phase,
            active_goal=self.active_phase is not None or self.command_request_in_flight,
        )
        decision = self.state_machine.decide(context, plc)
        if decision.mode == PlannerMode.FAULT:
            if decision.mode != self.mode:
                self._transition(decision.mode, decision.reason)
            if self.active_phase is not None and self.phase_command_state != 'stop_requested' and not self.command_request_in_flight:
                self._send_phase_command(self.active_phase, COMMAND_STOP)
            return

        if self._execution_timed_out():
            self._stage_failure(self.active_phase or 'unknown', ERROR_TIMEOUT, 'execution timeout', retryable=True)

        if self.active_phase is not None:
            self._drive_active_phase()
            return

        if self.pending_failure is not None and not self.command_request_in_flight:
            self._finalize_failure()
            return

        self._apply_decision(decision)

    def _apply_decision(self, decision: Decision) -> None:
        if decision.mode != self.mode:
            self._transition(decision.mode, decision.reason)
        if self.command_request_in_flight:
            return
        if decision.command == 'dispatch_walk':
            self._send_phase_command('walk', COMMAND_START)
        elif decision.command == 'dispatch_dig':
            self._send_phase_command('dig', COMMAND_START)

    def _drive_active_phase(self) -> None:
        status = self._current_phase_status(self.active_phase)
        if status is None or not self._status_matches_active_phase(status):
            return
        if status.error:
            self._stage_failure(self.active_phase, status.error_code or ERROR_FAILED, status.detail, retryable=True)
            return
        if self.phase_command_state == 'started' and status.phase in COMPLETED_PHASES and not self.command_request_in_flight:
            self._send_phase_command(self.active_phase, COMMAND_STOP)
            return
        if self.phase_command_state == 'stop_requested' and status.phase in STOPPED_PHASES and not status.active:
            if self.pending_failure is not None:
                self._clear_active_phase('failure_cleanup')
                self._finalize_failure()
                return
            self._complete_phase(self.active_phase)

    def _send_phase_command(self, phase: str, command: int) -> None:
        if self.current_mission is None and command == COMMAND_START:
            return
        if self.command_request_in_flight:
            return

        if phase == 'walk':
            client = self.walk_client
            goal = WalkMission.Goal()
            goal.command = command
            goal.mission_id = self.current_mission.mission_id if self.current_mission else ''
            if self.current_mission is not None:
                goal.target_pose = self.current_mission.walk_target
                goal.constraints_json = self.current_mission.walk_constraints_json
                goal.priority = self.current_mission.priority
            goal.timeout_sec = float(self.walk_timeout_sec)
        elif phase == 'dig':
            client = self.dig_client
            goal = DigMission.Goal()
            goal.command = command
            goal.mission_id = self.current_mission.mission_id if self.current_mission else ''
            if self.current_mission is not None:
                goal.target_zone = self.current_mission.dig_target_zone
                goal.process_parameters_json = self.current_mission.dig_parameters_json
                goal.safety_boundary_json = '{}'
                goal.priority = self.current_mission.priority
            goal.timeout_sec = float(self.dig_timeout_sec)
        else:
            raise ValueError(f'unknown phase: {phase}')

        if not client.wait_for_server(timeout_sec=0.5):
            if command == COMMAND_STOP:
                self.get_logger().error(f'{phase} stop action server unavailable')
                self._clear_active_phase(f'{phase}_stop_server_unavailable')
                self._transition(PlannerMode.FAULT, f'{phase}_stop_unavailable')
            else:
                self._stage_failure(phase, ERROR_FAILED, f'{phase} action server unavailable', retryable=True)
            return

        self.command_request_in_flight = True
        self.pending_command_phase = phase
        self.pending_command_type = 'start' if command == COMMAND_START else 'stop'
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda fut, phase_name=phase, command_type=self.pending_command_type: self._on_command_goal_response(phase_name, command_type, fut)
        )
        self.get_logger().info(f'Sent {self.pending_command_type} command to {phase}')

    def _on_command_goal_response(self, phase: str, command_type: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.command_request_in_flight = False
            if command_type == 'stop':
                self.get_logger().error(f'{phase} stop goal response failed: {exc}')
                self._clear_active_phase(f'{phase}_stop_goal_response_failed')
                self._transition(PlannerMode.FAULT, f'{phase}_stop_goal_response_failed')
            else:
                self._stage_failure(phase, ERROR_FAILED, f'{phase} {command_type} goal response failed: {exc}', retryable=True)
            return
        if not goal_handle.accepted:
            self.command_request_in_flight = False
            if command_type == 'stop':
                self.get_logger().error(f'{phase} stop goal rejected')
                self._clear_active_phase(f'{phase}_stop_goal_rejected')
                self._transition(PlannerMode.FAULT, f'{phase}_stop_goal_rejected')
            else:
                self._stage_failure(phase, ERROR_FAILED, f'{phase} {command_type} goal rejected', retryable=False)
            return
        goal_handle.get_result_async().add_done_callback(
            lambda fut, phase_name=phase, command_name=command_type: self._on_command_result(phase_name, command_name, fut)
        )

    def _on_command_result(self, phase: str, command_type: str, future) -> None:
        self.command_request_in_flight = False
        self.pending_command_phase = None
        self.pending_command_type = None
        try:
            wrapped = future.result()
        except Exception as exc:
            if command_type == 'stop':
                self.get_logger().error(f'{phase} stop result failed: {exc}')
                self._clear_active_phase(f'{phase}_stop_result_failed')
                self._transition(PlannerMode.FAULT, f'{phase}_stop_result_failed')
            else:
                self._stage_failure(phase, ERROR_FAILED, f'{phase} {command_type} result failed: {exc}', retryable=True)
            return
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.accepted:
            if command_type == 'stop':
                self.get_logger().error(f'{phase} stop failed: {result.message}')
                self._clear_active_phase(f'{phase}_stop_failed')
                self._transition(PlannerMode.FAULT, f'{phase}_stop_failed')
            else:
                self._stage_failure(phase, result.error_code or ERROR_FAILED, result.message or f'{phase} {command_type} failed', retryable=True)
            return
        if command_type == 'start':
            self.active_phase = phase
            self.phase_command_state = 'started'
            self.phase_started_at = time.monotonic()
            self._transition(DispatcherStateMachine.active_mode_for_phase(phase), f'{phase}_start_ack')
            return
        self.phase_command_state = 'stop_requested'
        self.get_logger().info(f'{phase} stop acknowledged')
        status = self._current_phase_status(phase)
        if status is not None and self._status_matches_active_phase(status) and status.phase in STOPPED_PHASES and not status.active:
            if self.pending_failure is not None:
                self._clear_active_phase('failure_cleanup')
                self._finalize_failure()
            else:
                self._complete_phase(phase)

    def _execution_timed_out(self) -> bool:
        if self.active_phase is None or self.phase_command_state != 'started':
            return False
        timeout = self.walk_timeout_sec if self.active_phase == 'walk' else self.dig_timeout_sec
        return (time.monotonic() - self.phase_started_at) > timeout

    def _stage_failure(self, phase: str, code: int, message: str, retryable: bool) -> None:
        if self.pending_failure is None:
            self.pending_failure = PendingFailure(phase=phase, code=code, message=message, retryable=retryable)
            self.get_logger().error(
                f'Phase {phase} failed: code={code} message={message} retryable={retryable}'
            )
        if self.active_phase is not None and self.phase_command_state != 'stop_requested' and not self.command_request_in_flight:
            self._send_phase_command(self.active_phase, COMMAND_STOP)
        elif self.active_phase is None and not self.command_request_in_flight:
            self._finalize_failure()

    def _finalize_failure(self) -> None:
        if self.pending_failure is None:
            return
        failure = self.pending_failure
        self.pending_failure = None
        if failure.phase in self.retry_count and failure.retryable and self.retry_count[failure.phase] < self.max_retry_count:
            self.retry_count[failure.phase] += 1
            self.next_phase = failure.phase
            self._transition(PlannerMode.TRANSITION, f'retry_{failure.phase}_{self.retry_count[failure.phase]}')
            return
        self._transition(PlannerMode.FAULT, f'{failure.phase}_failed:{failure.code}')

    def _complete_phase(self, phase: str) -> None:
        mission_id = self.current_mission.mission_id if self.current_mission else 'unknown'
        self._clear_active_phase(f'{phase}_complete')
        if phase == 'walk':
            self.next_phase = 'dig'
            self._transition(PlannerMode.TRANSITION, 'walk_complete')
            return
        if phase == 'dig':
            self.current_mission = None
            self.next_phase = None
            self.retry_count = {'walk': 0, 'dig': 0}
            self._transition(PlannerMode.IDLE, f'mission_complete:{mission_id}')
            return
        raise ValueError(f'unknown phase: {phase}')

    def _clear_active_phase(self, reason: str) -> None:
        self.active_phase = None
        self.phase_command_state = None
        self.phase_started_at = 0.0
        self.command_request_in_flight = False
        self.pending_command_phase = None
        self.pending_command_type = None
        self.get_logger().info(f'Active phase cleared: {reason}')

    def _current_phase_status(self, phase: Optional[str]) -> Optional[SubsystemStatus]:
        if phase == 'walk':
            return self.walk_status
        if phase == 'dig':
            return self.dig_status
        return None

    def _status_matches_active_phase(self, status: SubsystemStatus) -> bool:
        if self.current_mission is None:
            return True
        return status.mission_id in {'', self.current_mission.mission_id}

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
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
