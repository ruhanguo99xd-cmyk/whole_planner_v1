from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


class PlannerMode(IntEnum):
    IDLE = 0
    WALK_PREP = 1
    WALKING = 2
    DIG_PREP = 3
    DIGGING = 4
    TRANSITION = 5
    FAULT = 255


@dataclass
class PlcState:
    machine_ready: bool = False
    safe_to_walk: bool = False
    safe_to_dig: bool = False
    fault_active: bool = False
    manual_override: bool = False


@dataclass
class DispatchContext:
    mode: PlannerMode = PlannerMode.IDLE
    auto_enabled: bool = False
    mission_present: bool = False
    next_phase: Optional[str] = None
    active_goal: bool = False


@dataclass
class Decision:
    mode: PlannerMode
    reason: str
    command: Optional[str] = None


class DispatcherStateMachine:
    def decide(self, context: DispatchContext, plc: PlcState) -> Decision:
        if plc.fault_active or plc.manual_override:
            return Decision(PlannerMode.FAULT, 'plc_fault')
        if context.mode == PlannerMode.FAULT:
            return Decision(PlannerMode.FAULT, 'fault_latched')
        if not context.auto_enabled:
            return Decision(context.mode, 'auto_disabled')
        if not context.mission_present:
            return Decision(PlannerMode.IDLE, 'no_mission')
        if context.active_goal:
            return Decision(context.mode, 'goal_active')
        if context.next_phase == 'walk':
            if plc.machine_ready and plc.safe_to_walk:
                return Decision(PlannerMode.WALK_PREP, 'walk_ready', 'dispatch_walk')
            return Decision(PlannerMode.TRANSITION, 'waiting_walk_ready')
        if context.next_phase == 'dig':
            if plc.machine_ready and plc.safe_to_dig:
                return Decision(PlannerMode.DIG_PREP, 'dig_ready', 'dispatch_dig')
            return Decision(PlannerMode.TRANSITION, 'waiting_dig_ready')
        return Decision(PlannerMode.IDLE, 'mission_complete')

    @staticmethod
    def active_mode_for_phase(phase: str) -> PlannerMode:
        if phase == 'walk':
            return PlannerMode.WALKING
        if phase == 'dig':
            return PlannerMode.DIGGING
        raise ValueError(f'unknown phase: {phase}')
