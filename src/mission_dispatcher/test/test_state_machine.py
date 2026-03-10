from mission_dispatcher.state_machine import DispatchContext, DispatcherStateMachine, PlannerMode, PlcState


def test_walk_selected_when_walk_ready():
    sm = DispatcherStateMachine()
    decision = sm.decide(
        DispatchContext(mode=PlannerMode.IDLE, auto_enabled=True, mission_present=True, next_phase='walk', active_goal=False),
        PlcState(machine_ready=True, safe_to_walk=True),
    )
    assert decision.mode == PlannerMode.WALK_PREP
    assert decision.command == 'dispatch_walk'


def test_transition_when_dig_not_ready():
    sm = DispatcherStateMachine()
    decision = sm.decide(
        DispatchContext(mode=PlannerMode.TRANSITION, auto_enabled=True, mission_present=True, next_phase='dig', active_goal=False),
        PlcState(machine_ready=True, safe_to_walk=False, safe_to_dig=False),
    )
    assert decision.mode == PlannerMode.TRANSITION
    assert decision.command is None


def test_fault_when_plc_fault_active():
    sm = DispatcherStateMachine()
    decision = sm.decide(
        DispatchContext(mode=PlannerMode.WALKING, auto_enabled=True, mission_present=True, next_phase='dig', active_goal=True),
        PlcState(machine_ready=True, safe_to_walk=True, safe_to_dig=True, fault_active=True),
    )
    assert decision.mode == PlannerMode.FAULT


def test_fault_is_latched_until_external_reset():
    sm = DispatcherStateMachine()
    decision = sm.decide(
        DispatchContext(mode=PlannerMode.FAULT, auto_enabled=True, mission_present=True, next_phase='walk', active_goal=False),
        PlcState(machine_ready=True, safe_to_walk=True, safe_to_dig=True),
    )
    assert decision.mode == PlannerMode.FAULT
    assert decision.reason == 'fault_latched'
