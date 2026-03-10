from plc_adapter.plc_adapter_node import PlcAdapterNode


def test_step_to_msg_sets_fields():
    msg = PlcAdapterNode._step_to_msg(
        {
            'machine_ready': True,
            'safe_to_walk': True,
            'safe_to_dig': False,
            'walking_requested': True,
            'digging_requested': False,
            'fault_active': False,
            'manual_override': False,
            'fault_code': 12,
            'source': 'unit_test',
        }
    )
    assert msg.machine_ready is True
    assert msg.safe_to_walk is True
    assert msg.safe_to_dig is False
    assert msg.walking_requested is True
    assert msg.digging_requested is False
    assert msg.fault_code == 12
    assert msg.source == 'unit_test'
