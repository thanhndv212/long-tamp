import json
import xml.etree.ElementTree as ET

from long_tamp.tasks.task_planning.host import create_fake_session


def test_fake_host_session_exposes_json_contract_and_compiled_tree():
    session = create_fake_session("{}")

    xml = session.get_behavior_tree_xml()
    root = ET.fromstring(xml)
    assert root.attrib["BTCPP_format"] == "4"

    step_id = "move-home"
    assert json.loads(session.check_precondition(step_id))["ready"]
    assert json.loads(session.execute_step(step_id))["status"] == "success"
    assert json.loads(session.is_step_complete(step_id))["complete"]


def test_a_host_session_gives_its_plan_document():
    """#25: a C++ host starts its event stream with the plan (for viewers)."""
    import json as _json

    from long_tamp.tasks.task_planning.host import create_fake_session

    session = create_fake_session('{"shape": "composite"}')
    assert _json.loads(session.get_plan_document()) == session.plan.document
