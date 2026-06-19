"""Fish/shark choice resolution for the bin and torpedo missions.

The bin/torpedo trees branch on a `/global/choice_is_fish` blackboard key that
holds a ``std_srvs/Trigger.Response`` (read via its ``.success`` field — see
``bluerov_tasks/bins/choice_selector.py`` and
``bluerov_tasks/torpedo/move_and_shoot_seq.py``). This module builds the subtree
that populates that key from the choice server (``/bluerov/choice/get_is_fish``,
served by ``choice_server_node.py``), falling back to fish if the server isn't
reachable so the mission still runs standalone.

Pick fish or shark at runtime with:
    ros2 service call /bluerov/choice/set_is_fish std_srvs/srv/SetBool "{data: true}"   # fish
    ros2 service call /bluerov/choice/set_is_fish std_srvs/srv/SetBool "{data: false}"  # shark
"""

import py_trees
from std_srvs.srv import Trigger

from mission_planner_2.common.core import checked_service

CHOICE_KEY = "/global/choice_is_fish"
GET_CHOICE_SERVICE = "/bluerov/choice/get_is_fish"


def create_get_choice_root(
    choice_key: str = CHOICE_KEY,
    service_name: str = GET_CHOICE_SERVICE,
) -> py_trees.behaviour.Behaviour:
    """Populate ``choice_key`` from the choice server, defaulting to fish.

    Returns a memory Selector that first queries ``service_name`` and writes the
    ``Trigger.Response`` to ``choice_key``; if the server is unreachable it falls
    back to a hardcoded fish choice so downstream ``.success`` reads never
    KeyError. ``wait_for_server_timeout_sec=0.0`` keeps tree setup from blocking
    when the choice server isn't running.
    """
    query_choice = checked_service.FromConstant(
        name="Query choice server",
        service_type=Trigger,
        service_name=service_name,
        service_request=Trigger.Request(),
        key_response=choice_key,
        wait_for_server_timeout_sec=0.0,
        # Accept fish (success=True) OR shark (success=False); we only need a
        # response to land on the blackboard, not a particular value.
        check_func=lambda x: x is not None,
    )

    default_fish = py_trees.behaviours.SetBlackboardVariable(
        name="Default choice to fish",
        variable_name=choice_key,
        variable_value=Trigger.Response(success=True, message="default fish"),
        overwrite=True,
    )

    return py_trees.composites.Selector(
        name="Resolve fish/shark choice",
        memory=True,
        children=[query_choice, default_fish],
    )
