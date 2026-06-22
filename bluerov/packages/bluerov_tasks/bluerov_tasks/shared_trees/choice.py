import py_trees
from std_srvs.srv import Trigger

from mission_planner_2.common.core import checked_service

CHOICE_KEY = "/global/choice_is_fish"
GET_CHOICE_SERVICE = "/bluerov/choice/get_is_fish"


def create_get_choice_root(
    choice_key: str = CHOICE_KEY,
    service_name: str = GET_CHOICE_SERVICE,
) -> py_trees.behaviour.Behaviour:
    """Populate choice_key from the choice server, defaulting to fish."""
    query_choice = checked_service.FromConstant(
        name="Query choice server",
        service_type=Trigger,
        service_name=service_name,
        service_request=Trigger.Request(),
        key_response=choice_key,
        wait_for_server_timeout_sec=0.0,
        # Accept any response; we only need it on the blackboard, not a value.
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
