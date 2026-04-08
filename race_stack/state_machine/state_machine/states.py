from __future__ import annotations

from typing import List, TYPE_CHECKING
from f110_msgs.msg import Wpnt
from state_machine.state_types import StateType
if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine

def DefaultStateLogic(state_machine: StateMachine) -> List[Wpnt]:
    """
    This is a global state that incorporates the other states
    """
    match state_machine.state:
        case StateType.GB_TRACK:
            return GlobalTracking(state_machine)
        case StateType.TRAILING:
            return Trailing(state_machine)
        case StateType.OVERTAKE:
            return Overtaking(state_machine)
        case StateType.FTGONLY:
            return FTGOnly(state_machine)
        case _:
            raise NotImplementedError(f"State {state_machine.state} not recognized")

"""
Here we define the behaviour in the different states.
Every function should be fairly concise, and output an array of f110_msgs.Wpnt
"""
def GlobalTracking(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Trailing(state_machine: StateMachine) -> List[Wpnt]:
    # This allows us to trail on the last valid spline if necessary
    if state_machine.last_valid_avoidance_wpnts is not None:
        splini_wpts = state_machine.get_splini_wpts()
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        wpts = [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]
    else:
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        wpts = [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

    # SAFETY & PUSH: Maintain gap and close distance
    if len(state_machine.obstacles) > 0:
        # Find closest opponent (should be the one we are trailing)
        closest_obs = min(state_machine.obstacles, key=lambda obs: (obs.s_center - state_machine.cur_s) % state_machine.track_length)
        gap = (closest_obs.s_center - state_machine.cur_s) % state_machine.track_length
        
        if gap > 2.0:
            # Far away: Go slightly FASTER than opponent to close distance
            target_v = closest_obs.vs + 0.3
        else:
            # Very close: Match opponent speed exactly (minus tiny safety buffer)
            target_v = max(0.0, closest_obs.vs - 0.05)
        
        # Apply the cap to all local waypoints
        for wpnt in wpts:
            wpnt.vx_mps = min(wpnt.vx_mps, target_v)
            
    return wpts

def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
    splini_wpts = state_machine.get_splini_wpts()
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def FTGOnly(state_machine: StateMachine) -> List[Wpnt]:
    """No waypoints are generated in this follow the gap only state, all the 
    control inputs are generated in the control node.
    """
    return []