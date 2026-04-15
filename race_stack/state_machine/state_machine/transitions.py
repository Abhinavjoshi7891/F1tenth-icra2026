from __future__ import annotations

from typing import TYPE_CHECKING

from state_machine.state_types import StateType

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine


def dummy_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            if state_machine._low_bat:
                return StateType.LOW_BAT
            else:
                return StateType.GB_TRACK
        case StateType.LOW_BAT:
            return StateType.LOW_BAT
        case default:
            return StateType.GB_TRACK
        
        
def timetrials_transition(state_machine: StateMachine)->str:
    return StateType.GB_TRACK

def head_to_head_transition(state_machine: StateMachine)->str:
    mode = state_machine.params.overtake_mode
    if mode == "predictive_spliner":
        gb_fn   = PSGlobalTrackingTransition
        trail_fn = PSTrailingTransition
        ot_fn   = PSOvertakingTransition
        ftg_fn  = PSFTGOnlyTransition
    else:  # default: spliner
        gb_fn   = SpliniGlobalTrackingTransition
        trail_fn = SpliniTrailingTransition
        ot_fn   = SpliniOvertakingTransition
        ftg_fn  = SpliniFTGOnlyTransition

    match state_machine.state:
        case StateType.GB_TRACK:
            return gb_fn(state_machine)
        case StateType.TRAILING:
            return trail_fn(state_machine)
        case StateType.OVERTAKE:
            return ot_fn(state_machine)
        case StateType.FTGONLY:
            return ftg_fn(state_machine)
        case default:
            raise ValueError(f"Invalid state {state_machine.state}")


def SpliniGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK`"""
    if not state_machine._check_only_ftg_zone:
        if state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector
    splini_avail = state_machine._check_availability_splini_wpts
    o_free = state_machine._check_ofree

    # DIAGNOSTIC: print every gate condition to identify the blocker
    state_machine.get_logger().info(
        f"TRANSITION GATE: gb_free={gb_free} | ot_sector={ot_sector} | splini_avail={splini_avail} | o_free={o_free}",
        throttle_duration_sec=1.0
    )

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg:
            return StateType.FTGONLY
        if not gb_free and not ot_sector:
            return StateType.TRAILING
        elif gb_free and state_machine._check_close_to_raceline:
            return StateType.GB_TRACK
        elif (
            not gb_free
            and ot_sector
            and splini_avail
            and o_free
        ):
            return StateType.OVERTAKE
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    if not state_machine._check_only_ftg_zone:
        in_ot_sector = state_machine._check_ot_sector
        spline_valid = state_machine._check_availability_splini_wpts
        o_free = state_machine._check_ofree

        # If spline is on an obstacle we trail
        if not o_free:
            return StateType.TRAILING
        if in_ot_sector and o_free and spline_valid:
            return StateType.OVERTAKE
        # If spline becomes unvalid while overtaking, we trail
        elif in_ot_sector and not spline_valid and not o_free:
            return StateType.TRAILING
        # go to GB_TRACK if not in ot_sector and the GB is free
        elif not in_ot_sector and state_machine._check_gbfree:
            return StateType.GB_TRACK
        # go to Trailing if not in ot_sector and the GB is not free
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    else:
        if state_machine._check_close_to_raceline and state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.FTGONLY


#######################################
# PREDICTIVE SPLINER (PS) TRANSITIONS #
#######################################
def PSGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK` with predictive_spliner"""
    if not state_machine._check_only_ftg_zone:
        valid_spline    = state_machine._check_availability_splini_wpts
        enemy_in_front  = state_machine._check_enemy_in_front
        ot_sector       = state_machine._check_ot_sector
        gb_free         = state_machine._check_gbfree
        gb_predict_free = state_machine._check_prediction_gbfree
        o_free          = state_machine._check_ofree
        force_trailing  = state_machine._check_force_trailing

        if force_trailing:
            return StateType.TRAILING
        elif valid_spline and o_free and ot_sector:
            return StateType.OVERTAKE
        elif enemy_in_front and gb_free and gb_predict_free and ot_sector and not force_trailing:
            return StateType.GB_TRACK
        elif enemy_in_front:
            return StateType.TRAILING
        else:
            return StateType.GB_TRACK
    else:
        return StateType.FTGONLY


def PSTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING` with predictive_spliner"""
    if not state_machine._check_only_ftg_zone:
        if state_machine._check_ftg:
            return StateType.FTGONLY

        ot_sector        = state_machine._check_ot_sector
        valid_spline     = state_machine._check_availability_splini_wpts
        enemy_in_front   = state_machine._check_enemy_in_front
        gb_free          = state_machine._check_gbfree
        gb_predict_free  = state_machine._check_prediction_gbfree
        o_free           = state_machine._check_ofree
        on_avoidance     = state_machine._check_on_spline
        on_merger        = state_machine._check_on_merger
        force_trailing   = state_machine._check_force_trailing
        emergency_break  = state_machine._check_emergency_break

        state_machine.get_logger().info(
            f"PS_TRAIL: ft={force_trailing} spline={valid_spline} emg={emergency_break} "
            f"o_free={o_free} ot={ot_sector} merger={on_merger} "
            f"enemy={enemy_in_front} gb={gb_free} gbp={gb_predict_free} on_spl={on_avoidance}",
            throttle_duration_sec=1.0
        )

        if force_trailing:
            return StateType.TRAILING
        elif valid_spline and not emergency_break and o_free and ot_sector and not on_merger:
            return StateType.OVERTAKE
        elif not enemy_in_front and on_avoidance and not on_merger:
            return StateType.OVERTAKE
        elif not enemy_in_front:
            return StateType.GB_TRACK
        elif gb_free and gb_predict_free and ot_sector:
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def PSOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE` with predictive_spliner"""
    if not state_machine._check_only_ftg_zone:
        ot_sector       = state_machine._check_ot_sector
        spline_valid    = state_machine._check_availability_splini_wpts
        enemy_in_front  = state_machine._check_enemy_in_front
        emergency_break = state_machine._check_emergency_break
        on_avoidance    = state_machine._check_on_spline
        o_free          = state_machine._check_ofree
        force_trailing  = state_machine._check_force_trailing

        if emergency_break or not ot_sector:
            return StateType.TRAILING
        elif not o_free or force_trailing:
            return StateType.TRAILING
        elif spline_valid and o_free and ot_sector:
            return StateType.OVERTAKE
        elif not spline_valid and not enemy_in_front and on_avoidance:
            return StateType.OVERTAKE
        elif not spline_valid and not enemy_in_front and not on_avoidance:
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def PSFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    else:
        if state_machine._check_close_to_raceline and state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.FTGONLY
