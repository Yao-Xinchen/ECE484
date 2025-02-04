from enum import Enum, auto
import copy
from typing import List


class TLMode(Enum):
    GREEN = auto()
    YELLOW = auto()
    RED = auto()


class VehicleMode(Enum):
    Normal = auto()
    Brake = auto()
    Accel = auto()
    HardBrake = auto()


class State:
    x: float
    y: float
    theta: float
    v: float
    agent_mode: VehicleMode

    def __init__(self, x, y, theta, v, agent_mode: VehicleMode):
        pass


def decisionLogic(ego: State, other: State):
    output = copy.deepcopy(ego)

    # TODO: Edit this part of decision logic

    d_in = 20
    d_out = 15

    brake_dist = 19  # distance needed to brake

    if ego.agent_mode == VehicleMode.Normal:
        if other.signal_mode == TLMode.RED:
            if ego.x > other.x - d_in:
                # exit
                output.agent_mode = VehicleMode.Accel
            if other.x - d_out - brake_dist < ego.x < other.x - d_out:
                # too late for exit
                output.agent_mode = VehicleMode.HardBrake
        if other.signal_mode == TLMode.GREEN:
            output.agent_mode = VehicleMode.Accel
        if other.signal_mode == TLMode.YELLOW:
            if ego.x > other.x - d_in:
                # exit
                output.agent_mode = VehicleMode.Accel
            if other.x - d_in - brake_dist < ego.x < other.x - d_in:
                # too late for exit
                output.agent_mode = VehicleMode.HardBrake

    if ego.agent_mode == VehicleMode.Accel:
        if other.signal_mode == TLMode.RED:
            if other.x - d_out - brake_dist < ego.x < other.x - d_out:
                # too late for exit
                output.agent_mode = VehicleMode.HardBrake
        if other.signal_mode == TLMode.YELLOW:
            if other.x - d_in - brake_dist < ego.x < other.x - d_in:
                # too late for exit
                output.agent_mode = VehicleMode.HardBrake

    if ego.agent_mode == VehicleMode.HardBrake:
        if other.signal_mode == TLMode.GREEN:
            # resume speed
            output.agent_mode = VehicleMode.Accel

    ###########################################

    assert not (other.signal_mode == TLMode.RED
                and (ego.x > other.x - 20 and ego.x < other.x - 15)
                ), "Run Red Light"
    assert not (other.signal_mode == TLMode.RED
                and (ego.x > other.x - 15 and ego.x < other.x) and ego.v < 1
                ), "Stop at Intersection"

    return output
