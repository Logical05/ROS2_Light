#!/usr/bin/env python3
from math import atan2, asin, pi
from dataclasses import dataclass
from typing import TypedDict


@dataclass
class Euler:
    """ Custom Euler angle data type """
    roll : float
    pitch: float
    yaw  : float

class Info(TypedDict):
    """ Custom data type for transfer between UI and Node """
    pose    : list[float, float]
    euler   : Euler
    velocity: float
    button  : list

def normalizeRads(rads: float) -> float:
    """ Corrects the given angle in radians to be in a range from 0 to 2π """
    rads = rads % (2 * pi)
    if rads == (2 * pi): rads = 0
    return rads

def quat2Euler(x: float, y: float, z: float, w: float) -> Euler:
    """ Convert Quaternion to Euler """
    x2 = pow(x, 2)
    y2 = pow(y, 2)
    z2 = pow(z, 2)
    w2 = pow(w, 2)
    roll  = atan2(2.0 * (y * z + x * w),  (- x2 - y2 + z2 + w2))
    pitch = asin(-2.0 * (x * z - y * w) / (  x2 + y2 + z2 + w2))
    yaw   = atan2(2.0 * (x * y + z * w),  (  x2 - y2 - z2 + w2))
    euler = Euler(roll, pitch, yaw)
    return euler