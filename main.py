from __future__ import annotations
import sys
import math
from typing import Tuple, Union
import numpy as np

def log(*args):
    print(repr(args), file=sys.stderr, flush=True)

def direction(n: int) -> int:
    if n == 0: return 0
    return int(n / abs(n))

def same_sign(x, y) -> bool:
    return x ^ y >= 0

Loc = Tuple[int, int]

KPID = Tuple[float, float, float]
class PID:
    def __init__(self, k: KPID = (1.2, 0.9, 0.3)):
        (self.kp, self.ki, self.kd) = k
        self.iteration = 0

    def set_target(self, target):
        self.target = target
        self.prev_error = target - target
        self.cumulative_moving_average = self.prev_error

    def make_iteration(self, measurement):
        self.iteration += 1
        error = measurement - self.target

        rate_of_change = self.prev_error - error
        self.cumulative_moving_average = (error + self.cumulative_moving_average * self.iteration) / (self.iteration + 1)
        
        proportional = error * self.kp
        integral = self.cumulative_moving_average * self.ki
        derivative = rate_of_change * self.kd

        self.prev_error = error
        return sum((proportional, integral, derivative))

class Planet:
    def __init__(self, n: int):
        self.n = n
        self.points: list[Loc] = []

    def on_surface_complete(self):
        heights = np.array(self.points)[:,1]
        diffs = np.diff(heights)
        plain_at = np.where(diffs == 0)[0][0]
        self.plain_start = self.points[plain_at]
        self.plain_end = self.points[plain_at + 1]

class Rover:
    def __init__(self, planet, x, y, hs, vs, f, r, p):
        self.planet = planet
        self.loc: Loc = (x, y)
        self.hs = hs
        self.vs = vs
        self.fuel = f
        self.rot = r
        self.rotRad = r / 180 * math.pi
        self.power = p

    def dist_to_plain(self) -> Tuple[int, int]:
        diff = np.subtract(self.planet.plain_start, self.loc)
        if diff[0] < 0:
            end_diff = self.planet.plain_end[0] - self.loc[0]
            diff[0] = end_diff if end_diff < 0 else 0
        return tuple(diff)

    def turns_to_plain(self, dist: Tuple[int, int]) -> Tuple[int, int]:
        speed: Tuple[int, int] = (self.hs, self.vs)
        
        # d^s => direction and speed have the same sign
        max_turns = 100
        turns = tuple(
            min(int(d/s), max_turns) 
            if s != 0 and d^s >= 0 
            else 100 
            
            for (d, s) in zip(dist, speed)
        )
        return turns

(max_hs, max_vs) = (20, 40)

# Save the Planet.
# Use less Fossil Fuel.

n = int(input())  # the number of points used to draw the surface of Mars.
planet: Planet = Planet(n)

for i in range(n):
    # land_x: X coordinate of a surface point. (0 to 6999)
    # land_y: Y coordinate of a surface point. By linking all the points together in a sequential fashion, you form the surface of Mars.
    land_x, land_y = [int(j) for j in input().split()]
    planet.points.append((land_x, land_y))

# event to compute planet stats
planet.on_surface_complete()

# game loop
while True:
    # hs: the horizontal speed (in m/s), can be negative.
    # vs: the vertical speed (in m/s), can be negative.
    # f: the quantity of remaining fuel in liters.
    # r: the rotation angle in degrees (-90 to 90).
    # p: the thrust power (0 to 4).
    x, y, hs, vs, f, r, p = [int(i) for i in input().split()]
    me = Rover(planet, x, y, hs, vs, f, r, p)

    # offset rotation
    (dist_x, dist_y) = me.dist_to_plain()
    
    # turns until landing
    (turns_x, turns_y) = me.turns_to_plain((dist_x, dist_y))

    new_rotation = direction(me.rot)
    if abs(dist_x) > 0: 
        if abs(me.hs) < max_hs:
            new_rotation = - direction(dist_x)
    else:
        if abs(me.hs) > max_hs:
            new_rotation = direction(me.hs)
        elif abs(me.rot) > 0:
            new_rotation = 0

    thrust = 3
    if abs(me.vs) > max_vs / 2:
        thrust = 4
            
    
    # start turning if horizontal area not reached in time
    # if turns_x > 50 and abs(me.rot) <= 30:
    #     rotation_offset = -direction(dist_x)

    # # stop turning if area reached in reasonable time  
    # elif turns_x < 20:
    #     if abs(dist_x) > 0:
    #         # mai exista de mers pana la plan, dar avem viteza suficienta > ne intoarcem invers
    #         rotation_offset = direction(dist_x)
    #     elif me.hs > max_hs:
    #         # am ajuns la plan, dar viteza orizontala e prea mare
    #         rotation_offset = direction(me.hs)

            #rot_offset = - direction(me.rot)

    log(turns_x, turns_y)

    # R P. R is the desired rotation angle. P is the desired thrust power.
    print(f"{new_rotation * 20} {thrust}")
