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

Loc = Tuple[int, int]

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

    rotation_offset = 0
    if abs(me.rot) <= 30:
        rotation_offset = -direction(dist_x)

    if dist_x == 0:
        rotation_offset = - direction(me.rot)

    # R P. R is the desired rotation angle. P is the desired thrust power.
    print(f"{me.rot + rotation_offset * 3} 3")
