import sys
import math
from typing import List, Tuple
import numpy as np

def log(*args):
    print(*args, file=sys.stderr, flush=True)

# Surface
surface: List[Tuple[int, int]] = []
for i in range(int(input())):
    pos = [int(j) for j in input().split()]
    surface.append(tuple(pos))

# Landing Zone
lz_x0, lz_x1, lz_y = -1, -1, -1
last_pos = -1, -1
for pos in surface:
    if pos[1] == last_pos[1]:
        lz_x1, lz_y = pos
        lz_x0 = last_pos[0]
        break
    last_pos = pos
lz_i = surface.index((lz_x0, lz_y))

# Flatten surface
flat_surface = []
get_y = lambda point: point[1]

# Flat to LZ
i = 0
while i < lz_i:
    highest = max(surface[i : lz_i+1], key = get_y)
    flat_surface.append(highest)
    i = surface.index(highest) + 1

# Flat after LZ
flat_surface_after_lz = []
i = len(surface)
while i > lz_i + 2:
    highest = max(surface[lz_i+2 : i], key = get_y)
    flat_surface_after_lz.append(highest)
    i = surface.index(highest)
flat_surface_after_lz.reverse()

# Add start, LZ and end points
if flat_surface[0][0] > 0:
    flat_surface = [(0, flat_surface[0][1]), *flat_surface]
flat_surface = [*flat_surface, (lz_x0, lz_y), (lz_x1, lz_y), *flat_surface_after_lz]
if flat_surface[-1][0] < 6999:
    flat_surface = [*flat_surface, (6999, flat_surface[-1][1])]

# parameters
vx_max, vy_max = 20, 40
cruise_vx = vx_max * 2

# game loop
while True:
    x, y, vx, vy, fuel, rotate, power = [int(i) for i in input().split()]

    # above LZ?
    dx = 0
    if x < lz_x0: dx = 1
    if x > lz_x1: dx = -1

    # horizontal acceleration
    theta_coeff = 0
    if dx:
        # regulate acceleration
        if abs(vx) > (vx_max * 4): theta_coeff = dx * 2
        elif abs(vx) > (vx_max * 2.5): theta_coeff = dx
        elif abs(vx) > (vx_max * 1.5): theta_coeff = 0
        else: theta_coeff = -dx

        # prevent colision with ground
        ax, ay = flat_surface[0]
        for bx, by in flat_surface[1:]:
            if bx > x: break
            ax, ay = bx, by
        
        # Groud line - y = gm * x + gb
        gm = (by - ay) / (bx - ax)
        gb = ay - gm * ax

        # Shuttle trajectory - y = sm * x + sb
        sm = vy / vx if vx != 0 else 1000 * (-dx)
        sb = y - sm * x

        # Intersection: gm * x + gb = sm * x + sb
        if gm != sm:
            ix = (sb - gb) / (gm - sm)
            iy = gm * ix + gb

            # intersection too soon -> lift
            if (
                dx == 1 and (bx - ix) < 100 or # going right
                dx == -1 and (ix - ax) < 100 # going left
            ): theta_coeff = 0

    else:
        # fit vx in parameters and prevent drift off
        landing_t = (lz_y - y) // vy if vy else 0
        drift_off_t = min(x - lz_x0, lz_x1 - x) // abs(vx) if vx else 100

        if (
            abs(vx) > vx_max * 0.9 or
            (drift_off_t - landing_t) < 3
        ): theta_coeff = np.sign(vx)

    theta = theta_coeff * 45

    # landing
    thrust = 4
    if dx == 0 and theta == 0 and abs(vy) < (vy_max * 0.95) or vy > 0:
        thrust = 3

    # rotate power. rotate is the desired rotation angle. power is the desired thrust power.
    print(f"{theta} {thrust}")
