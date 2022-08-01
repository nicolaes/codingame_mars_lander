from __future__ import annotations
import sys
import math
from typing import Tuple, Union
import numpy as np

Loc = Tuple[int, int]
Vector = Tuple[Loc, Loc]

def log(*args):
    print(repr(args), file=sys.stderr, flush=True)

def sign(n: int) -> int:
    if n == 0: return 0
    return 1 if n > 0 else -1

def same_sign(x, y) -> bool:
    return x ^ y >= 0

def speed(hs: int, vs: int) -> float:
    return math.hypot(hs, vs) * sign(hs * vs)

def np_to_tuple(v) -> Vector:
    return tuple(map(tuple, v))

def magnitude(v: Vector) -> float:
    a, b = v
    return math.hypot(*tuple(np.subtract(b, a)))

def vector_rho(v: Vector) -> float:
    a, b = v
    ax, ay = a
    bx, by = b
    return math.atan2(by -ay, bx - ax)

def subtract_vectors(a: Vector, b: Vector) -> Vector:
    return tuple(np.subtract(a, b))

def center_vector(v: Vector) -> Vector:
    new_v = np.subtract(v, v[0])
    return tuple(map(tuple, new_v))


KPID = Tuple[float, float, float]
class PID:
    def __init__(self, k: KPID = (1.2, 0.9, 0.3)):
        (self.kp, self.ki, self.kd) = k
        self.iteration = 0

    def set_target(self, target: tuple):
        self.target: Vector = target
        self.prev_error: Vector = tuple(np.subtract(target, target))
        self.cumulative_moving_average: Vector = self.prev_error

    def make_iteration(self, measurement: tuple) -> tuple:
        self.iteration += 1
        error = tuple(np.subtract(measurement, self.target))
        
        rate_of_change = np.subtract(self.prev_error, error)
        self.cumulative_moving_average = tuple(
            np.divide(
                np.add(error, np.multiply(self.cumulative_moving_average, self.iteration)),
                self.iteration + 1
            )
        )
        
        proportional = np.multiply(error, self.kp)
        integral = np.multiply(self.cumulative_moving_average, self.ki)
        derivative = np.multiply(rate_of_change, self.kd)

        self.prev_error = error
        delta_vector = np.multiply(np.add(proportional, integral, derivative), -1)
        return tuple(map(tuple, delta_vector))

class Planet:
    def __init__(self, n: int):
        self.n = n
        self.points: list[Loc] = []

    def on_surface_complete(self):
        heights = np.array(self.points)[:,1]
        diffs = np.diff(heights)
        plain_at = np.where(diffs == 0)[0][0]
        self.plain_start: Loc = self.points[plain_at]
        self.plain_end: Loc = self.points[plain_at + 1]
        self.plain_mid: Loc = tuple(
            int(sum(x) / 2) 
            for x in zip(self.plain_start, self.plain_end)
        )

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
mars_g = 3.711

# Save the Planet.
# Use less Fossil Fuel.

n = int(input())  # the number of points used to draw the surface of Mars.
planet: Planet = Planet(n)
target_vector_pid = PID()

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

    new_rotation = sign(me.rot)
    if abs(dist_x) > 0: 
        if abs(me.hs) < max_hs:
            new_rotation = - sign(dist_x)
    else:
        if abs(me.hs) > max_hs:
            new_rotation = sign(me.hs)
        elif abs(me.rot) > 0:
            new_rotation = 0

    thrust = 3
    if abs(me.vs) > max_vs / 2:
        thrust = 4
    
    
    if not hasattr(target_vector_pid, 'target'):
        target_direction: Vector = (me.loc, me.planet.plain_mid)
        target_magnitude = math.hypot(max_hs, max_vs) * 1
        target_vector: Vector = tuple(np.multiply(
            target_direction,
            target_magnitude / magnitude(target_direction)
        ))
        target_vector_pid.set_target(target_vector)

        log('target_direction', center_vector(target_direction))
        log('target_vector', center_vector(target_vector))

    my_vector = ((0, 0), (me.hs, me.vs))

    # direction to ground
    target_delta = target_vector_pid.make_iteration(my_vector)
    
    # direction of thrust cosnidering gravity
    mars_gravity_vector = ((0, 0), (0, -mars_g))
    target_thrust = np_to_tuple(np.subtract(target_delta, mars_gravity_vector))

    # shuttle rotation based on thrust vector
    thrust_rad = vector_rho(target_thrust)
    magnitude_multiplier = -1
    if thrust_rad > 0:
        thrust_rad = thrust_rad - math.pi / 2
        magnitude_multiplier = magnitude_multiplier * -1
    thrust_rad += math.pi / 2
    thrust_rad = -thrust_rad

    thrust_deg = round(math.degrees(thrust_rad))

    # thrust vector to magnitude
    thrust_mag = magnitude(target_thrust) * magnitude_multiplier
    if (thrust_mag < 2): thrust_mag = 3
    if (thrust_mag > 4): thrust_mag = 4
    thrust_mag = round(thrust_mag)

    #log('target_delta', center_vector(target_delta)[1])

    log('magnitude(target_thrust)', magnitude(target_thrust))
    log('vector_rho(target_thrust)', vector_rho(target_thrust))
    log('thrust_deg', thrust_deg)
    
    # subtract mars gravity to find thrust vector

    
    
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

    
    # R P. R is the desired rotation angle. P is the desired thrust power.
    # print(f"{new_rotation * 20} {thrust}")
    out = f"{thrust_deg} {thrust_mag}"
    log('out', out)
    print(f"{thrust_deg} {thrust_mag}")
