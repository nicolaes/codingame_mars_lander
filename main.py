from __future__ import annotations
import sys
import math
from typing import Tuple, Union
import numpy as np

Loc = Tuple[int, int]
Dir = Tuple[float, float]
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

def magnitude(v) -> float:
    return math.hypot(*v)

def vector_rho(v: Dir) -> float:
    a, b = v
    return math.atan2(b, a)

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

    def set_target(self, target):
        self.target = np.array(target)
        self.prev_error = np.zeros_like(target)
        self.cumulative_moving_average = self.prev_error

    def make_iteration(self, measurement):
        self.iteration += 1
        error = np.array(measurement) - self.target
        
        rate_of_change = self.prev_error - error
        self.cumulative_moving_average = (error + self.cumulative_moving_average * self.iteration) / (self.iteration + 1)
        
        proportional = error * self.kp
        integral = self.cumulative_moving_average * self.ki
        derivative = rate_of_change * self.kd

        self.prev_error = error
        delta_vector = (proportional + integral + derivative) * -1
        return delta_vector

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
        self.hs: int = hs
        self.vs: int = vs
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

    def dist_to_target_speed(self, v_final: Dir = (0, 0)):
        # t = (vf - vi) / a
        # dist = vi * t + 1/2 a * t^2

        v_final_np = np.array(v_final)
        v_init = np.array((self.hs, self.vs))
        v_diff = v_final_np - v_init
        
        # split max acceleration between X and Y to reach v_diff evenly
        v_theta = math.atan2(*np.flip(v_diff))
        max_vertical_acc = mars_g if v_theta < 0 else 4 - mars_g
        acc = np.array((math.cos(v_theta), math.sin(v_theta))) * (4, max_vertical_acc)
        
        acc_no_zero = np.where(acc != 0, acc, 1)
        time = v_diff / acc_no_zero
        dist = v_init * time + 1/2 * acc * time ** 2
        return dist

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

max_hs, max_vs = 20, 40
mars_g = 3.711

# Save the Planet.
# Use less Fossil Fuel.

n = int(input())  # the number of points used to draw the surface of Mars.
planet: Planet = Planet(n)
target_vector_pid = PID((1, 0.7, 0.3))

# PID based on distance from target and landing offset
distance_pid = PID((1.2, 0, 0))
distance_pid.set_target((0, 0)) # target is to reach (0, 0) offset to LZ (e.g. land)

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

    # how far is LZ
    dist = me.dist_to_target_speed((20, 20))
    lz_landing_offset = dist + me.loc - me.planet.plain_mid    
    target_delta = distance_pid.make_iteration(lz_landing_offset)
    
    # log('lz_landing_offset', lz_landing_offset)
    
    # direction of thrust considering gravity
    mars_gravity_vector = (0, -mars_g)
    target_acceleration = (target_delta - mars_gravity_vector) * -1

    # shuttle rotation based on thrust vector
    acceleration_rad = vector_rho(target_acceleration)
    magnitude_multiplier = -1
    if acceleration_rad > 0:
        acceleration_rad = acceleration_rad - math.pi
        magnitude_multiplier = magnitude_multiplier * -1
    acceleration_rad += math.pi / 2
    acceleration_rad = -acceleration_rad

    thrust_deg = round(math.degrees(acceleration_rad))

    # thrust vector to magnitude
    thrust_mag = magnitude(target_acceleration) * magnitude_multiplier
    if (thrust_mag < 2): thrust_mag = 3
    if (thrust_mag > 4): thrust_mag = 4
    thrust_mag = round(thrust_mag)

    log('delta - vect:', target_delta)
    log('accel - vect:', target_acceleration)
    log('acc - magn:', magnitude(target_acceleration))
    log('acc - grad:', math.degrees(vector_rho(target_acceleration)))
    # log('thrust_deg', thrust_deg)
    
    
    # R P. R is the desired rotation angle. P is the desired thrust power.
    out = f"{thrust_deg} {thrust_mag}"
    log('out', out)
    print(out)
