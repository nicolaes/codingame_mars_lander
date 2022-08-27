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

        # log('proportional', error)
        # log('integral    ', self.cumulative_moving_average)
        # log('derivative  ', rate_of_change)

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
            int(sum(x) // 2) 
            for x in zip(self.plain_start, self.plain_end)
        )

class Rover:
    def __init__(self, planet, x, y, hs, vs, f, r, p):
        self.planet = planet
        self.loc: Loc = np.array([x, y])
        self.hs: int = hs
        self.vs: int = vs
        self.speed = np.array([vs, hs])
        self.fuel = f
        self.rot = r
        self.rotRad = r / 180 * math.pi
        self.power = p

        rotation_rad = math.radians(self.rot + 90)
        self.acc = np.array([ math.cos(rotation_rad), math.sin(rotation_rad) ]) * self.power
        self.acc[1] -= mars_g
        

    def dist_to_plain(self) -> Tuple[int, int]:
        diff = np.subtract(self.planet.plain_start, self.loc)
        if diff[0] < 0:
            end_diff = self.planet.plain_end[0] - self.loc[0]
            diff[0] = end_diff if end_diff < 0 else 0
        return tuple(diff)

    def dist_to_reach_speed(self, v_final: Dir):
        # t = (vf - vi) / a
        # dist = vi * t + 1/2 a * t^2

        v_final_np = np.array(v_final)
        v_init = np.array((self.hs, self.vs))
        v_diff = v_final_np - v_init

        # split max acceleration between X and Y to reach v_diff evenly
        acc_theta = math.atan2(*np.flip(v_diff))


        if acc_theta < 0:
            max_horiz_acc = 4
            max_vertical_acc = mars_g
        else:
            max_horiz_acc = math.sqrt(1 - mars_g ** 2 / 16) * 4
            max_vertical_acc = 4 - mars_g
        
        acc = np.array((math.cos(acc_theta), math.sin(acc_theta))) * (max_horiz_acc, max_vertical_acc)
        acc_no_zero = np.where(acc != 0, acc, 1)
        time = v_diff / acc_no_zero

        # log('time', max_horiz_acc)
        # log('max_vertical_acc', max_vertical_acc)
        # log('acc_theta', acc_theta)

        # it takes 6 turns to start accelerating up if shuttle is lateral
        # + account for stopping the current acceleration
        time += 6

        dist = v_init * time + 1/2 * acc * time ** 2

        # V_init_y = -22 * 116 + 1/2 * 0.2 * 116^2

        # distanta in care Vx si Vy ajung la 0 in acelasi timp
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

# PID based on distance from target and landing offset

###########
Kp = 2.5 ##
Ki = 0.0 ##
Kd = 0.00 #
###########

distance_pid = PID((Kp, Ki, Kd))
distance_pid.set_target((0, 0)) # target is to reach (0, 0) offset to LZ (e.g. land)

for i in range(n):
    # land_x: X coordinate of a surface point. (0 to 6999)
    # land_y: Y coordinate of a surface point. By linking all the points together in a sequential fashion, you form the surface of Mars.
    land_x, land_y = [int(j) for j in input().split()]
    planet.points.append((land_x, land_y))

# event to compute planet stats
planet.on_surface_complete()

# game loop
large_ratio_increased = 0
while True:
    # hs: the horizontal speed (in m/s), can be negative.
    # vs: the vertical speed (in m/s), can be negative.
    # f: the quantity of remaining fuel in liters.
    # r: the rotation angle in degrees (-90 to 90).
    # p: the thrust power (0 to 4).
    x, y, hs, vs, f, r, p = [int(i) for i in input().split()]
    me = Rover(planet, x, y, hs, vs, f, r, p)

    # how far is LZ
    dist = me.dist_to_reach_speed(np.array([0, 0]) * 1/2)
    
    # unde ajung la V = (0, 0) daca pun frana cu toata puterea:
    projected_stopping_location = dist + me.loc
    
    # dorinta mea: sa ajung la LZ
    # in ce directie ar trebui sa ma deplasez?
    road_to_lz = np.array(me.planet.plain_mid) - projected_stopping_location

    # PID determines how much should we correct for error
    PID_adjust_to_lz = distance_pid.make_iteration(road_to_lz)

    # direction of acceleration considering gravity
    turns_for_gravity_to_act = 1
    mars_acceleration_vector = np.array((0, -mars_g)) * turns_for_gravity_to_act
    target_acceleration = (PID_adjust_to_lz - mars_acceleration_vector) * -1

    # shuttle rotation based on acceleration vector direction
    acc_rho = vector_rho(target_acceleration)
    shuttle_rotation = 0
    if acc_rho > 0:
        # cadran 1 sau 2
        # directia de accelerare e in SUS
        shuttle_rotation = math.degrees(acc_rho) - 90
    else:
        # cadran 3 sau 4
        # las gravitatea sa actioneze; throttle doar spre lateral
        target_acceleration[1] = 0 

        if acc_rho < - math.pi / 2:
            # cadran 3
            shuttle_rotation = 90 # inclinatie maxima spre stanga
        else:
            # cadran 4
            shuttle_rotation = -90 # inclinatie maxima spre stanga
    
    shuttle_rotation = round(shuttle_rotation)
    
    # thrust vector to magnitude
    thrust = magnitude(target_acceleration)
    if (thrust < 0): thrust = 0
    if (thrust > 4): thrust = 4
    thrust = round(thrust)

    # take over just before landing if params are correct
    dist_to_lz = me.dist_to_plain()
    time_to_land = abs(dist_to_lz[1] / me.vs) if me.vs !=0 else 1000
    if dist_to_lz[0] == 0 and time_to_land < 3 and me.hs < max_hs and me.vs < max_vs:
        shuttle_rotation = 0
        thrust = 3

    # log('ROAD TO LZ:', road_to_lz)
    # log('ADJUST    :', PID_adjust_to_lz)
    # log('acc - magn:', magnitude(target_acceleration))
    # log('acc - grad:', math.degrees(acc_rho))
    # log('thrust    :', thrust)

    turns_to_lz = np.array(me.planet.plain_mid) - me.loc
    turns_to_lz = np.floor_divide(turns_to_lz, me.speed, 
        out = np.full_like(me.speed, 1000), where = me.speed!= 0)
    # log('turns_to_lz', turns_to_lz)

    turns_to_lz = np.abs(turns_to_lz)
    turns_to_lz_ratio = turns_to_lz[0] / turns_to_lz[1] if turns_to_lz[1] != 0 else 1000
    # log('turns_to_lz_ratio', turns_to_lz_ratio)

    if turns_to_lz_ratio > 20 or (
        large_ratio_increased > 20 and 
        turns_to_lz_ratio > 10
    ):
        shuttle_rotation = 0
        thrust = 4
        large_ratio_increased += 1
    # log('large_ratio_increased', large_ratio_increased)
    
    # R P. R is the desired rotation angle. P is the desired thrust power.
    out = f"{shuttle_rotation} {thrust}"
    # log('now', f"{me.rot} {me.power}")
    # log('out', out)
    print(out)
