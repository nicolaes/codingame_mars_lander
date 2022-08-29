import sys
import math
import random
from dataclasses import dataclass
from typing import Tuple, List
import numpy as np
from sklearn import preprocessing
from datetime import datetime

def log(*args):
    print(*args, file=sys.stderr, flush=True)

@dataclass
class Rover:
    x: float
    y: float
    vx: float
    vy: float
    fuel: int
    rotate: int
    power: int

@dataclass
class Surface:
    points: List[Tuple[int, int]]
    start: Tuple[int, int]
    end: Tuple[int, int]

Chromosome = List[int]

@dataclass
class ScoredChrom:
    c: Chromosome
    distance_to_lz: float
    overspeed: float
    rotate_delta: int
    fuel: int

def is_rover_outside_map(rover: Rover):
    return (
        rover.x < 0 or rover.x > 6999 or
        rover.y < 0 or rover.y > 2999
    )

def get_surface_line_below_rover(rover: Rover, surface: Surface):
    end_i = 0
    while surface.points[end_i][0] < rover.x:
        end_i += 1
    
    # surface below rover
    sx, sy = surface.points[end_i - 1]
    ex, ey = surface.points[end_i]

    # surface y = mx + b
    m = (ey - sy) / (ex - sx)
    b = sy - m * sx

    return (m, b)

def is_rover_on_surface(rover: Rover, surface: Surface):
    m, b = get_surface_line_below_rover(rover, surface)
    return rover.y <= (m * rover.x + b)

def simulate_trajectory(rover: Rover, surface: Surface, trajectory: List[int]) -> Rover:
    # start_sim = datetime.now().microsecond
    # initialized_times = False
    # total_times = None

    i = 0
    prev_r = rover
    r = rover
    on_surface = False
    while (
        not (outside_map := is_rover_outside_map(r)) and
        not (on_surface := is_rover_on_surface(r, surface)) and
        i < len(trajectory)
    ):
        # times = np.append([], datetime.now().microsecond)
        
        # Next power based on trajectory
        rotate = np.clip(r.rotate + trajectory[i], -90, 90)
        # try:
        power = np.clip(r.power + trajectory[i + 1], 0, 4) if r.fuel > 0 else 0
        # except:
        #     log(i, len(trajectory))
        i += 2

        # times = np.append(times, datetime.now().microsecond)

        # Acceleration
        theta = math.radians(rotate + 90)
        ax = power * math.cos(theta)
        ay = power * math.sin(theta) - 3.711

        # times = np.append(times, datetime.now().microsecond)

        # Next speed
        vx = r.vx + ax
        vy = r.vy + ay

        # times = np.append(times, datetime.now().microsecond)

        # Next position
        x = r.x + r.vx + ax/2
        y = r.y + r.vy + ay/2

        # Next fuel
        fuel = r.fuel - power
        if fuel < 0: fuel = 0

        # times = np.append(times, datetime.now().microsecond)

        # Next rover
        prev_r = r
        r = Rover(x, y, vx, vy, fuel, rotate, power)

        # times = np.append(times, datetime.now().microsecond)
        # diff = np.diff(times)

        # if initialized_times:
        #     total_times = total_times + diff
        # else:
        #     total_times = diff
        #     initialized_times = True

    
    if on_surface:
        # place it on surface
        m, b = get_surface_line_below_rover(prev_r, surface)
        r.x = prev_r.x
        r.y = m * prev_r.x + b

    # log(total_times)
    # log(datetime.now().microsecond - start_sim)

    return r

def get_dist_to_lz(rover: Rover, surface: Surface):
    sx = np.clip(rover.x, surface.start[0], surface.end[0])
    sy = surface.start[1]
    return math.hypot(rover.x - sx, rover.y - sy)

GA_PERFORMANCE_PROPORTION = 0.3
GA_RANDOM_PROPORTION = 0.2
GA_MUTATION_CHANCE = 0.005

CHROM_ROTATIONS = range(-15, 16)
CHROM_POWERS = range(-1, 2)

# GA
## generate initial population
## RANK by distance to LZ: 0 - 100
## ## then by speed within limits: 0 - 100
## ## then by final orientation 0: 0 - 100
## ## (then by fuel: 0 - 100)
## is 90ms up? stop and use the best
## is any at score 300? stop and use it
## select 30% best, then 20% random
## create child to fill population; 0.1 - 1% chance to do mutation

def create_chromosome(chrom_size):
    chrom = []
    for i in range(chrom_size):
        chrom.append(random.choice(CHROM_ROTATIONS))
        chrom.append(random.choice(CHROM_POWERS))
    return chrom

def score_chrom(rover: Rover, surface: Surface, chrom: Chromosome) -> List[float]:
    r = simulate_trajectory(rover, surface, chrom)
    dist_to_lz = get_dist_to_lz(r, surface)
    overspeed = max(0, abs(r.vx) - 20) + max(0, abs(r.vy) - 40)
    rotate_delta = abs(r.rotate)
    return [dist_to_lz, overspeed, rotate_delta, r.fuel]

def generate_population(pop_size, chrom_size) -> List[Chromosome]:
    return [create_chromosome(chrom_size) for i in range(pop_size)]

def get_best_genetics(rover: Rover, surface: Surface):
    ga_start = datetime.now().microsecond

    pop_size = 10
    chrom_size = 30
    population = generate_population(pop_size, chrom_size)

    i = 0
    best_chrom_idx = None
    winner = None
    while not winner and (datetime.now().microsecond - ga_start) < 80000:
        # times = np.append([], datetime.now().microsecond) ### 1

        # Normalize population scoring metrics
        pop_metrics = np.array([score_chrom(rover, surface, c) for c in population])
        pop_norm = preprocessing.minmax_scale(pop_metrics)

        # times = np.append(times, datetime.now().microsecond) ### 2

        # By goals
        reached_lz = pop_metrics[:,0] < 1 # dist
        within_speed = reached_lz & (pop_metrics[:,1] == 0) # overspeed
        correct_rotation = within_speed & (pop_metrics[:,2] == 0) # rotation

        # times = np.append(times, datetime.now().microsecond) ### 3

        # Add scores conditionally
        pop_scores = pop_norm[:,0]
        pop_scores += np.where(reached_lz, pop_norm[:,1], 0)
        pop_scores += np.where(within_speed, pop_norm[:,2], 0)
        pop_scores += np.where(correct_rotation, pop_norm[:,3], 0)

        # times = np.append(times, datetime.now().microsecond) ### 4

        # Sort by score
        sorted_population_idx = pop_scores.argsort()
        best_chrom_idx = sorted_population_idx[-1]
        if correct_rotation[best_chrom_idx]:
            winner = population[best_chrom_idx]
            break

        # Select best + random
        perf_size = int(len(population) * GA_PERFORMANCE_PROPORTION)
        select = [population[i] for i in sorted_population_idx[-perf_size:]]
        
        rand_size = int(len(population) * GA_RANDOM_PROPORTION)
        rand_chrom_indexes = random.choices(sorted_population_idx[:perf_size + 1], k = rand_size)
        select += [population[i] for i in rand_chrom_indexes]
        
        # Populate with children
        while len(select) < pop_size:
            p1, p2 = random.choices(select, k = 2)
            half_point = chrom_size // 2
            half_point -= half_point % 2

            child = p1[:half_point] + p2[half_point:]
            mutation_idx = int(1 / GA_MUTATION_CHANCE * random.random())
            if mutation_idx < chrom_size: 
                mutation_idx -= mutation_idx % 2
                child[mutation_idx] = random.choice(CHROM_ROTATIONS)
                child[mutation_idx + 1] = random.choice(CHROM_POWERS)

            select.append(child)
        
        population = select
        i += 1
        
        # times = np.append(times, datetime.now().microsecond) ### 4
        # log(np.diff(times))

    log(i)
    return winner if winner else population[best_chrom_idx]


surface = Surface([], (0, 0), (0, 0))
surface_n = int(input())  # the number of points used to draw the surface of Mars.
for i in range(surface_n):
    # land_x: X coordinate of a surface point. (0 to 6999)
    # land_y: Y coordinate of a surface point. By linking all the points together in a sequential fashion, you form the surface of Mars.
    land_x, land_y = [int(j) for j in input().split()]
    surface.points.append((land_x, land_y))

    if i > 0 and land_y == surface.points[i-1][1]:
        surface.start = surface.points[i-1]
        surface.end = surface.points[i]



# game loop
ga = False
while True:
    # h_speed: the horizontal speed (in m/s), can be negative.
    # v_speed: the vertical speed (in m/s), can be negative.
    # fuel: the quantity of remaining fuel in liters.
    # rotate: the rotation angle in degrees (-90 to 90).
    # power: the thrust power (0 to 4).
    x, y, h_speed, v_speed, fuel, rotate, power = [int(i) for i in input().split()]

    rover = Rover(x, y, h_speed, v_speed, fuel, rotate, power)

    winner = get_best_genetics(rover, surface)
    log(winner)

    # rotate power. rotate is the desired rotation angle. power is the desired thrust power.
    r = 0
    p = 0
    print(f"{r} {p}")
