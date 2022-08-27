import sys
import math
import numpy as np

def log(*args):
    print(*args, file=sys.stderr, flush=True)

# Save the Planet.
# Use less Fossil Fuel.

flat_land_i = 0
land = []
land_n = int(input())  # the number of points used to draw the surface of Mars.
for i in range(land_n):
    land_x, land_y = [int(j) for j in input().split()]
    land.append((land_x, land_y))

    if land[i-1] and land_y == land[i-1][1]:
        flat_land_i = i-1

lz = (
    (land[flat_land_i][0] + land[flat_land_i + 1][0]) // 2,
    land[flat_land_i][1]
)
(lx_x, lz_y) = lz

max_hs = 20
max_vs = 40

# game loop
while True:
    # hs: the horizontal speed (in m/s), can be negative.
    # vs: the vertical speed (in m/s), can be negative.
    # f: the quantity of remaining fuel in liters.
    # r: the rotation angle in degrees (-90 to 90).
    # p: the thrust power (0 to 4).
    x, y, hs, vs, f, r, p = [int(i) for i in input().split()]
    pos = (x, y)
    speed = (vs, hs)

    def navigate() -> tuple[int, int]:
        # if speed ok -> do nothing
        # OK = 

        x_acc_dir = 0 # or -1 or 1
        y_acc_dir = 0

        # if speed too large -> slow down
        if abs(vs) > max_vs + 5:
            return (0, np.sign(vs) * -1)
        if abs(vs) > max_vs:
            y_acc_dir = np.sign(vs) * -1

        if abs(hs) > max_hs * 2:
            return (np.sign(hs) * -1, 0)
        if abs(hs) > max_hs:
            x_acc_dir = np.sign(hs) * -1

        if x_acc_dir != 0 or y_acc_dir != 0:
            return (x_acc_dir, y_acc_dir)
        
        # if speed too small -> 
        #      if no speed too large: speed up
        #      if othe speed too large: do nothing
        


        target_hs = 0
        # if x > lz_x + 500
        target_vs = 0
        #if lz_
        return (x_acc_dir, y_acc_dir)

    def nav_to_rot(nav):
        nav_x, nav_y = nav
        if nav_x == 0 and nav_y == 0:
            return 0

        R = math.degrees(math.atan2(nav_y, nav_x))

        # negatives
        if R < -100: return 90
        if R < -80 : return 0
        if R < 0   : return -90

        R -= 90
        return round(R / 45) * 45
    
    nav = navigate()
    log('nav', nav)
    
    R = nav_to_rot(nav)
    log('r', R)
    P = 3 if nav[1] < 0 else 4

    # R P. R is the desired rotation angle. P is the desired thrust power.
    print(f"{R} {P}")
