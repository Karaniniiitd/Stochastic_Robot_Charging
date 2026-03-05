from z3 import *
import random

# USER INPUT
rows = int(input("Enter rows: "))
cols = int(input("Enter cols: "))
T = int(input("Enter time horizon: "))

# START POSITIONS
x1_start = int(input("Worker1 start x: "))
y1_start = int(input("Worker1 start y: "))

x2_start = int(input("Worker2 start x: "))
y2_start = int(input("Worker2 start y: "))

xc_start = int(input("Charger start x: "))
yc_start = int(input("Charger start y: "))

# BATTERY
b1_init = int(input("Worker1 battery: "))
b2_init = int(input("Worker2 battery: "))

# STOCHASTIC COST
cost1 = [1 + random.uniform(0,0.5) for _ in range(T)]
cost2 = [1 + random.uniform(0,0.5) for _ in range(T)]

opt = Optimize()

# VARIABLES
x1 = [Int(f"x1_{t}") for t in range(T)]
y1 = [Int(f"y1_{t}") for t in range(T)]

x2 = [Int(f"x2_{t}") for t in range(T)]
y2 = [Int(f"y2_{t}") for t in range(T)]

xc = [Int(f"xc_{t}") for t in range(T)]
yc = [Int(f"yc_{t}") for t in range(T)]

b1 = [Real(f"b1_{t}") for t in range(T)]
b2 = [Real(f"b2_{t}") for t in range(T)]

charge1 = [Bool(f"charge1_{t}") for t in range(T)]
charge2 = [Bool(f"charge2_{t}") for t in range(T)]

wait1 = [Int(f"wait1_{t}") for t in range(T)]
wait2 = [Int(f"wait2_{t}") for t in range(T)]

# INITIAL CONDITIONS
opt.add(x1[0] == x1_start, y1[0] == y1_start)
opt.add(x2[0] == x2_start, y2[0] == y2_start)
opt.add(xc[0] == xc_start, yc[0] == yc_start)

opt.add(b1[0] == b1_init)
opt.add(b2[0] == b2_init)

# GRID CONSTRAINTS
for t in range(T):

    opt.add(x1[t] >= 0, x1[t] < rows)
    opt.add(y1[t] >= 0, y1[t] < cols)

    opt.add(x2[t] >= 0, x2[t] < rows)
    opt.add(y2[t] >= 0, y2[t] < cols)

    opt.add(xc[t] >= 0, xc[t] < rows)
    opt.add(yc[t] >= 0, yc[t] < cols)

# MOTION
for t in range(T-1):

    opt.add(Abs(x1[t+1]-x1[t]) + Abs(y1[t+1]-y1[t]) <= 1)

    opt.add(Abs(x2[t+1]-x2[t]) + Abs(y2[t+1]-y2[t]) <= 1)

    opt.add(Abs(xc[t+1]-xc[t]) + Abs(yc[t+1]-yc[t]) <= 1)

# BATTERY
for t in range(T-1):

    opt.add(
        b1[t+1] ==
        If(charge1[t],
           b1[t] + 5,
           b1[t] - cost1[t])
    )

    opt.add(
        b2[t+1] ==
        If(charge2[t],
           b2[t] + 5,
           b2[t] - cost2[t])
    )

# WAIT
for t in range(T):

    opt.add(wait1[t] == If(And(b1[t] <= 0, Not(charge1[t])), 1, 0))
    opt.add(wait2[t] == If(And(b2[t] <= 0, Not(charge2[t])), 1, 0))

# OBJECTIVE
total_wait = Sum(wait1) + Sum(wait2)

opt.minimize(total_wait)

if opt.check() == sat:

    m = opt.model()
    print("Total wait:", m.evaluate(total_wait))

else:

    print("UNSAT")