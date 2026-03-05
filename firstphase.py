from z3 import *
import random

# PARAMETERS
ROWS = 10
COLS = 10
T = 50

INIT_B1 = 20
INIT_B2 = 15
CHARGE_RATE = 5

# RANDOM OBSTACLES
obstacles = set()
for i in range(ROWS):
    for j in range(COLS):
        if random.random() < 0.3:
            obstacles.add((i,j))

# STOCHASTIC COSTS
cost1 = [1 + random.uniform(0,0.5) for _ in range(T)]
cost2 = [1 + random.uniform(0,0.5) for _ in range(T)]

opt = Optimize()

# WORKER1 POSITION
x1 = [Int(f"x1_{t}") for t in range(T)]
y1 = [Int(f"y1_{t}") for t in range(T)]

# WORKER2 POSITION
x2 = [Int(f"x2_{t}") for t in range(T)]
y2 = [Int(f"y2_{t}") for t in range(T)]

# CHARGER POSITION
xc = [Int(f"xc_{t}") for t in range(T)]
yc = [Int(f"yc_{t}") for t in range(T)]

# BATTERY
b1 = [Real(f"b1_{t}") for t in range(T)]
b2 = [Real(f"b2_{t}") for t in range(T)]

# CHARGING FLAGS
charge1 = [Bool(f"charge1_{t}") for t in range(T)]
charge2 = [Bool(f"charge2_{t}") for t in range(T)]

# WAIT FLAGS
wait1 = [Int(f"wait1_{t}") for t in range(T)]
wait2 = [Int(f"wait2_{t}") for t in range(T)]

# INITIAL CONDITIONS
opt.add(x1[0] == 0, y1[0] == 0)
opt.add(x2[0] == 9, y2[0] == 9)

opt.add(xc[0] == 5, yc[0] == 5)

opt.add(b1[0] == INIT_B1)
opt.add(b2[0] == INIT_B2)

# GRID CONSTRAINTS
for t in range(T):
    opt.add(x1[t] >= 0, x1[t] < ROWS)
    opt.add(y1[t] >= 0, y1[t] < COLS)

    opt.add(x2[t] >= 0, x2[t] < ROWS)
    opt.add(y2[t] >= 0, y2[t] < COLS)

    opt.add(xc[t] >= 0, xc[t] < ROWS)
    opt.add(yc[t] >= 0, yc[t] < COLS)

# OBSTACLE CONSTRAINTS
for t in range(T):
    for (ox,oy) in obstacles:
        opt.add(Or(x1[t] != ox, y1[t] != oy))
        opt.add(Or(x2[t] != ox, y2[t] != oy))
        opt.add(Or(xc[t] != ox, yc[t] != oy))

# MOTION
for t in range(T-1):

    opt.add(Abs(x1[t+1]-x1[t]) + Abs(y1[t+1]-y1[t]) <= 1)
    opt.add(Abs(x2[t+1]-x2[t]) + Abs(y2[t+1]-y2[t]) <= 1)
    opt.add(Abs(xc[t+1]-xc[t]) + Abs(yc[t+1]-yc[t]) <= 1)

# BATTERY UPDATE
for t in range(T-1):

    opt.add(
        b1[t+1] ==
        If(charge1[t],
           b1[t] + CHARGE_RATE,
           b1[t] - cost1[t])
    )

    opt.add(
        b2[t+1] ==
        If(charge2[t],
           b2[t] + CHARGE_RATE,
           b2[t] - cost2[t])
    )

# STOP IF BATTERY ZERO
for t in range(T-1):

    opt.add(
        If(b1[t] <= 0,
           And(x1[t+1] == x1[t], y1[t+1] == y1[t]),
           True)
    )

    opt.add(
        If(b2[t] <= 0,
           And(x2[t+1] == x2[t], y2[t+1] == y2[t]),
           True)
    )

# CHARGER CAN CHARGE ONLY ONE
for t in range(T):

    opt.add(
        Or(
            And(charge1[t], Not(charge2[t])),
            And(charge2[t], Not(charge1[t])),
            And(Not(charge1[t]), Not(charge2[t]))
        )
    )

# MUST BE AT SAME LOCATION  (new thing for us: use of Implies)
for t in range(T):

    opt.add(
        Implies(charge1[t],
                And(xc[t] == x1[t], yc[t] == y1[t]))
    )

    opt.add(
        Implies(charge2[t],
                And(xc[t] == x2[t], yc[t] == y2[t]))
    )

# WAIT TIME
for t in range(T):

    opt.add(
        wait1[t] ==
        If(And(b1[t] <= 0, Not(charge1[t])), 1, 0)
    )

    opt.add(
        wait2[t] ==
        If(And(b2[t] <= 0, Not(charge2[t])), 1, 0)
    )

# OBJECTIVE
total_wait = Sum(wait1) + Sum(wait2)

opt.minimize(total_wait)

# SOLVE
if opt.check() == sat:
    m = opt.model()
    print("Total wait:", m.evaluate(total_wait))
else:
    print("UNSAT")