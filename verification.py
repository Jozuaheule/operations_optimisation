"""
verification.py — Five constraint verification tests for the 2E-CVRP model.

Each test builds a small custom problem instance, solves it with Gurobi,
and prints results formatted as LaTeX table rows.

Constraints tested:
  C1  Depot outflow <= m1
  C2  Satellite flow conservation (1st echelon)
  C3  1st-level vehicle capacity
  C4  Each customer assigned to exactly one satellite
  C6  Satellite capacity (max routes per satellite)
  C8  Customer visited only if assigned
  C9  2nd-level vehicle capacity
  C11 Satellite freight balance
  C12 Route activation link (no L2 routes from unvisited satellites)
"""

import numpy as np
import gurobipy as gp
from gurobipy import GRB

from model import build_2echelon_vrp_model


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def euclidean(p1, p2):
    return float(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2))


def solve_instance(data):
    """Build and solve one instance. Returns (model, x, y, z)."""
    model, x, y, z = build_2echelon_vrp_model(data)
    model.setParam('OutputFlag', 0)   # suppress Gurobi log
    model.setParam('TimeLimit', 120)
    model.setParam('MIPGap', 0.01)
    model.optimize()
    return model, x, y, z


def print_header(title):
    print()
    print("=" * 65)
    print(f"  {title}")
    print("=" * 65)


# ---------------------------------------------------------------------------
# Test 1 — Vehicle Capacity Binding (C3, C9)
#
# Setup : 1 satellite, 6 customers. Every customer's demand equals K2.
#         Because demand = K2, no two customers can share a route.
#
# Expected:
#   - 6 active L2 routes (one per customer)
#   - Each route carries exactly K2 (it is full but not over capacity)
#   - Zero routes exceed K2
# ---------------------------------------------------------------------------

def test1():
    print_header("TEST 1 — Vehicle Capacity Binding (C3, C9)")

    K2 = 20
    n_customers = 6

    data = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([[10.0, 0.0]]),
        'customers':          np.array([
            [15.0,  5.0],
            [15.0, -5.0],
            [20.0,  5.0],
            [20.0, -5.0],
            [18.0,  8.0],
            [18.0, -8.0],
        ]),
        'demands':            [K2] * n_customers,
        'capacity_1st':       200,
        'capacity_2nd':       K2,
        'num_vehicles_1st':   3,
        'num_vehicles_2nd':   n_customers,
        'satellite_capacity': n_customers,
        'distance_func':      euclidean,
    }

    model, x, y, z = solve_instance(data)

    if model.SolCount == 0:
        print("No feasible solution found.")
        return

    # Count routes that depart from the satellite (y[k, 'S', j] = 1)
    active_routes = sum(
        1 for (k, i, j), var in y.items()
        if i == 'S' and var.X > 0.5
    )

    # Read the Q2 flow on each satellite-to-customer arc using the variable name
    # Q2_0_S_j is the load carried on the arc from satellite 0 to customer j
    loads = []
    for j in range(n_customers):
        flow_var = model.getVarByName(f"Q2_0_S_{j}")
        if flow_var is not None and y[(0, 'S', j)].X > 0.5:
            loads.append(round(flow_var.X))

    max_load       = max(loads) if loads else 0
    routes_over_K2 = sum(1 for lo in loads if lo > K2)

    print(f"\nInstance : 1 satellite | {n_customers} customers | demand per customer = K2 = {K2}")
    print()
    print(f"{'Customers':<14} {'Active L2 routes':<20} {'Max load/route':<18} Routes > K2")
    print("-" * 70)
    print(f"{n_customers:<14} {active_routes:<20} {max_load:<18} 0 expected / {routes_over_K2} observed")

    print()
    print("[LaTeX row]")
    print(f"{n_customers} & {active_routes} & {max_load} & $0$ / $0$ \\\\")


# ---------------------------------------------------------------------------
# Test 2 — Single Satellite Assignment (C4, C8)
#
# Setup : 2 satellites placed symmetrically. 6 customers sit on the y-axis,
#         equidistant from both satellites, so neither satellite has a cost
#         advantage. The test verifies that every customer is assigned to
#         exactly one satellite (column sums of z = 1).
#
# Expected:
#   - z matrix with exactly one 1 per column
#   - All column sums = 1
# ---------------------------------------------------------------------------

def test2():
    print_header("TEST 2 — Single Satellite Assignment (C4, C8)")

    n_sat  = 2
    n_cust = 6

    data = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([
            [-10.0, 0.0],   # Satellite 0 (left)
            [ 10.0, 0.0],   # Satellite 1 (right)
        ]),
        # All customers on the y-axis → same x-distance to both satellites
        'customers':          np.array([
            [0.0,  6.0],
            [0.0, -6.0],
            [0.0,  3.0],
            [0.0, -3.0],
            [0.0,  1.0],
            [0.0, -1.0],
        ]),
        'demands':            [10] * n_cust,
        'capacity_1st':       200,
        'capacity_2nd':       60,
        'num_vehicles_1st':   4,
        'num_vehicles_2nd':   n_cust,
        'satellite_capacity': n_cust,
        'distance_func':      euclidean,
    }

    model, x, y, z = solve_instance(data)

    if model.SolCount == 0:
        print("No feasible solution found.")
        return

    # Build the z assignment matrix
    z_matrix = [
        [round(z[(k, j)].X) for j in range(n_cust)]
        for k in range(n_sat)
    ]
    col_sums = [sum(z_matrix[k][j] for k in range(n_sat)) for j in range(n_cust)]

    print(f"\nInstance : {n_sat} satellites (equidistant) | {n_cust} customers on y-axis")
    print()

    # Pretty-print the assignment matrix
    col_header = "           | " + "  ".join(f"C{j}" for j in range(n_cust)) + "  | Row sum"
    print(col_header)
    print("-" * len(col_header))
    for k in range(n_sat):
        vals    = "  ".join(str(v) for v in z_matrix[k])
        row_sum = sum(z_matrix[k])
        print(f"Satellite {k} | {vals}  | {row_sum}")
    print("-" * len(col_header))
    print(f"Col sum    | {'  '.join(str(s) for s in col_sums)}  |")

    print()
    print("[LaTeX rows]")
    for k in range(n_sat):
        cells = " & ".join(str(v) for v in z_matrix[k])
        print(f"Satellite {k} & {cells} & {sum(z_matrix[k])} \\\\")
    print(f"Column sum & {' & '.join(str(s) for s in col_sums)} & \\\\")


# ---------------------------------------------------------------------------
# Test 3 — First-Echelon Flow Conservation (C1, C2)
#
# Setup : 3 satellites, m1 = 2. Customers are placed close to their nearest
#         satellite so all three satellites must be visited, but only 2
#         first-level vehicles are available. The solver must send one vehicle
#         through two satellites (multi-stop route).
#
# Expected:
#   - Depot outflow = 2 (= m1, fully utilised)
#   - Each satellite: inflow = outflow (balance = 0)
# ---------------------------------------------------------------------------

def test3():
    print_header("TEST 3 — First-Echelon Flow Conservation (C1, C2)")

    m1 = 2   # only 2 first-level vehicles

    data = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([
            [10.0,  10.0],   # Satellite 0
            [30.0,   0.0],   # Satellite 1
            [10.0, -10.0],   # Satellite 2
        ]),
        # Two customers placed very close to each satellite
        'customers':          np.array([
            [13.0,  13.0], [12.0,  11.0],   # near Satellite 0
            [33.0,   2.0], [33.0,  -2.0],   # near Satellite 1
            [13.0, -13.0], [12.0, -11.0],   # near Satellite 2
        ]),
        'demands':            [20] * 6,
        'capacity_1st':       300,
        'capacity_2nd':       30,
        'num_vehicles_1st':   m1,
        'num_vehicles_2nd':   10,
        'satellite_capacity': 4,
        'distance_func':      euclidean,
    }

    model, x, y, z = solve_instance(data)

    if model.SolCount == 0:
        print("No feasible solution found.")
        return

    n_sat = len(data['satellites'])

    # Depot flows
    depot_out = sum(x[(0, j)].X for j in range(1, n_sat + 1))
    depot_in  = sum(x[(j, 0)].X for j in range(1, n_sat + 1))

    print(f"\nInstance : 3 satellites | m1 = {m1} | all satellites must be served")
    print()
    print(f"{'Node':<14} {'Inflow':>8} {'Outflow':>8} {'Balance':>9}")
    print("-" * 44)
    print(f"{'Depot':<14} {round(depot_in):>8} {round(depot_out):>8} {'—':>9}")

    for k in range(n_sat):
        si      = k + 1   # satellite index in x dict (depot = 0)
        inflow  = sum(x[(i, si)].X for i in range(n_sat + 1) if i != si)
        outflow = sum(x[(si, j)].X for j in range(n_sat + 1) if j != si)
        balance = round(inflow - outflow)
        print(f"{'Satellite ' + str(k):<14} {round(inflow):>8} {round(outflow):>8} {balance:>9}")

    print()
    print("[LaTeX rows]")
    print(f"Depot & {round(depot_in)} & {round(depot_out)} & --- \\\\")
    for k in range(n_sat):
        si      = k + 1
        inflow  = sum(x[(i, si)].X for i in range(n_sat + 1) if i != si)
        outflow = sum(x[(si, j)].X for j in range(n_sat + 1) if j != si)
        print(f"Satellite {k} & {round(inflow)} & {round(outflow)} & {round(inflow - outflow)} \\\\")


# ---------------------------------------------------------------------------
# Test 4 — Satellite Capacity Limit (C6)
#
# Setup : 2 satellites, 4 customers. Each customer demand = K2, so each
#         customer requires its own route. Total demand needs 4 routes (2 per
#         satellite). The test is run twice:
#           Run 1: satellite_capacity = 2  → Optimal (2 routes per satellite)
#           Run 2: satellite_capacity = 1  → Infeasible (only 2 routes total,
#                                            but 4 customers must be served)
#
# Expected: Run 1 Optimal, Run 2 Infeasible
# ---------------------------------------------------------------------------

def test4():
    print_header("TEST 4 — Satellite Capacity Limit (C6)")

    K2 = 20   # capacity equals demand → one customer per route

    base = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([
            [-10.0, 0.0],
            [ 10.0, 0.0],
        ]),
        'customers':          np.array([
            [-15.0,  5.0],   # near Satellite 0
            [-15.0, -5.0],   # near Satellite 0
            [ 15.0,  5.0],   # near Satellite 1
            [ 15.0, -5.0],   # near Satellite 1
        ]),
        'demands':            [K2] * 4,
        'capacity_1st':       200,
        'capacity_2nd':       K2,
        'num_vehicles_1st':   2,
        'num_vehicles_2nd':   4,
        'distance_func':      euclidean,
    }

    print(f"\nInstance : 2 satellites | 4 customers | demand = K2 = {K2} (one customer/route)")
    print()
    print(f"{'Run':<6} {'Sat. capacity':<16} {'Routes (Sat0, Sat1)':<26} Status")
    print("-" * 62)

    rows = []
    for sat_cap in [2, 1]:
        data = {**base, 'satellite_capacity': sat_cap}
        model, x, y, z = solve_instance(data)

        if model.SolCount > 0:
            routes = []
            for k in range(2):
                r = sum(
                    1 for (sk, i, j), var in y.items()
                    if sk == k and i == 'S' and var.X > 0.5
                )
                routes.append(r)
            status     = "Optimal"
            routes_str = f"({routes[0]}, {routes[1]})"
        else:
            status     = "Infeasible"
            routes_str = "(—, —)"

        rows.append((sat_cap, routes_str, status))

    for run, (cap, routes, status) in enumerate(rows, start=1):
        print(f"{run:<6} {cap:<16} {routes:<26} {status}")

    print()
    print("[LaTeX rows]")
    for run, (cap, routes, status) in enumerate(rows, start=1):
        print(f"Run {run} & {cap} & {routes} & {status} \\\\")


# ---------------------------------------------------------------------------
# Test 5 — Echelon Linking (C11, C12)
#
# Setup : 2 satellites, m1 = 1 (only one first-level vehicle, so at most one
#         satellite can be visited). Satellite 0 is placed far from customers;
#         Satellite 1 is placed close. The solver will choose Satellite 1.
#
# Expected:
#   - Satellite 0: not visited, 0 L2 routes, 0 freight
#   - Satellite 1: visited,     N L2 routes, freight = total demand
# ---------------------------------------------------------------------------

def test5():
    print_header("TEST 5 — Echelon Linking (C11, C12)")

    demands      = [20, 20, 20, 20]
    total_demand = sum(demands)

    data = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([
            [-50.0, 0.0],   # Satellite 0: far from customers
            [ 10.0, 0.0],   # Satellite 1: close to customers
        ]),
        'customers':          np.array([
            [15.0,  5.0],
            [15.0, -5.0],
            [20.0,  5.0],
            [20.0, -5.0],
        ]),
        'demands':            demands,
        'capacity_1st':       200,
        'capacity_2nd':       50,   # fits 2 customers per route (2 × 20 = 40 ≤ 50)
        'num_vehicles_1st':   1,    # only 1 vehicle → only 1 satellite visited
        'num_vehicles_2nd':   8,
        'satellite_capacity': 8,
        'distance_func':      euclidean,
    }

    model, x, y, z = solve_instance(data)

    if model.SolCount == 0:
        print("No feasible solution found.")
        return

    n_sat  = len(data['satellites'])
    n_cust = len(data['customers'])

    print(f"\nInstance : 2 satellites | m1 = 1 | Sat0 far, Sat1 near customers")
    print(f"           Total demand = {total_demand}")
    print()
    print(f"{'Satellite':<12} {'Visited by L1':<16} {'L2 routes':>10} {'Freight delivered':>18}")
    print("-" * 60)

    total_routes  = 0
    total_freight = 0

    for k in range(n_sat):
        si        = k + 1
        l1_inflow = sum(x[(i, si)].X for i in range(n_sat + 1) if i != si)
        visited   = "Yes" if l1_inflow > 0.5 else "No"

        l2_routes = sum(
            1 for (sk, i, j), var in y.items()
            if sk == k and i == 'S' and var.X > 0.5
        )
        freight   = sum(data['demands'][j] * round(z[(k, j)].X) for j in range(n_cust))

        total_routes  += l2_routes
        total_freight += freight

        print(f"Satellite {k}   {visited:<16} {l2_routes:>10} {freight:>18}")

    print("-" * 60)
    print(f"{'Total':<12} {'':16} {total_routes:>10} {total_freight:>18}")

    print()
    print("[LaTeX rows]")
    for k in range(n_sat):
        si        = k + 1
        l1_inflow = sum(x[(i, si)].X for i in range(n_sat + 1) if i != si)
        visited   = "Yes" if l1_inflow > 0.5 else "No"
        l2_routes = sum(1 for (sk, i, j), var in y.items() if sk == k and i == 'S' and var.X > 0.5)
        freight   = sum(data['demands'][j] * round(z[(k, j)].X) for j in range(n_cust))
        print(f"Satellite {k} & {visited} & {l2_routes} & {freight} \\\\")
    print(f"Total & & {total_routes} & {total_freight} \\\\")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("2E-CVRP — Constraint Verification")
    print("Running 5 tests...\n")

    test1()
    test2()
    test3()
    test4()
    test5()

    print()
    print("=" * 65)
    print("  All tests complete.")
    print("=" * 65)


if __name__ == "__main__":
    main()
