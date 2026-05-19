"""
verification.py — Constraint verification tests for the 2E-CVRP model.

Each test builds a small custom instance, solves it with Gurobi, and prints
the relevant solution values for the constraints under test.
"""

import os
import numpy as np
import gurobipy as gp
import matplotlib.pyplot as plt
from gurobipy import GRB

from model import build_2echelon_vrp_model


def euclidean(p1, p2):
    return float(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2))


def solve_instance(data):
    model, x, y, z = build_2echelon_vrp_model(data)
    model.setParam('OutputFlag', 0)
    model.setParam('TimeLimit', 120)
    model.setParam('MIPGap', 0.01)
    model.optimize()
    return model, x, y, z


def save_plot(data, x_sol, y_sol, z_sol, filename):
    depot      = data['depot']
    satellites = data['satellites']
    customers  = data['customers']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))

    ax1.set_title("1st Level: Depot → Satellites", fontsize=14, fontweight='bold')
    ax1.scatter(*depot, c='red', s=300, marker='s', label='Depot',
                zorder=5, edgecolors='black')
    ax1.scatter(satellites[:, 0], satellites[:, 1], c='blue', s=200,
                marker='^', label='Satellites', zorder=5, edgecolors='black')
    for i, sat in enumerate(satellites):
        ax1.annotate(f'Sat {i}', sat, textcoords='offset points', xytext=(6, 4),
                     fontsize=9, fontweight='bold')
    for (i, j), val in x_sol.items():
        if val > 0.5:
            p1 = depot if i == 0 else satellites[i - 1]
            p2 = depot if j == 0 else satellites[j - 1]
            ax1.annotate('', xy=p2, xytext=p1,
                         arrowprops=dict(arrowstyle='->', color='green', lw=2.5))
    ax1.set_xlabel('X Coordinate')
    ax1.set_ylabel('Y Coordinate')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)

    ax2.set_title("2nd Level: Satellites → Customers", fontsize=14, fontweight='bold')
    ax2.scatter(satellites[:, 0], satellites[:, 1], c='blue', s=200,
                marker='^', label='Satellites', zorder=5, edgecolors='black')
    cust_colors  = ['orange', 'purple', 'brown', 'pink']
    route_colors = ['salmon', 'mediumpurple', 'peru', 'hotpink']
    for k, sat_pos in enumerate(satellites):
        assigned = [j for j in range(len(customers)) if z_sol.get((k, j), 0) > 0.5]
        if assigned:
            ax2.scatter(customers[assigned, 0], customers[assigned, 1],
                        c=cust_colors[k % 4], s=150,
                        label=f'Customers served by S{k}', zorder=4, edgecolors='black')
        color = route_colors[k % 4]
        for (sk, i, j), val in y_sol.items():
            if sk != k or val <= 0.5:
                continue
            p1 = sat_pos if i == 'S' else customers[i]
            p2 = sat_pos if j == 'S' else customers[j]
            ax2.annotate('', xy=p2, xytext=p1,
                         arrowprops=dict(arrowstyle='->', color=color, lw=1.8))
    for j, cust in enumerate(customers):
        ax2.annotate(f'C{j}', cust, textcoords='offset points', xytext=(0, 6),
                     ha='center', fontsize=8)
    ax2.set_xlabel('X Coordinate')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)
    plt.savefig(filename, bbox_inches='tight', dpi=150)
    plt.close()
    print(f"  Plot saved: {filename}")


# Baseline — Routing Behaviour (C5, C7)
# 1 satellite, 8 customers, unlimited capacity → pure routing.
# Sub-test A (m2=1): single vehicle TSP tour.
# Sub-test B (m2=2): two-vehicle split.
def test_baseline_routing():
    print("\nBaseline — Routing Behaviour (C5, C7)")

    cx, cy = 20.0, 20.0
    cust_pos = np.array([
        [21.0, 31.0],
        [29.0, 27.0],
        [33.0, 20.0],
        [30.0, 12.0],
        [22.0,  8.0],
        [12.0, 11.0],
        [ 8.0, 19.0],
        [13.0, 28.0],
    ])

    base_data = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([[cx, cy]]),
        'customers':          cust_pos,
        'demands':            [1] * 8,
        'capacity_1st':       9999,
        'capacity_2nd':       9999,
        'num_vehicles_1st':   1,
        'satellite_capacity': 8,
        'distance_func':      euclidean,
    }

    cases = [
        (1, "Single vehicle (TSP)", "figures/Verification/TSP_example2.pdf"),
        (2, "Two vehicles",         "figures/Verification/2Vehicles.pdf"),
    ]

    print(f"{'Variant':<24} {'m2':>4} {'Routes':>8} {'Obj':>10} {'Status':>10}")
    print("-" * 60)
    for m2, label, fig_path in cases:
        data = {**base_data, 'num_vehicles_2nd': m2}
        model, x, y, z = solve_instance(data)

        if model.SolCount == 0:
            print(f"{label:<24} {m2:>4} {'—':>8} {'—':>10} {'No solution':>10}")
            continue

        x_sol = {k: v.X for k, v in x.items() if v.X > 0.5}
        y_sol = {k: v.X for k, v in y.items() if v.X > 0.5}
        z_sol = {k: v.X for k, v in z.items() if v.X > 0.5}

        n_routes = sum(1 for key in y_sol if key[1] == 'S')
        obj      = round(model.ObjVal, 2)

        save_plot(data, x_sol, y_sol, z_sol, filename=fig_path)
        print(f"{label:<24} {m2:>4} {n_routes:>8} {obj:>10} {'Optimal':>10}")


# Test 1 — Vehicle Capacity Binding (C3, C9)
# 1 satellite, 6 customers, demand = K2 per customer → one customer per route.
def test1():
    print("\nTest 1 — Vehicle Capacity Binding (C3, C9)")

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

    active_routes = sum(1 for key, var in y.items() if key[1] == 'S' and var.X > 0.5)

    loads = []
    for j in range(n_customers):
        flow_var = model.getVarByName(f"Q2_0_S_{j}")
        if flow_var is not None and y[(0, 'S', j)].X > 0.5:
            loads.append(round(flow_var.X))

    max_load       = max(loads) if loads else 0
    routes_over_K2 = sum(1 for lo in loads if lo > K2)

    print(f"{'Customers':<14} {'Active routes':<16} {'Max load':<12} Routes > K2")
    print("-" * 56)
    print(f"{n_customers:<14} {active_routes:<16} {max_load:<12} {routes_over_K2}")


# Test 2 — Single Satellite Assignment (C4, C8)
# 2 satellites equidistant from 6 customers → each customer assigned to exactly one.
def test2():
    print("\nTest 2 — Single Satellite Assignment (C4, C8)")

    n_sat  = 2
    n_cust = 6

    data = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([
            [-10.0, 0.0],
            [ 10.0, 0.0],
        ]),
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
        'capacity_2nd':       10,
        'num_vehicles_1st':   4,
        'num_vehicles_2nd':   n_cust,
        'satellite_capacity': n_cust,
        'distance_func':      euclidean,
    }

    model, x, y, z = solve_instance(data)

    if model.SolCount == 0:
        print("No feasible solution found.")
        return

    z_matrix = [
        [round(z[(k, j)].X) for j in range(n_cust)]
        for k in range(n_sat)
    ]
    col_sums = [sum(z_matrix[k][j] for k in range(n_sat)) for j in range(n_cust)]

    col_header = "           | " + "  ".join(f"C{j}" for j in range(n_cust)) + "  | Sum"
    print(col_header)
    print("-" * len(col_header))
    for k in range(n_sat):
        vals = "  ".join(str(v) for v in z_matrix[k])
        print(f"Satellite {k} | {vals}  | {sum(z_matrix[k])}")
    print("-" * len(col_header))
    print(f"Col sum    | {'  '.join(str(s) for s in col_sums)}  |")


# Test 3 — First-Echelon Flow Conservation (C1, C2)
# 3 satellites, m1=2 → one vehicle must visit two satellites in sequence.
def test3():
    print("\nTest 3 — First-Echelon Flow Conservation (C1, C2)")

    m1 = 2

    data = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([
            [10.0,  10.0],
            [30.0,   0.0],
            [10.0, -10.0],
        ]),
        'customers':          np.array([
            [13.0,  13.0], [12.0,  11.0],
            [33.0,   2.0], [33.0,  -2.0],
            [13.0, -13.0], [12.0, -11.0],
        ]),
        'demands':            [20] * 6,
        'capacity_1st':       100,
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
    depot_out = sum(x[(0, j)].X for j in range(1, n_sat + 1))
    depot_in  = sum(x[(j, 0)].X for j in range(1, n_sat + 1))

    print(f"{'Node':<14} {'Inflow':>8} {'Outflow':>8} {'Balance':>9}")
    print("-" * 44)
    print(f"{'Depot':<14} {round(depot_in):>8} {round(depot_out):>8} {'—':>9}")
    for k in range(n_sat):
        si      = k + 1
        inflow  = sum(x[(i, si)].X for i in range(n_sat + 1) if i != si)
        outflow = sum(x[(si, j)].X for j in range(n_sat + 1) if j != si)
        print(f"{'Satellite ' + str(k):<14} {round(inflow):>8} {round(outflow):>8} {round(inflow - outflow):>9}")


# Test 4 — Satellite Capacity Limit (C6)
# sat_cap=2 → feasible; sat_cap=1 → infeasible.
def test4():
    print("\nTest 4 — Satellite Capacity Limit (C6)")

    K2 = 20

    base = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([
            [-10.0, 0.0],
            [ 10.0, 0.0],
        ]),
        'customers':          np.array([
            [-15.0,  5.0],
            [-15.0, -5.0],
            [ 15.0,  5.0],
            [ 15.0, -5.0],
        ]),
        'demands':            [K2] * 4,
        'capacity_1st':       200,
        'capacity_2nd':       K2,
        'num_vehicles_1st':   2,
        'num_vehicles_2nd':   4,
        'distance_func':      euclidean,
    }

    print(f"{'Sat. capacity':<16} {'Routes (S0, S1)':<20} Status")
    print("-" * 50)
    for sat_cap in [2, 1]:
        data = {**base, 'satellite_capacity': sat_cap}
        model, x, y, z = solve_instance(data)

        if model.SolCount > 0:
            routes = [
                sum(1 for (sk, i, j), var in y.items() if sk == k and i == 'S' and var.X > 0.5)
                for k in range(2)
            ]
            print(f"{sat_cap:<16} ({routes[0]}, {routes[1]}){'':12} Optimal")
        else:
            print(f"{sat_cap:<16} {'—':<20} Infeasible")


# Test 5 — Echelon Linking (C11, C12)
# 2 satellites, m1=1 → only Satellite 0 (near) is visited; Satellite 1 (far) is skipped.
def test5():
    print("\nTest 5 — Echelon Linking (C11, C12)")

    demands = [20, 20, 20, 20]

    data = {
        'depot':              np.array([0.0, 0.0]),
        'satellites':         np.array([
            [ 10.0, 0.0],   # Satellite 0: near
            [-50.0, 0.0],   # Satellite 1: far
        ]),
        'customers':          np.array([
            [15.0,  5.0],
            [15.0, -5.0],
            [20.0,  5.0],
            [20.0, -5.0],
        ]),
        'demands':            demands,
        'capacity_1st':       200,
        'capacity_2nd':       50,
        'num_vehicles_1st':   1,
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

    print(f"{'Satellite':<12} {'Visited':>8} {'Routes':>8} {'Freight':>10}")
    print("-" * 42)

    total_routes  = 0
    total_freight = 0
    for k in range(n_sat):
        si        = k + 1
        visited   = "Yes" if sum(x[(i, si)].X for i in range(n_sat + 1) if i != si) > 0.5 else "No"
        l2_routes = sum(1 for (sk, i, j), var in y.items() if sk == k and i == 'S' and var.X > 0.5)
        freight   = sum(demands[j] * round(z[(k, j)].X) for j in range(n_cust))
        total_routes  += l2_routes
        total_freight += freight
        print(f"Satellite {k}   {visited:>8} {l2_routes:>8} {freight:>10}")

    print("-" * 42)
    print(f"{'Total':<12} {'':>8} {total_routes:>8} {total_freight:>10}")


def main():
    test_baseline_routing()
    test1()
    test2()
    test3()
    test4()
    test5()


if __name__ == "__main__":
    main()
