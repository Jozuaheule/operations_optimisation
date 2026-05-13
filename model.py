import numpy as np
import gurobipy as gp
from gurobipy import GRB
from data import parse_file


def load_problem_data(filepath, param_type=None, level1=1.0, level2=1.0):
    """Load a 2E-CVRP instance from file and optionally perturb one parameter group.

    param_type options:
      'vehicle_capacity' — scale capacity_1st by level1, capacity_2nd by level2
      'demand_capacity'  — scale all customer demands by level1
      'vehicle_amount'   — scale num_vehicles_1st by level1, num_vehicles_2nd by level2
    """
    data = parse_file(filepath)

    if param_type == 'vehicle_capacity':
        data['capacity_1st'] = int(round(data['capacity_1st'] * level1))
        data['capacity_2nd'] = int(round(data['capacity_2nd'] * level2))
    elif param_type == 'demand_capacity':
        data['demands'] = [int(round(d * level1)) for d in data['demands']]
    elif param_type == 'vehicle_amount':
        data['num_vehicles_1st'] = int(round(data['num_vehicles_1st'] * level1))
        data['num_vehicles_2nd'] = int(round(data['num_vehicles_2nd'] * level2))

    return data


def build_2echelon_vrp_model(data):
    """Build and return the Gurobi model for 2E-CVRP together with variable dicts.

    Returns: (model, x, y, z)
      x[i,j]      — integer arc variables for the 1st echelon (depot=0, satellites=1..n_sat)
      y[k,'S',j]  — binary arc from satellite k to customer j
      y[k,i,j]    — binary arc between customers i and j under satellite k
      y[k,i,'S']  — binary arc from customer i back to satellite k
      z[k,j]      — binary assignment: customer j served by satellite k
    """
    depot = data['depot']
    satellites = data['satellites']
    customers = data['customers']
    demands = data['demands']
    K1 = data['capacity_1st']
    K2 = data['capacity_2nd']
    m1 = data['num_vehicles_1st']
    m2 = data['num_vehicles_2nd']
    satellite_cap = data['satellite_capacity']
    dist_func = data['distance_func']

    n_sat = len(satellites)
    n_cust = len(customers)

    model = gp.Model("2E-CVRP")

    # --- 1st-level arc variables (depot + satellites) ---
    x = {
        (i, j): model.addVar(vtype=GRB.INTEGER, name=f"x_{i}_{j}")
        for i in range(n_sat + 1)
        for j in range(n_sat + 1)
        if i != j
    }

    # --- 2nd-level arc variables per satellite ---
    y = {}
    for k in range(n_sat):
        for j in range(n_cust):
            y[k, 'S', j] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_S_{j}")
        for i in range(n_cust):
            for j in range(n_cust):
                if i != j:
                    y[k, i, j] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_{i}_{j}")
            y[k, i, 'S'] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_{i}_S")

    # --- Assignment variables ---
    z = {
        (k, j): model.addVar(vtype=GRB.BINARY, name=f"z_{k}_{j}")
        for k in range(n_sat)
        for j in range(n_cust)
    }

    # --- Flow variables ---
    Q1 = {
        (i, j): model.addVar(vtype=GRB.CONTINUOUS, name=f"Q1_{i}_{j}")
        for i in range(n_sat + 1)
        for j in range(n_sat + 1)
        if i != j
    }
    Q2 = {}
    for k in range(n_sat):
        for j in range(n_cust):
            Q2[k, 'S', j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_S_{j}")
        for i in range(n_cust):
            for j in range(n_cust):
                if i != j:
                    Q2[k, i, j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_{i}_{j}")
            Q2[k, i, 'S'] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_{i}_S")

    model.update()

    # --- Objective: minimise total travel cost ---
    obj = gp.LinExpr()
    for (i, j) in x:
        pos_i = depot if i == 0 else satellites[i - 1]
        pos_j = depot if j == 0 else satellites[j - 1]
        obj += dist_func(pos_i, pos_j) * x[i, j]
    for k in range(n_sat):
        sat_pos = satellites[k]
        for j in range(n_cust):
            obj += dist_func(sat_pos, customers[j]) * y[k, 'S', j]
        for i in range(n_cust):
            for j in range(n_cust):
                if i != j:
                    obj += dist_func(customers[i], customers[j]) * y[k, i, j]
            obj += dist_func(customers[i], sat_pos) * y[k, i, 'S']
    model.setObjective(obj, GRB.MINIMIZE)

    # --- Constraints ---

    # (1) Depot outflow bounded by available 1st-level vehicles
    model.addConstr(gp.quicksum(x[0, j] for j in range(1, n_sat + 1)) <= m1, "depot_outflow")

    # (2) Flow conservation at 1st-level satellites
    for k in range(1, n_sat + 1):
        model.addConstr(
            gp.quicksum(x[i, k] for i in range(n_sat + 1) if i != k) ==
            gp.quicksum(x[k, j] for j in range(n_sat + 1) if j != k),
            f"sat_flow_1st_{k}",
        )

    # (3) 1st-level arc capacity
    for (i, j) in x:
        model.addConstr(Q1[i, j] <= K1 * x[i, j], f"cap1_{i}_{j}")

    # (4) Each customer assigned to exactly one satellite
    for j in range(n_cust):
        model.addConstr(gp.quicksum(z[k, j] for k in range(n_sat)) == 1, f"assign_{j}")

    # (5) Flow conservation at 2nd-level customers
    for k in range(n_sat):
        for j in range(n_cust):
            inflow  = gp.quicksum(y[k, i, j] for i in range(n_cust) if i != j) + y[k, 'S', j]
            outflow = gp.quicksum(y[k, j, i] for i in range(n_cust) if i != j) + y[k, j, 'S']
            model.addConstr(inflow == outflow, f"cust_flow_{k}_{j}")

    # (6) Satellite route capacity (max routes per satellite)
    for k in range(n_sat):
        model.addConstr(
            gp.quicksum(y[k, 'S', j] for j in range(n_cust)) <= satellite_cap,
            f"sat_cap_{k}",
        )

    # (7) Total 2nd-level vehicles
    model.addConstr(
        gp.quicksum(y[k, 'S', j] for k in range(n_sat) for j in range(n_cust)) <= m2,
        "total_2nd_veh",
    )

    # (8) Customers can only be visited by their assigned satellite
    for k in range(n_sat):
        for j in range(n_cust):
            model.addConstr(
                gp.quicksum(y[k, i, j] for i in range(n_cust) if i != j) + y[k, 'S', j] <= z[k, j],
                f"visit_assign_{k}_{j}",
            )

    # (9) 2nd-level arc capacity
    for k in range(n_sat):
        for j in range(n_cust):
            model.addConstr(Q2[k, 'S', j] <= K2 * y[k, 'S', j], f"cap2_S_{k}_{j}")
        for i in range(n_cust):
            for j in range(n_cust):
                if i != j:
                    model.addConstr(Q2[k, i, j] <= K2 * y[k, i, j], f"cap2_{k}_{i}_{j}")

    # (10) Demand satisfaction: flow balance at each customer
    for k in range(n_sat):
        for j in range(n_cust):
            inflow_Q  = gp.quicksum(Q2[k, i, j] for i in range(n_cust) if i != j) + Q2[k, 'S', j]
            outflow_Q = gp.quicksum(Q2[k, j, i] for i in range(n_cust) if i != j) + Q2[k, j, 'S']
            model.addConstr(inflow_Q - outflow_Q == demands[j] * z[k, j], f"demand_{k}_{j}")

    # (11) 1st-level supply to each satellite must meet its assigned demand
    for k in range(n_sat):
        sat_idx = k + 1
        demand_at_k = gp.quicksum(demands[j] * z[k, j] for j in range(n_cust))
        model.addConstr(
            gp.quicksum(Q1[i, sat_idx] for i in range(n_sat + 1) if i != sat_idx) -
            gp.quicksum(Q1[sat_idx, j] for j in range(n_sat + 1) if j != sat_idx) == demand_at_k,
            f"sat_supply_{k}",
        )

    # (12) 2nd-level routes can only start from a satellite that is visited by 1st level
    for k in range(n_sat):
        sat_idx = k + 1
        model.addConstr(
            gp.quicksum(y[k, 'S', j] for j in range(n_cust)) <=
            satellite_cap * gp.quicksum(x[i, sat_idx] for i in range(n_sat + 1) if i != sat_idx),
            f"route_avail_{k}",
        )

    return model, x, y, z
