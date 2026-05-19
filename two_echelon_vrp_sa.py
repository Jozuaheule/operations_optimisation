"""
Two-Echelon Capacitated Vehicle Routing Problem (2E-CVRP) Sample Implementation
(Modified for Sensitivity Analysis)
"""
import argparse
import json
import time
import gurobipy as gp
from gurobipy import GRB
from dataset_processer import parse_file
import numpy as np

# ============================================================================
# PROBLEM DATA SETUP
# ============================================================================
def load_problem_data(filepath, type, level1, level2):
    """
    Loads a 2E-CVRP problem instance from a file and applies modifications.

    Args:
        filepath (str): Path to the dataset file.
        capacity_multiplier (float): Multiplier to adjust vehicle capacities.

    Returns:
        dict: Problem data.
    """
    data = parse_file(filepath)
    

    if type == 'vehicle_capacity':
    
        # Apply capacity multiplier for sensitivity analysis
        if 'capacity_1st' in data:
            data['capacity_1st'] = np.round(np.array(data['capacity_1st']) * level1).astype(int)
        if 'capacity_2nd' in data:
            data['capacity_2nd'] = np.round(np.array(data['capacity_2nd']) * level2).astype(int)

    if type == 'demand_capacity':
    
        # Apply capacity multiplier for sensitivity analysis
        if 'demands' in data:
            customers = np.array(data['demands'])
            data['demands'] = (customers * level1).round().astype(int)

    if type == 'vehicle_amount':
    
        # Apply TIGHT BASELINE (e.g., 25% of original) then add DISCRETE OFFSET
        if 'num_vehicles_1st' in data:
            tight_base_1st = max(1, int(np.round(data['num_vehicles_1st'] * 0.25)))
            data['num_vehicles_1st'] = max(1, int(tight_base_1st + level1))
        if 'num_vehicles_2nd' in data:
            tight_base_2nd = max(1, int(np.round(data['num_vehicles_2nd'] * 0.25)))
            data['num_vehicles_2nd'] = max(1, int(tight_base_2nd + level2))
    
    if type == 'satellite_capacity':
        # Apply DISCRETE OFFSET to the satellite route capacity
        if 'satellite_capacity' in data:
            data['satellite_capacity'] = max(1, int(data['satellite_capacity'] + level1))
            
    return data

# ============================================================================
# MODEL BUILDING FUNCTIONS
# ============================================================================
def build_2echelon_vrp_model(data):
    """
    Builds the Gurobi optimization model for 2E-CVRP.
    Returns the model and dictionaries of variables and key constraints.
    """
    # Extract data
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

    n_satellites = len(satellites)
    n_customers = len(customers)

    model = gp.Model("2E-CVRP")
    
    # --- Decision Variables ---
    x, y, z = {}, {}, {}
    # 1st Level
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                x[i, j] = model.addVar(vtype=GRB.INTEGER, name=f"x_{i}_{j}")
    # 2nd Level
    for k in range(n_satellites):
        for j in range(n_customers):
            y[k, 'S', j] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_S_{j}")
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    y[k, i, j] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_{i}_{j}")
            y[k, i, 'S'] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_{i}_S")
    # Assignment
    for k in range(n_satellites):
        for j in range(n_customers):
            z[k, j] = model.addVar(vtype=GRB.BINARY, name=f"z_{k}_{j}")

    # --- Flow Variables ---
    Q1, Q2 = {}, {}
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                Q1[i, j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q1_{i}_{j}")
    for k in range(n_satellites):
        for j in range(n_customers):
            Q2[k, 'S', j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_S_{j}")
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    Q2[k, i, j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_{i}_{j}")
            Q2[k, i, 'S'] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_{i}_S")

    model.update()

    # --- Objective Function ---
    obj = gp.LinExpr()
    # 1st level costs
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                pos_i = depot if i == 0 else satellites[i - 1]
                pos_j = depot if j == 0 else satellites[j - 1]
                obj += dist_func(pos_i, pos_j) * x[i, j]

    # 2nd level costs
    for k in range(n_satellites):
        sat_pos = satellites[k]
        for j in range(n_customers):
            obj += dist_func(sat_pos, customers[j]) * y[k, 'S', j]
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    obj += dist_func(customers[i], customers[j]) * y[k, i, j]
            obj += dist_func(customers[i], sat_pos) * y[k, i, 'S']
    model.setObjective(obj, GRB.MINIMIZE)

    # --- Constraints ---
    constrs = {}
    # (1) Depot outflow
    constrs["depot_outflow"] = model.addConstr(gp.quicksum(x[0, j] for j in range(1, n_satellites + 1)) <= m1)
    
    # (4) Customer assignment
    constrs["customer_assignment"] = {}
    for j in range(n_customers):
        constrs["customer_assignment"][j] = model.addConstr(gp.quicksum(z[k, j] for k in range(n_satellites)) == 1)

    # (9) 2nd level capacity constraints
    constrs["cap2"] = {}
    for k in range(n_satellites):
        for j in range(n_customers):
            constrs["cap2"][k, 'S', j] = model.addConstr(Q2[k, 'S', j] <= K2 * y[k, 'S', j])
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    constrs["cap2"][k, i, j] = model.addConstr(Q2[k, i, j] <= K2 * y[k, i, j])

    # (10) Demand satisfaction
    constrs["demand_satisfaction"] = {}
    for k in range(n_satellites):
        for j in range(n_customers):
            inflow = gp.quicksum(Q2[k, i, j] for i in range(n_customers) if i != j) + Q2[k, 'S', j]
            outflow = gp.quicksum(Q2[k, j, i] for i in range(n_customers) if i != j) + Q2[k, j, 'S']
            constrs["demand_satisfaction"][k, j] = model.addConstr(inflow - outflow == demands[j] * z[k, j])

    # (Other constraints - simplified for brevity, full logic remains similar)
    for k in range(1, n_satellites + 1):
        model.addConstr(gp.quicksum(x[i, k] for i in range(n_satellites + 1) if i != k) == gp.quicksum(x[k, j] for j in range(n_satellites + 1) if j != k))
    
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                model.addConstr(Q1[i, j] <= K1 * x[i, j])
    
    for k in range(n_satellites):
        for j in range(n_customers):
            inflow = gp.quicksum(y[k, i, j] for i in range(n_customers) if i != j) + y[k, 'S', j]
            outflow = gp.quicksum(y[k, j, i] for i in range(n_customers) if i != j) + y[k, j, 'S']
            model.addConstr(inflow == outflow)
    
    for k in range(n_satellites):
        model.addConstr(gp.quicksum(y[k, 'S', j] for j in range(n_customers)) <= satellite_cap)
    model.addConstr(gp.quicksum(y[k, 'S', j] for k in range(n_satellites) for j in range(n_customers)) <= m2)
    
    for k in range(n_satellites):
        for j in range(n_customers):
            model.addConstr(gp.quicksum(y[k, i, j] for i in range(n_customers) if i != j) + y[k, 'S', j] <= z[k, j])
    
    for k in range(n_satellites):
        sat_idx = k + 1
        demand_at_k = gp.quicksum(demands[j] * z[k, j] for j in range(n_customers))
        model.addConstr(gp.quicksum(Q1[i, sat_idx] for i in range(n_satellites + 1) if i != sat_idx) - gp.quicksum(Q1[sat_idx, j] for j in range(n_satellites + 1) if j != sat_idx) == demand_at_k)
    
    for k in range(n_satellites):
        sat_idx = k + 1
        model.addConstr(gp.quicksum(y[k, 'S', j] for j in range(n_customers)) <= satellite_cap * gp.quicksum(x[i, sat_idx] for i in range(n_satellites + 1) if i != sat_idx))

    return model    #, x, y, z, constrs

# ============================================================================
# MAIN EXECUTION
# ============================================================================


"""
Main function to load data, solve the 2E-CVRP, and output results as JSON.
"""

parser = argparse.ArgumentParser(description="Solve a 2E-CVRP instance.")
parser.add_argument("--dataset", required=True, help="Path to the dataset file.")
parser.add_argument("--param_name", required=True, default="baseline", help="Parameter for sensitivity input")
parser.add_argument("--sensitivity_multiplier1", required = True, type=float, default=1.0, help="Multiplier for vehicle capacity.")
parser.add_argument("--sensitivity_multiplier2", required = True, type=float, default=1.0, help="Multiplier for vehicle capacity.")
args = parser.parse_args()


results = {
    "dataset": None,
    "parameter_modified": args.param_name,
    "parameter_value1": args.sensitivity_multiplier1,
    "parameter_value2": args.sensitivity_multiplier2,
    "total_cost": None,
    "execution_time": None,
    "is_feasible": False,
    "error": None
    
}

try:
    # Load data
    data = load_problem_data(args.dataset, args.param_name, args.sensitivity_multiplier1, args.sensitivity_multiplier2)

    # Build model
    model = build_2echelon_vrp_model(data)
    
    # Suppress Gurobi output
    model.setParam('OutputFlag', 0)
    model.setParam("LogToConsole", 0)
    # model.setParam("LogToConsole", 1)
    model.setParam('TimeLimit', 300)  # 5 minute time limit
    model.setParam('MIPGap', 0.1)    # 10% optimality gap tolerance

    # Solve
    start_time = time.time()
    model.optimize()
    end_time = time.time()

    results["execution_time"] = end_time - start_time

    # # Check solution status
    # if model.Status in [GRB.OPTIMAL, GRB.TIME_LIMIT]:
    results["is_feasible"] = True
    results["total_cost"] = model.ObjVal

    # else:
    #     results["error"] = f"No solution found. Status code: {model.Status}"

except Exception as e:
    results["error"] = str(e)

# Print results as a single JSON string
print(json.dumps(results))



