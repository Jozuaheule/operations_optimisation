"""
Two-Echelon Capacitated Vehicle Routing Problem (2E-CVRP) - FIXED
"""

import gurobipy as gp
from gurobipy import GRB
import numpy as np
import matplotlib.pyplot as plt

# ============================================================================
# PROBLEM DATA SETUP
# ============================================================================

def create_sample_problem():
    """
    Creates a small sample 2E-CVRP problem instance.
    """
    # Depot is at origin (0, 0)
    depot = np.array([0, 0])

    # Satellites
    satellites = np.array([
        [20, 20],   # Satellite 0
        [20, -20]   # Satellite 1
    ])

    # Customers
    customers = np.array([
        [25, 25],   # Customer 0 (near satellite 0)
        [22, 18],   # Customer 1 (near satellite 0)
        [18, 22],   # Customer 2 (near satellite 0)
        [15, 15],   # Customer 3 (between satellites)
        [25, -25],  # Customer 4 (near satellite 1)
        [22, -18],  # Customer 5 (near satellite 1)
        [18, -22],  # Customer 6 (near satellite 1)
        [15, -15]   # Customer 7 (between satellites)
    ])

    demands = np.array([10, 8, 7, 7, 8, 9, 8, 6])

    # Parameters
    capacity_1st_level = 80
    capacity_2nd_level = 40
    num_vehicles_1st = 10
    num_vehicles_2nd = 20
    satellite_capacity = 50 

    def euclidean_distance(p1, p2):
        return np.sqrt(np.sum((p1 - p2) ** 2))

    return {
        'depot': depot,
        'satellites': satellites,
        'customers': customers,
        'demands': demands,
        'capacity_1st': capacity_1st_level,
        'capacity_2nd': capacity_2nd_level,
        'num_vehicles_1st': num_vehicles_1st,
        'num_vehicles_2nd': num_vehicles_2nd,
        'satellite_capacity': satellite_capacity,
        'distance_func': euclidean_distance
    }


# ============================================================================
# MODEL BUILDING FUNCTIONS
# ============================================================================

def build_2echelon_vrp_model(data):
    """
    Builds the Gurobi optimization model for 2E-CVRP.
    FIXED: Uses 'S' string key for satellites in 2nd echelon to avoid index collision.
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

    n_satellites = len(satellites)
    n_customers = len(customers)

    model = gp.Model("2E-CVRP")

    # ========================================================================
    # DECISION VARIABLES
    # ========================================================================

    # --- 1st Level Variables (Depot to Satellites) ---
    x = {}
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                x[i, j] = model.addVar(vtype=GRB.INTEGER, name=f"x_{i}_{j}")

    # --- 2nd Level Variables (Satellites to Customers) ---
    # FIXED: We use 'S' to represent the satellite node in the route
    y = {}
    for k in range(n_satellites):
        # From Satellite k to Customer j
        for j in range(n_customers):
            y[k, 'S', j] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_Sat_{j}")
        
        # Between Customers
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    y[k, i, j] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_{i}_{j}")
            
            # From Customer i back to Satellite k
            y[k, i, 'S'] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_{i}_Sat")

    # --- Assignment Variables ---
    z = {}
    for k in range(n_satellites):
        for j in range(n_customers):
            z[k, j] = model.addVar(vtype=GRB.BINARY, name=f"z_{k}_{j}")

    # --- Flow Variables ---
    Q1 = {}
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                Q1[i, j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q1_{i}_{j}")

    Q2 = {}
    for k in range(n_satellites):
        for j in range(n_customers):
            Q2[k, 'S', j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_Sat_{j}")
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    Q2[k, i, j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_{i}_{j}")
            Q2[k, i, 'S'] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_{i}_Sat")

    model.update()

    # ========================================================================
    # OBJECTIVE FUNCTION
    # ========================================================================

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

        # Satellite to customer
        for j in range(n_customers):
            obj += dist_func(sat_pos, customers[j]) * y[k, 'S', j]

        # Customer to customer
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    obj += dist_func(customers[i], customers[j]) * y[k, i, j]
            
            # Customer to satellite
            obj += dist_func(customers[i], sat_pos) * y[k, i, 'S']

    model.setObjective(obj, GRB.MINIMIZE)

    # ========================================================================
    # CONSTRAINTS
    # ========================================================================

    # (1) Flow conservation at depot
    model.addConstr(gp.quicksum(x[0, j] for j in range(1, n_satellites + 1)) <= m1, "depot_outflow")

    # (2) Flow conservation at satellites (1st level)
    for k in range(1, n_satellites + 1):
        model.addConstr(
            gp.quicksum(x[i, k] for i in range(n_satellites + 1) if i != k) ==
            gp.quicksum(x[k, j] for j in range(n_satellites + 1) if j != k),
            f"sat_flow_1st_{k}"
        )

    # (3) Capacity constraints 1st level
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                model.addConstr(Q1[i, j] <= K1 * x[i, j], f"cap1_{i}_{j}")

    # (4) Single assignment
    for j in range(n_customers):
        model.addConstr(gp.quicksum(z[k, j] for k in range(n_satellites)) == 1, f"assign_{j}")

    # (5) Flow conservation at customers (2nd level)
    for k in range(n_satellites):
        for j in range(n_customers):
            # Inflow: From other customers OR from Satellite k
            inflow = gp.quicksum(y[k, i, j] for i in range(n_customers) if i != j) + y[k, 'S', j]
            # Outflow: To other customers OR to Satellite k
            outflow = gp.quicksum(y[k, j, i] for i in range(n_customers) if i != j) + y[k, j, 'S']
            
            model.addConstr(inflow == outflow, f"cust_flow_{k}_{j}")

    # (6) Satellite capacity (number of routes)
    for k in range(n_satellites):
        model.addConstr(
            gp.quicksum(y[k, 'S', j] for j in range(n_customers)) <= satellite_cap,
            f"sat_cap_{k}"
        )

    # (7) Total 2nd level vehicles
    model.addConstr(
        gp.quicksum(y[k, 'S', j] for k in range(n_satellites) for j in range(n_customers)) <= m2,
        "total_2nd_veh"
    )

    # (8) Visit if assigned
    for k in range(n_satellites):
        for j in range(n_customers):
            # If assigned to k, must be visited by a route from k
            model.addConstr(
                gp.quicksum(y[k, i, j] for i in range(n_customers) if i != j) + y[k, 'S', j] <= z[k, j],
                f"visit_assign_{k}_{j}"
            )

    # (9) 2nd level capacity
    for k in range(n_satellites):
        for j in range(n_customers):
            model.addConstr(Q2[k, 'S', j] <= K2 * y[k, 'S', j], f"cap2_S_{j}")
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    model.addConstr(Q2[k, i, j] <= K2 * y[k, i, j], f"cap2_{i}_{j}")

    # (10) Demand satisfaction flow balance
    for k in range(n_satellites):
        for j in range(n_customers):
            inflow_Q = gp.quicksum(Q2[k, i, j] for i in range(n_customers) if i != j) + Q2[k, 'S', j]
            outflow_Q = gp.quicksum(Q2[k, j, i] for i in range(n_customers) if i != j) + Q2[k, j, 'S']
            
            model.addConstr(inflow_Q - outflow_Q == demands[j] * z[k, j], f"demand_{k}_{j}")

    # (11) Link levels: Satellite demand must be met by 1st level
    for k in range(n_satellites):
        sat_idx = k + 1
        demand_at_k = gp.quicksum(demands[j] * z[k, j] for j in range(n_customers))
        
        model.addConstr(
            gp.quicksum(Q1[i, sat_idx] for i in range(n_satellites + 1) if i != sat_idx) -
            gp.quicksum(Q1[sat_idx, j] for j in range(n_satellites + 1) if j != sat_idx) == demand_at_k,
            f"sat_supply_{k}"
        )

    # (12) Link levels: Routes can only start if satellite is visited
    for k in range(n_satellites):
        sat_idx = k + 1
        model.addConstr(
            gp.quicksum(y[k, 'S', j] for j in range(n_customers)) <=
            satellite_cap * gp.quicksum(x[i, sat_idx] for i in range(n_satellites + 1) if i != sat_idx),
            f"route_avail_{k}"
        )

    return model, x, y, z


# ============================================================================
# SOLUTION VISUALIZATION
# ============================================================================

def visualize_solution(data, x_sol, y_sol, z_sol):
    """
    Visualizes the solution showing both echelons.
    Updated to handle 'S' string keys.
    """
    depot = data['depot']
    satellites = data['satellites']
    customers = data['customers']

    plt.figure(figsize=(14, 7))

    # --- 1st Level ---
    plt.subplot(1, 2, 1)
    plt.title("1st Level: Depot → Satellites", fontsize=14)
    plt.scatter(depot[0], depot[1], c='red', s=300, marker='s', label='Depot', zorder=5)
    plt.scatter(satellites[:, 0], satellites[:, 1], c='blue', s=200, marker='^', label='Satellites', zorder=5)
    
    for i, sat in enumerate(satellites):
        plt.annotate(f'S{i}', (sat[0], sat[1]), xytext=(0, 10), textcoords='offset points', ha='center')

    for (i, j), val in x_sol.items():
        if val > 0.5:
            pos_i = depot if i == 0 else satellites[i - 1]
            pos_j = depot if j == 0 else satellites[j - 1]
            plt.arrow(pos_i[0], pos_i[1], pos_j[0] - pos_i[0], pos_j[1] - pos_i[1],
                     head_width=1.5, fc='green', ec='green', alpha=0.6, length_includes_head=True)
            mid_x, mid_y = (pos_i[0] + pos_j[0]) / 2, (pos_i[1] + pos_j[1]) / 2
            plt.text(mid_x, mid_y, f'{int(val)}', bbox=dict(facecolor='white', alpha=0.8))

    plt.grid(True, alpha=0.3)
    plt.legend()

    # --- 2nd Level ---
    plt.subplot(1, 2, 2)
    plt.title("2nd Level: Satellites → Customers", fontsize=14)
    plt.scatter(satellites[:, 0], satellites[:, 1], c='blue', s=200, marker='^', label='Satellites', zorder=5)
    
    colors = ['orange', 'purple', 'brown', 'pink']
    for k in range(len(satellites)):
        assigned = [j for j in range(len(customers)) if z_sol.get((k, j), 0) > 0.5]
        if assigned:
            plt.scatter(customers[assigned, 0], customers[assigned, 1], c=colors[k % 4], s=150)

    for j, cust in enumerate(customers):
        plt.annotate(f'C{j}', (cust[0], cust[1]), ha='center', va='center')

    route_colors = ['red', 'green', 'brown', 'purple']
    for k in range(len(satellites)):
        sat_pos = satellites[k]
        color = route_colors[k % 4]
        
        # Filter routes for this satellite
        routes = [(i, j) for (sat, i, j), val in y_sol.items() if sat == k and val > 0.5]
        
        for i, j in routes:
            # Determine start point
            if i == 'S':
                p1 = sat_pos
            else:
                p1 = customers[i]
                
            # Determine end point
            if j == 'S':
                p2 = sat_pos
            else:
                p2 = customers[j]

            plt.arrow(p1[0], p1[1], p2[0] - p1[0], p2[1] - p1[1],
                     head_width=1, fc=color, ec=color, alpha=0.5, length_includes_head=True)

    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

# ============================================================================
# MAIN
# ============================================================================

def main():
    print("=" * 70)
    print("TWO-ECHELON CVRP - FIXED MODEL")
    print("=" * 70)

    data = create_sample_problem()
    model, x, y, z = build_2echelon_vrp_model(data)
    
    model.setParam('OutputFlag', 1)
    model.optimize()

    if model.Status == GRB.OPTIMAL:
        print(f"\nObjective Value: {model.ObjVal:.2f}")
        
        x_sol = {key: var.X for key, var in x.items() if var.X > 0.5}
        y_sol = {key: var.X for key, var in y.items() if var.X > 0.5}
        z_sol = {key: var.X for key, var in z.items() if var.X > 0.5}
        
        print("\n--- 2nd Level Routes ---")
        for k in range(len(data['satellites'])):
            print(f"Satellite {k}:")
            # Filter routes for sat k
            k_routes = [(i,j) for (sat, i, j), val in y_sol.items() if sat == k]
            for i, j in k_routes:
                start = "Sat" if i == 'S' else f"C{i}"
                end = "Sat" if j == 'S' else f"C{j}"
                print(f"  {start} -> {end}")

        visualize_solution(data, x_sol, y_sol, z_sol)
    else:
        print("No optimal solution found.")

if __name__ == "__main__":
    main()