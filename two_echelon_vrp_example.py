"""
Two-Echelon Capacitated Vehicle Routing Problem (2E-CVRP) Sample Implementation

This implementation is based on the paper:
"The Two-Echelon Capacitated Vehicle Routing Problem"
by Gonzalez-Feliu, Perboli, Tadei, and Vigo (2008)

Problem Description:
- Freight is delivered from a depot to customers through intermediate satellites
- Two transportation levels:
  * 1st level: Depot → Satellites (larger vehicles)
  * 2nd level: Satellites → Customers (smaller vehicles)
- Objective: Minimize total transportation cost
- Constraints: Vehicle capacities, satellite capacities
"""
import re
import gurobipy as gp
from gurobipy import GRB
import numpy as np
import matplotlib.pyplot as plt
from data_processer import parse_file

# ============================================================================
# PROBLEM DATA SETUP
# ============================================================================

# Specify if sample or not
sample = False

# Specify folder to use for verification
folder = "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/folder verification"
dataset = 4   # datasets 2,3 have same structure, dataset 4 is a bit different, dataset 1 works with matrix
filepath = "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/Dataset2_example.dat"


def create_sample_problem():
    """
    Creates a small sample 2E-CVRP problem instance.

    Returns:
        dict: Problem data including coordinates, demands, capacities, etc.
    """
    if sample:
        # Define coordinates for depot, satellites, and customers
        # Depot is at origin (0, 0)
        depot = np.array([0, 0])

        # Satellites are positioned around the customer area
        # In practice, these would be strategically located facilities
        satellites = np.array([
            [20, 20],   # Satellite 0
            [20, -20]   # Satellite 1
        ])

        # Customers are distributed in clusters
        # Each cluster is roughly near one satellite
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

        # Customer demands (amount of goods each customer requires)
        cust_demands = np.array([10, 8, 12, 7, 11, 9, 10, 6])

        # Vehicle capacities
        # 1st level vehicles are larger (e.g., trucks)
        capacity_1st_level = 50
        # 2nd level vehicles are smaller (e.g., vans for city delivery)
        capacity_2nd_level = 20

        # Number of available vehicles
        num_vehicles_1st = 2  # Vehicles at depot
        num_vehicles_2nd = 4  # Total vehicles for 2nd level distribution

        # Satellite capacity (max number of 2nd-level routes from each satellite)
        satellite_capacity = 3  # Each satellite can handle up to 3 delivery routes

        def euclidean_distance(p1, p2):
            return np.sqrt(np.sum((p1 - p2) ** 2))

        return {
        'depot': depot,
        'satellites': satellites,
        'customers': customers,
        'demands': cust_demands,
        'capacity_1st': capacity_1st_level,
        'capacity_2nd': capacity_2nd_level,
        'num_vehicles_1st': num_vehicles_1st,
        'num_vehicles_2nd': num_vehicles_2nd,
        'satellite_capacity': satellite_capacity,
        'distance_func': euclidean_distance
    }

    else:

        data = parse_file(filepath)
        return data

    
    # Calculate distance matrix using Euclidean distance
    # This represents travel costs (simplified as distances)


# ============================================================================
# MODEL BUILDING FUNCTIONS
# ============================================================================

def build_2echelon_vrp_model(data):
    """
    Builds the Gurobi optimization model for 2E-CVRP.

    This is a simplified version of the mathematical model presented in the paper.
    The full model includes flow variables and more complex constraints.

    Args:
        data (dict): Problem data from create_sample_problem()

    Returns:
        gp.Model: Configured Gurobi model
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

    # Problem dimensions
    n_satellites = len(satellites)
    n_customers = len(customers)

    # Create Gurobi model
    model = gp.Model("2E-CVRP")

    # ========================================================================
    # DECISION VARIABLES
    # ========================================================================

    # --- 1st Level Variables (Depot to Satellites) ---
    # x[i,j] = number of 1st-level vehicles traveling from node i to node j
    # Nodes: depot (0), satellites (1 to n_satellites)
    x = {}
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                x[i, j] = model.addVar(vtype=GRB.INTEGER, name=f"x_{i}_{j}")

    # --- 2nd Level Variables (Satellites to Customers) ---
    # y[k,i,j] = 1 if a 2nd-level vehicle from satellite k travels from i to j
    # k = satellite index, i,j can be satellite or customer
    y = {}
    for k in range(n_satellites):
        # Arcs from satellite k to customers
        for j in range(n_customers):
            y[k, 'S', j] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_{k}_{j}")
        # Arcs between customers (for routes starting from satellite k)
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    y[k, i, j] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_{i}_{j}")
            # Arcs from customers back to satellite k
            y[k, i, 'S'] = model.addVar(vtype=GRB.BINARY, name=f"y_{k}_{i}_{k}")

    # --- Assignment Variables ---
    # z[k,j] = 1 if customer j is served by satellite k
    z = {}
    for k in range(n_satellites):
        for j in range(n_customers):
            z[k, j] = model.addVar(vtype=GRB.BINARY, name=f"z_{k}_{j}")

    # --- Flow Variables (simplified for this example) ---
    # These track the amount of goods flowing on each arc
    # Q1[i,j] = flow on 1st-level arc (i,j)
    Q1 = {}
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                Q1[i, j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q1_{i}_{j}")

    # Q2[k,i,j] = flow on 2nd-level arc (i,j) from satellite k
    Q2 = {}
    for k in range(n_satellites):
        for j in range(n_customers):
            Q2[k, 'S', j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_{k}_{j}")
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    Q2[k, i, j] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_{i}_{j}")
            Q2[k, i, 'S'] = model.addVar(vtype=GRB.CONTINUOUS, name=f"Q2_{k}_{i}_{k}")

    model.update()

    # ========================================================================
    # OBJECTIVE FUNCTION
    # ========================================================================

    # Minimize total transportation cost (simplified as total distance)
    obj = gp.LinExpr()

    # 1st level costs: depot-satellite and satellite-satellite arcs
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                if i == 0:  # From depot
                    pos_i = depot
                else:
                    pos_i = satellites[i - 1]

                if j == 0:  # To depot
                    pos_j = depot
                else:
                    pos_j = satellites[j - 1]

                cost = dist_func(pos_i, pos_j)
                obj += cost * x[i, j]

    # 2nd level costs: satellite-customer and customer-customer arcs
    for k in range(n_satellites):
        sat_pos = satellites[k]

        # Satellite to customer
        for j in range(n_customers):
            cost = dist_func(sat_pos, customers[j])
            obj += cost * y[k, 'S', j]

        # Customer to customer
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    cost = dist_func(customers[i], customers[j])
                    obj += cost * y[k, i, j]

            # Customer back to satellite
            cost = dist_func(customers[i], sat_pos)
            obj += cost * y[k, i, 'S']

    model.setObjective(obj, GRB.MINIMIZE)

    # ========================================================================
    # CONSTRAINTS
    # ========================================================================

    # --- 1st Level Constraints ---

    # (1) Flow conservation at depot: vehicles leave and return
    model.addConstr(
        gp.quicksum(x[0, j] for j in range(1, n_satellites + 1)) <= m1,
        name="depot_outflow"
    )

    # (2) Flow conservation at satellites
    for k in range(1, n_satellites + 1):
        model.addConstr(
            gp.quicksum(x[i, k] for i in range(n_satellites + 1) if i != k) ==
            gp.quicksum(x[k, j] for j in range(n_satellites + 1) if j != k),
            name=f"satellite_flow_{k}"
        )

    # (3) Capacity constraints for 1st level (using flow variables)
    # Note: total_demand = sum(demands) could be used for validation
    for i in range(n_satellites + 1):
        for j in range(n_satellites + 1):
            if i != j:
                model.addConstr(Q1[i, j] <= K1 * x[i, j], name=f"cap1_{i}_{j}")

    # --- 2nd Level Constraints ---

    # (4) Each customer must be assigned to exactly one satellite
    for j in range(n_customers):
        model.addConstr(
            gp.quicksum(z[k, j] for k in range(n_satellites)) == 1,
            name=f"customer_assignment_{j}"
        )

    # (5) Flow conservation at customers for each satellite
    for k in range(n_satellites):
        for j in range(n_customers):
            # Inflow: From other customers OR from Satellite k
            inflow = gp.quicksum(y[k, i, j] for i in range(n_customers) if i != j) + y[k, 'S', j]
            # Outflow: To other customers OR to Satellite k
            outflow = gp.quicksum(y[k, j, i] for i in range(n_customers) if i != j) + y[k, j, 'S']
            
            model.addConstr(inflow == outflow, f"cust_flow_{k}_{j}")

    # (6) Satellite capacity: limit number of routes from each satellite
    for k in range(n_satellites):
        model.addConstr(
            gp.quicksum(y[k, 'S', j] for j in range(n_customers)) <= satellite_cap,
            name=f"satellite_capacity_{k}"
        )

    # (7) Total 2nd level vehicles constraint
    model.addConstr(
        gp.quicksum(y[k, 'S', j] for k in range(n_satellites) for j in range(n_customers)) <= m2,
        name="total_2nd_vehicles"
    )

    # (8) Customers can only be visited if assigned to satellite
    for k in range(n_satellites):
        for j in range(n_customers):
            model.addConstr(
                gp.quicksum(y[k, i, j] for i in range(n_customers) if i != j) + y[k, 'S', j] <= z[k, j],
                name=f"visit_if_assigned_{k}_{j}"
            )

    # (9) 2nd level capacity constraints (using flow variables)
    for k in range(n_satellites):
        for j in range(n_customers):
            model.addConstr(
                Q2[k, 'S', j] <= K2 * y[k, 'S', j],
                name=f"cap2_{k}_{'S'}_{j}"
            )
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    model.addConstr(
                        Q2[k, i, j] <= K2 * y[k, i, j],
                        name=f"cap2_{k}_{i}_{j}"
                    )

    # (10) Flow balance for demand satisfaction
    # Each customer receives its demand through the assigned satellite
    for k in range(n_satellites):
        for j in range(n_customers):
            # Flow in minus flow out equals demand if assigned to this satellite
            inflow = gp.quicksum(Q2[k, i, j] for i in range(n_customers) if i != j) + Q2[k, 'S', j]
            outflow = gp.quicksum(Q2[k, j, i] for i in range(n_customers) if i != j) + Q2[k, j, 'S']

            model.addConstr(
                inflow - outflow == demands[j] * z[k, j],
                name=f"demand_satisfaction_{k}_{j}"
            )

    # (11) Link 1st and 2nd levels: satellites receive what customers need
    for k in range(n_satellites):
        # Demand flowing to satellite k from depot
        sat_idx = k + 1  # Satellite index in 1st level (depot is 0)
        demand_at_k = gp.quicksum(demands[j] * z[k, j] for j in range(n_customers))

        # Flow into satellite k must equal total demand assigned to it
        model.addConstr(
            gp.quicksum(Q1[i, sat_idx] for i in range(n_satellites + 1) if i != sat_idx) -
            gp.quicksum(Q1[sat_idx, j] for j in range(n_satellites + 1) if j != sat_idx) == demand_at_k,
            name=f"satellite_demand_{k}"
        )

    # (12) A 2nd-level route can start from satellite only if 1st-level serves it
    for k in range(n_satellites):
        sat_idx = k + 1
        model.addConstr(
            gp.quicksum(y[k, 'S', j] for j in range(n_customers)) <=
            satellite_cap * gp.quicksum(x[i, sat_idx] for i in range(n_satellites + 1) if i != sat_idx),
            name=f"link_levels_{k}"
        )

    return model, x, y, z


# ============================================================================
# SOLUTION VISUALIZATION
# ============================================================================

def visualize_solution(data, x_sol, y_sol, z_sol):
    """
    Visualizes the solution showing both echelons of routes.

    Args:
        data (dict): Problem data
        x_sol (dict): Solution values for 1st level variables
        y_sol (dict): Solution values for 2nd level variables
        z_sol (dict): Solution values for assignment variables
    """

    depot = data['depot']
    satellites = data['satellites']
    customers = data['customers']

    plt.figure(figsize=(14, 7))

    # --- Plot 1st Level (Depot to Satellites) ---
    plt.subplot(1, 2, 1)
    plt.title("1st Level: Depot → Satellites", fontsize=14, fontweight='bold')

    # Plot depot
    plt.scatter(depot[0], depot[1], c='red', s=300, marker='s',
                label='Depot', zorder=5, edgecolors='black', linewidths=2)

    # Plot satellites
    plt.scatter(satellites[:, 0], satellites[:, 1], c='blue', s=200,
                marker='^', label='Satellites', zorder=5, edgecolors='black', linewidths=2)

    # Annotate satellites
    for i, sat in enumerate(satellites):
        plt.annotate(f'S{i}', (sat[0], sat[1]), fontsize=12, fontweight='bold',
                    ha='center', va='center')

    # Draw 1st level routes
    for (i, j), val in x_sol.items():
        if val > 0.5:  # If route is used
            if i == 0:
                pos_i = depot
            else:
                pos_i = satellites[i - 1]

            if j == 0:
                pos_j = depot
            else:
                pos_j = satellites[j - 1]

            plt.arrow(pos_i[0], pos_i[1],
                     pos_j[0] - pos_i[0], pos_j[1] - pos_i[1],
                     head_width=1.5, head_length=1, fc='green', ec='green',
                     alpha=0.6, linewidth=2, length_includes_head=True)

            # Add label showing number of vehicles
            mid_x, mid_y = (pos_i[0] + pos_j[0]) / 2, (pos_i[1] + pos_j[1]) / 2
            plt.text(mid_x, mid_y, f'{int(val)}', fontsize=10,
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    plt.grid(True, alpha=0.3)
    plt.legend(loc='best')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    # --- Plot 2nd Level (Satellites to Customers) ---
    plt.subplot(1, 2, 2)
    plt.title("2nd Level: Satellites → Customers", fontsize=14, fontweight='bold')

    # Plot satellites
    plt.scatter(satellites[:, 0], satellites[:, 1], c='blue', s=200,
                marker='^', label='Satellites', zorder=5, edgecolors='black', linewidths=2)

    # Plot customers with colors based on assignment
    colors = ['orange', 'purple', 'brown', 'pink']
    for k in range(len(satellites)):
        assigned_customers = [j for j in range(len(customers))
                            if z_sol.get((k, j), 0) > 0.5]
        if assigned_customers:
            cust_coords = customers[assigned_customers]
            plt.scatter(cust_coords[:, 0], cust_coords[:, 1],
                       c=colors[k % len(colors)], s=150,
                       label=f'Customers served by S{k}',
                       zorder=4, edgecolors='black', linewidths=1.5)

    # Annotate customers
    for j, cust in enumerate(customers):
        plt.annotate(f'C{j}', (cust[0], cust[1]), fontsize=10,
                    ha='center', va='center')

    # Draw 2nd level routes
    route_colors = ['red', 'green', 'brown', 'purple']
    for k in range(len(satellites)):
        sat_pos = satellites[k]
        color = route_colors[k % len(route_colors)]

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
    plt.legend(loc='best', fontsize=8)
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    plt.tight_layout()
    plt.savefig('2echelon_vrp_solution.png', dpi=300, bbox_inches='tight')
    plt.show()


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """
    Main function to solve the 2E-CVRP example problem.
    """

    print("=" * 70)
    print("TWO-ECHELON CAPACITATED VEHICLE ROUTING PROBLEM (2E-CVRP)")
    print("=" * 70)
    print()

    # Create problem instance
    print("Creating sample problem instance...")
    data = create_sample_problem()

    print(f"Problem size:")
    print(f"  - Satellites: {len(data['satellites'])}")
    print(f"  - Customers: {len(data['customers'])}")
    print(f"  - Total demand: {sum(data['demands'])}")
    print(f"  - 1st level capacity: {data['capacity_1st']}")
    print(f"  - 2nd level capacity: {data['capacity_2nd']}")
    print()

    # Build optimization model
    print("Building Gurobi optimization model...")
    model, x, y, z = build_2echelon_vrp_model(data)
    print(f"Model has {model.NumVars} variables and {model.NumConstrs} constraints")
    print()

    # Solve the model
    print("Solving the model...")
    print("-" * 70)
    model.setParam('TimeLimit', 300)  # 5 minute time limit
    model.setParam('MIPGap', 0.05)    # 5% optimality gap tolerance
    model.optimize()
    print("-" * 70)
    print()

    # Check solution status
    if model.Status == GRB.OPTIMAL:
        print("✓ OPTIMAL SOLUTION FOUND!")
    elif model.Status == GRB.TIME_LIMIT:
        print("✓ TIME LIMIT REACHED - Best solution found:")
    else:
        print("✗ No solution found")
        return

    print()
    print(f"Objective value (total distance): {model.ObjVal:.2f}")
    print()

    # Extract solution
    x_sol = {key: var.X for key, var in x.items() if var.X > 0.5}
    y_sol = {key: var.X for key, var in y.items() if var.X > 0.5}
    z_sol = {key: var.X for key, var in z.items() if var.X > 0.5}

    # Print solution details
    print("=" * 70)
    print("SOLUTION DETAILS")
    print("=" * 70)
    print()

    print("1st Level Routes (Depot → Satellites):")
    print("-" * 40)
    for (i, j), val in x_sol.items():
        node_names = {0: "Depot"}
        for k in range(len(data['satellites'])):
            node_names[k + 1] = f"Satellite {k}"

        print(f"  {node_names[i]} → {node_names[j]}: {int(val)} vehicle(s)")
    print()

    print("Customer Assignments:")
    print("-" * 40)
    for k in range(len(data['satellites'])):
        assigned = [j for j in range(len(data['customers']))
                   if z_sol.get((k, j), 0) > 0.5]
        if assigned:
            total_demand = sum(data['demands'][j] for j in assigned)
            print(f"  Satellite {k}: Customers {assigned} (Total demand: {total_demand})")
    print()

    print("2nd Level Routes (Satellites → Customers):")
    print("-" * 40)
    for k in range(len(data['satellites'])):
        routes_from_k = [(i, j) for (sat, i, j), val in y_sol.items()
                        if sat == k and val > 0.5]
        if routes_from_k:
            print(f"  From Satellite {k}:")
            for i, j in routes_from_k:
                if i == k:
                    print(f"    S{k} → C{j}")
                elif j == k:
                    print(f"    C{i} → S{k}")
                else:
                    print(f"    C{i} → C{j}")
    print()

    # Visualize solution
    print("Generating visualization...")
    visualize_solution(data, x_sol, y_sol, z_sol)
    print("✓ Visualization saved as '2echelon_vrp_solution.png'")
    print()

    print("=" * 70)
    print("DONE!")
    print("=" * 70)


if __name__ == "__main__":
    main()
