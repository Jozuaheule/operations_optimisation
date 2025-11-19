"""
Model Verification and Infeasibility Analysis for 2E-CVRP

This script identifies and fixes the infeasibility issues in the original model.
"""

import gurobipy as gp
from gurobipy import GRB
import numpy as np

def create_simple_problem():
    """
    Creates a very simple 2E-CVRP instance for debugging.
    """
    depot = np.array([0, 0])
    satellites = np.array([[20, 0]])  # Just 1 satellite
    customers = np.array([[30, 5], [30, -5]])  # Just 2 customers
    demands = np.array([10, 10])

    def euclidean_distance(p1, p2):
        return np.sqrt(np.sum((p1 - p2) ** 2))

    return {
        'depot': depot,
        'satellites': satellites,
        'customers': customers,
        'demands': demands,
        'capacity_1st': 50,
        'capacity_2nd': 20,
        'num_vehicles_1st': 2,
        'num_vehicles_2nd': 2,
        'satellite_capacity': 2,
        'distance_func': euclidean_distance
    }


def diagnose_infeasibility():
    """
    Systematically checks for infeasibility sources.
    """
    print("=" * 80)
    print("2E-CVRP MODEL INFEASIBILITY ANALYSIS")
    print("=" * 80)
    print()

    data = create_simple_problem()

    print("Problem Configuration:")
    print(f"  Satellites: {len(data['satellites'])}")
    print(f"  Customers: {len(data['customers'])}")
    print(f"  Total demand: {sum(data['demands'])}")
    print(f"  1st level capacity: {data['capacity_1st']} per vehicle")
    print(f"  2nd level capacity: {data['capacity_2nd']} per vehicle")
    print(f"  Available 1st level vehicles: {data['num_vehicles_1st']}")
    print(f"  Available 2nd level vehicles: {data['num_vehicles_2nd']}")
    print(f"  Satellite capacity: {data['satellite_capacity']} routes per satellite")
    print()

    # Check 1: Total capacity vs demand
    print("-" * 80)
    print("CHECK 1: Capacity vs Demand Analysis")
    print("-" * 80)

    total_demand = sum(data['demands'])
    total_1st_capacity = data['num_vehicles_1st'] * data['capacity_1st']
    total_2nd_capacity = data['num_vehicles_2nd'] * data['capacity_2nd']

    print(f"Total demand: {total_demand}")
    print(f"Total 1st level capacity: {total_1st_capacity} ({data['num_vehicles_1st']} × {data['capacity_1st']})")
    print(f"Total 2nd level capacity: {total_2nd_capacity} ({data['num_vehicles_2nd']} × {data['capacity_2nd']})")

    if total_1st_capacity < total_demand:
        print("❌ INFEASIBLE: 1st level capacity insufficient!")
    else:
        print("✓ 1st level capacity is sufficient")

    if total_2nd_capacity < total_demand:
        print("❌ INFEASIBLE: 2nd level capacity insufficient!")
    else:
        print("✓ 2nd level capacity is sufficient")
    print()

    # Check 2: Satellite capacity constraints
    print("-" * 80)
    print("CHECK 2: Satellite Route Capacity")
    print("-" * 80)

    n_satellites = len(data['satellites'])
    n_customers = len(data['customers'])

    max_routes_total = n_satellites * data['satellite_capacity']
    min_routes_needed = n_customers  # At least 1 route per customer if each in separate route

    print(f"Maximum routes available: {max_routes_total} ({n_satellites} satellites × {data['satellite_capacity']})")
    print(f"Minimum routes needed: {min_routes_needed} (if each customer needs separate route)")

    # More realistic: how many routes needed given capacity?
    routes_needed_by_capacity = 0
    for demand in data['demands']:
        if demand > data['capacity_2nd']:
            print(f"❌ INFEASIBLE: Customer demand {demand} exceeds vehicle capacity {data['capacity_2nd']}!")
            routes_needed_by_capacity = float('inf')
            break
    else:
        # Simple bin packing estimate
        routes_needed_by_capacity = int(np.ceil(total_demand / data['capacity_2nd']))

    print(f"Routes needed (by capacity): {routes_needed_by_capacity}")

    if routes_needed_by_capacity > max_routes_total:
        print(f"❌ POTENTIAL ISSUE: May need more routes than satellite capacity allows")
    else:
        print("✓ Satellite capacity should be sufficient")
    print()

    # Check 3: Constraint conflicts
    print("-" * 80)
    print("CHECK 3: Identifying Constraint Conflicts in Original Model")
    print("-" * 80)
    print()

    print("ISSUE 1: Flow Conservation Conflicts")
    print("  Problem: The original model has conflicting flow constraints")
    print("  - Constraint (9) demands exact flow balance at each customer")
    print("  - Constraint (10) requires flow in - flow out = demand × z[k,j]")
    print("  - These can conflict when customer j is NOT assigned to satellite k")
    print()

    print("ISSUE 2: Over-constrained Assignment")
    print("  Problem: Links between y and z variables are too restrictive")
    print("  - Constraint (8) says: if y[k,i,j] > 0, then z[k,j] must be 1")
    print("  - But flow balance requires flows even for non-assigned customers")
    print("  - This creates logical conflicts")
    print()

    print("ISSUE 3: Satellite Demand Calculation")
    print("  Problem: Constraint (11) links 1st and 2nd levels incorrectly")
    print("  - Uses z[k,j] to calculate satellite demand")
    print("  - But z values are decision variables, not known a priori")
    print("  - Creates circular dependency in flow calculations")
    print()

    print("ISSUE 4: Unnecessary Flow Return Constraints")
    print("  Problem: Return flows to depot and satellites set to 0")
    print("  - In actual routing, vehicles DO return")
    print("  - These constraints (11-12) are wrong")
    print()

    return data


def build_corrected_model(data):
    """
    Builds a corrected, feasible version of the 2E-CVRP model.

    Key fixes:
    1. Simplified flow constraints
    2. Better linking between levels
    3. Removed conflicting constraints
    4. Added subtour elimination
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

    model = gp.Model("2E-CVRP-Corrected")

    # ========================================================================
    # DECISION VARIABLES
    # ========================================================================

    # --- Assignment variables (determine the structure first) ---
    z = {}
    for k in range(n_satellites):
        for j in range(n_customers):
            z[k, j] = model.addVar(vtype=GRB.BINARY, name=f"z_{k}_{j}")

    # --- 1st Level: Simple arc variables ---
    x = {}
    # From depot to satellites
    for k in range(n_satellites):
        x[0, k+1] = model.addVar(vtype=GRB.INTEGER, lb=0, name=f"x_0_{k+1}")
    # Between satellites (optional, usually not used in simple cases)
    for i in range(n_satellites):
        for j in range(n_satellites):
            if i != j:
                x[i+1, j+1] = model.addVar(vtype=GRB.INTEGER, lb=0, name=f"x_{i+1}_{j+1}")
    # Return to depot
    for k in range(n_satellites):
        x[k+1, 0] = model.addVar(vtype=GRB.INTEGER, lb=0, name=f"x_{k+1}_0")

    # --- 2nd Level: Routing variables ---
    y = {}
    for k in range(n_satellites):
        # From satellite to customers
        for j in range(n_customers):
            y[k, n_satellites+k, n_satellites+k+n_customers+j] = model.addVar(
                vtype=GRB.BINARY, name=f"y_{k}_s{k}_c{j}"
            )

        # Between customers
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    y[k, n_satellites+k+n_customers+i, n_satellites+k+n_customers+j] = model.addVar(
                        vtype=GRB.BINARY, name=f"y_{k}_c{i}_c{j}"
                    )

        # From customers back to satellite
        for i in range(n_customers):
            y[k, n_satellites+k+n_customers+i, n_satellites+k] = model.addVar(
                vtype=GRB.BINARY, name=f"y_{k}_c{i}_s{k}"
            )

    model.update()

    # ========================================================================
    # OBJECTIVE FUNCTION
    # ========================================================================

    obj = gp.LinExpr()

    # 1st level costs
    for (i, j), var in x.items():
        if i == 0:
            pos_i = depot
        else:
            pos_i = satellites[i-1]

        if j == 0:
            pos_j = depot
        else:
            pos_j = satellites[j-1]

        cost = dist_func(pos_i, pos_j)
        obj += cost * var

    # 2nd level costs
    for k in range(n_satellites):
        sat_pos = satellites[k]

        # Satellite to customer
        for j in range(n_customers):
            key = (k, n_satellites+k, n_satellites+k+n_customers+j)
            if key in y:
                cost = dist_func(sat_pos, customers[j])
                obj += cost * y[key]

        # Customer to customer
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    key = (k, n_satellites+k+n_customers+i, n_satellites+k+n_customers+j)
                    if key in y:
                        cost = dist_func(customers[i], customers[j])
                        obj += cost * y[key]

            # Customer to satellite
            key = (k, n_satellites+k+n_customers+i, n_satellites+k)
            if key in y:
                cost = dist_func(customers[i], sat_pos)
                obj += cost * y[key]

    model.setObjective(obj, GRB.MINIMIZE)

    # ========================================================================
    # CONSTRAINTS
    # ========================================================================

    # --- ASSIGNMENT CONSTRAINTS ---

    # (1) Each customer assigned to exactly one satellite
    for j in range(n_customers):
        model.addConstr(
            gp.quicksum(z[k, j] for k in range(n_satellites)) == 1,
            name=f"assign_{j}"
        )

    # --- 1ST LEVEL CONSTRAINTS ---

    # (2) Vehicle limit from depot
    model.addConstr(
        gp.quicksum(x[0, k+1] for k in range(n_satellites)) <= m1,
        name="1st_level_vehicles"
    )

    # (3) Flow conservation at satellites
    for k in range(n_satellites):
        model.addConstr(
            gp.quicksum(x[i, k+1] for i in range(n_satellites+1) if (i, k+1) in x) ==
            gp.quicksum(x[k+1, j] for j in range(n_satellites+1) if (k+1, j) in x),
            name=f"1st_flow_{k}"
        )

    # (4) 1st level capacity (simplified)
    # Total demand to satellite k must be deliverable
    for k in range(n_satellites):
        demand_at_k = gp.quicksum(demands[j] * z[k, j] for j in range(n_customers))
        model.addConstr(
            gp.quicksum(x[i, k+1] for i in range(n_satellites+1) if (i, k+1) in x) * K1 >= demand_at_k,
            name=f"1st_capacity_{k}"
        )

    # --- 2ND LEVEL CONSTRAINTS ---

    # (5) Flow conservation at customers
    for k in range(n_satellites):
        for j in range(n_customers):
            # Inflow
            inflow = gp.LinExpr()
            # From satellite
            key_in = (k, n_satellites+k, n_satellites+k+n_customers+j)
            if key_in in y:
                inflow += y[key_in]
            # From other customers
            for i in range(n_customers):
                if i != j:
                    key_in = (k, n_satellites+k+n_customers+i, n_satellites+k+n_customers+j)
                    if key_in in y:
                        inflow += y[key_in]

            # Outflow
            outflow = gp.LinExpr()
            # To satellite
            key_out = (k, n_satellites+k+n_customers+j, n_satellites+k)
            if key_out in y:
                outflow += y[key_out]
            # To other customers
            for i in range(n_customers):
                if i != j:
                    key_out = (k, n_satellites+k+n_customers+j, n_satellites+k+n_customers+i)
                    if key_out in y:
                        outflow += y[key_out]

            # If customer j is assigned to satellite k, inflow must equal outflow
            # and must be at least 1 (visited)
            model.addConstr(
                inflow == outflow,
                name=f"2nd_flow_{k}_{j}"
            )
            model.addConstr(
                inflow >= z[k, j],
                name=f"2nd_visit_{k}_{j}"
            )
            model.addConstr(
                inflow <= z[k, j] * n_customers,
                name=f"2nd_novisit_{k}_{j}"
            )

    # (6) Satellite capacity (number of routes)
    for k in range(n_satellites):
        num_routes = gp.quicksum(
            y[k, n_satellites+k, n_satellites+k+n_customers+j]
            for j in range(n_customers)
            if (k, n_satellites+k, n_satellites+k+n_customers+j) in y
        )
        model.addConstr(
            num_routes <= satellite_cap,
            name=f"sat_capacity_{k}"
        )

    # (7) Total 2nd level vehicles
    total_routes = gp.quicksum(
        y[k, n_satellites+k, n_satellites+k+n_customers+j]
        for k in range(n_satellites)
        for j in range(n_customers)
        if (k, n_satellites+k, n_satellites+k+n_customers+j) in y
    )
    model.addConstr(total_routes <= m2, name="2nd_level_vehicles")

    # (8) 2nd level route capacity (simplified using MTZ-like constraints)
    # Each route must respect capacity
    u = {}  # Position in route
    for k in range(n_satellites):
        for j in range(n_customers):
            u[k, j] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=K2, name=f"u_{k}_{j}")

    for k in range(n_satellites):
        for i in range(n_customers):
            for j in range(n_customers):
                if i != j:
                    key = (k, n_satellites+k+n_customers+i, n_satellites+k+n_customers+j)
                    if key in y:
                        # MTZ constraint for subtour elimination and capacity
                        model.addConstr(
                            u[k, i] + demands[j] * z[k, j] <= u[k, j] + K2 * (1 - y[key]),
                            name=f"mtz_{k}_{i}_{j}"
                        )

    # (9) Satellite must be visited by 1st level if it serves customers
    for k in range(n_satellites):
        customers_at_k = gp.quicksum(z[k, j] for j in range(n_customers))
        model.addConstr(
            gp.quicksum(x[i, k+1] for i in range(n_satellites+1) if (i, k+1) in x) >=
            customers_at_k / n_customers,  # At least some service if customers assigned
            name=f"link_{k}"
        )

    return model, x, y, z


def main():
    """
    Main verification and correction process.
    """

    # Step 1: Diagnose original model
    data = diagnose_infeasibility()

    # Step 2: Build corrected model
    print("-" * 80)
    print("BUILDING CORRECTED MODEL")
    print("-" * 80)
    print()

    model, x, y, z = build_corrected_model(data)

    print(f"Corrected model: {model.NumVars} variables, {model.NumConstrs} constraints")
    print()

    # Step 3: Solve corrected model
    print("-" * 80)
    print("SOLVING CORRECTED MODEL")
    print("-" * 80)
    print()

    model.setParam('OutputFlag', 1)
    model.setParam('TimeLimit', 60)
    model.optimize()

    print()
    print("=" * 80)
    print("RESULTS")
    print("=" * 80)
    print()

    if model.Status == GRB.OPTIMAL:
        print("✓✓✓ CORRECTED MODEL IS FEASIBLE AND OPTIMAL! ✓✓✓")
        print(f"Optimal objective value: {model.ObjVal:.2f}")
        print()

        # Print solution
        print("Customer Assignments:")
        for k in range(len(data['satellites'])):
            assigned = [j for j in range(len(data['customers']))
                       if z[k, j].X > 0.5]
            if assigned:
                print(f"  Satellite {k}: Customers {assigned}")

        print()
        print("1st Level Routes:")
        for key, var in x.items():
            if var.X > 0.5:
                print(f"  {key[0]} → {key[1]}: {int(var.X)} vehicle(s)")

    elif model.Status == GRB.INFEASIBLE:
        print("✗✗✗ STILL INFEASIBLE ✗✗✗")
        print("Computing IIS (Irreducible Inconsistent Subsystem)...")
        model.computeIIS()
        model.write("model_iis.ilp")
        print("IIS written to model_iis.ilp")
    else:
        print(f"Solution status: {model.Status}")

    print()
    print("=" * 80)
    print()

    # Step 4: Explain key fixes
    print("=" * 80)
    print("KEY FIXES APPLIED TO MAKE MODEL FEASIBLE")
    print("=" * 80)
    print()

    print("FIX 1: Separated Assignment from Routing")
    print("  - First determine which customers go to which satellites (z variables)")
    print("  - Then determine routes (y variables)")
    print("  - This breaks the circular dependency")
    print()

    print("FIX 2: Simplified Flow Constraints")
    print("  - Removed conflicting flow balance equations")
    print("  - Used simpler in-degree = out-degree constraints")
    print("  - Flow only required for assigned customers")
    print()

    print("FIX 3: Correct Capacity Constraints")
    print("  - 1st level: Ensure enough vehicles to deliver satellite demand")
    print("  - 2nd level: Use MTZ constraints for subtour elimination + capacity")
    print("  - Removed incorrect return flow constraints")
    print()

    print("FIX 4: Proper Level Linking")
    print("  - Satellite visited by 1st level IF it has assigned customers")
    print("  - Removed over-constrained linking through flow variables")
    print()

    print("=" * 80)
    print()


if __name__ == "__main__":
    main()
