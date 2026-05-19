import numpy as np

def parse_file(filepath):
    """
    Parses a 2ECVRP data file and returns a dictionary with specific named keys,
    focusing on coordinates of Depots, Satellites, Customers, and customer demands.
    """
    depot_coords = None
    satellite_coords = []
    customer_coords = []
    customer_demands = []

    with open(filepath, 'r') as f:
        # Skip header
        next(f) 
        for line in f:
            line = line.strip()
            if not line:
                continue

            parts = line.split()
            # Ensure there are enough parts and the node type is recognized
            if len(parts) < 5 or parts[1] not in ['d', 's', 'c', 'f']:
                # Assume we've reached the end of node data if format is unexpected
                # or if the type is not 'd', 's', 'c', or 'f'
                break

            # Extract relevant fields
            node_type = parts[1]
            x = float(parts[2])
            y = float(parts[3])
            # The demand column is the 5th element (index 4)
            demand = float(parts[4]) 

            if node_type == 'd':
                depot_coords = [x, y]
            if node_type == 's':
                satellite_coords.append([x, y])
            if node_type == 'c':
                customer_coords.append([x, y])
                customer_demands.append(demand)

            def euclidean_distance(p1, p2):
                return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

            # Vehicle capacities
            # 1st level vehicles are larger (e.g., trucks)
            # 10 customers = 35,50,6,5
            # 15 customers = 50, 80, 7, 11

            capacity_1st_level = 500
            # 2nd level vehicles are smaller (e.g., vans for city delivery)
            capacity_2nd_level = 600

            # Number of available vehicles
            num_vehicles_1st = 20  # Vehicles at depot
            num_vehicles_2nd = 70  # Total vehicles for 2nd level distribution

            # Satellite capacity (max number of 2nd-level routes from each satellite)
            satellite_capacity = 10  # Each satellite can handle up to 3 delivery routes
    
    
    return {
        'depot': np.array(depot_coords) if depot_coords is not None else None,
        'satellites': np.array(satellite_coords).reshape(-1, 2) if satellite_coords else np.array([]),
        'customers': np.array(customer_coords).reshape(-1, 2) if customer_coords else np.array([]),
        'demands': customer_demands,
        'capacity_1st': capacity_1st_level,
        'capacity_2nd': capacity_2nd_level,
        'num_vehicles_1st': num_vehicles_1st,
        'num_vehicles_2nd': num_vehicles_2nd,
        'satellite_capacity': satellite_capacity,
        'distance_func': euclidean_distance
    }
