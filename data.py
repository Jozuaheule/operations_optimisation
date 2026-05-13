import numpy as np


def parse_file(filepath):
    """Parse a 2E-CVRP dataset file and return a problem data dict."""
    depot_coords = None
    satellite_coords = []
    customer_coords = []
    customer_demands = []

    with open(filepath, 'r') as f:
        next(f)  # skip header line
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 5 or parts[1] not in ('d', 's', 'c', 'f'):
                break
            node_type = parts[1]
            x, y, demand = float(parts[2]), float(parts[3]), float(parts[4])
            if node_type == 'd':
                depot_coords = [x, y]
            elif node_type == 's':
                satellite_coords.append([x, y])
            elif node_type == 'c':
                customer_coords.append([x, y])
                customer_demands.append(demand)

    def euclidean_distance(p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    return {
        'depot': np.array(depot_coords) if depot_coords is not None else None,
        'satellites': np.array(satellite_coords).reshape(-1, 2) if satellite_coords else np.array([]),
        'customers': np.array(customer_coords).reshape(-1, 2) if customer_coords else np.array([]),
        'demands': customer_demands,
        'capacity_1st': 300,
        'capacity_2nd': 550,
        'num_vehicles_1st': 14,
        'num_vehicles_2nd': 60,
        'satellite_capacity': 5,
        'distance_func': euclidean_distance,
    }
