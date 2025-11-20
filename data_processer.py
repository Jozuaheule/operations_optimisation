import numpy as np

folder = "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/folder verification"
dataset = 4   # datasets 2,3 have same structure, dataset 4 is a bit different, dataset 1 works with matrix

filepath = "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/Dataset2_example.dat"


def parse_file(filepath):
    """
    Parses a 2ECVRP data file and returns a dictionary with specific named keys.
    """
    # Raw data storage
    raw_data = {
        "metadata": {},
        "nodes": {},
        "satellites": [],
        "demands": {},
        "depots": []
    }

    current_section = None

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            
            if not line or line == "EOF":
                continue

            # Detect sections
            if line in ["NODE_COORD_SECTION", "SATELLITE_SECTION", "DEMAND_SECTION", "DEPOT_SECTION", "FLEET_SECTION"]:
                current_section = line
                continue

            # Parse Metadata & Fleet Key-Values
            if ":" in line:
                key, value = line.split(":", 1)
                key = key.strip()
                value = value.strip()
                
                if value.isdigit() or (value.startswith('-') and value[1:].isdigit()):
                    value = int(value)
                else:
                    try:
                        value = float(value)
                    except ValueError:
                        pass 
                
                raw_data["metadata"][key] = value
                continue

            # Parse Sections
            if current_section == "NODE_COORD_SECTION":
                parts = line.split()
                if len(parts) >= 3:
                    nid, x, y = int(parts[0]), int(parts[1]), int(parts[2])
                    raw_data["nodes"][nid] = [x, y]

            elif current_section == "SATELLITE_SECTION":
                parts = line.split()
                if len(parts) >= 3:
                    # Assuming format: ID X Y
                    x, y = int(parts[1]), int(parts[2])
                    raw_data["satellites"].append([x, y])

            elif current_section == "DEMAND_SECTION":
                parts = line.split()
                if len(parts) >= 2:
                    nid, demand = int(parts[0]), int(parts[1])
                    raw_data["demands"][nid] = demand

            elif current_section == "DEPOT_SECTION":
                val = int(line)
                if val != -1:
                    raw_data["depots"].append(val)

    # --- Construct the specific return object ---
    
    # 1. Depot (Coordinates of the first depot found)
    depot_id = raw_data["depots"][0] if raw_data["depots"] else 0
    depot = raw_data["nodes"].get(depot_id)

    # 2. Satellites (List of coordinates)
    satellites = np.array(raw_data["satellites"]).reshape(-1, 2)

    # 3. Customers (Coordinates) & Demands (Values)
    # We assume all nodes in NODE_COORD_SECTION that are NOT the depot are customers.
    # We sort by ID to ensure alignment between 'customers' list and 'demands' list.
    customer_ids = sorted([
        nid for nid in raw_data["nodes"] 
        if nid not in raw_data["depots"]
    ])
    
    customers = np.array([raw_data["nodes"][cid] for cid in customer_ids]).reshape(-1,2)
    cust_demands = [raw_data["demands"].get(cid, 0) for cid in customer_ids]

    def euclidean_distance(p1, p2):
            return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    # 4. Fleet & Capacity Info
    meta = raw_data["metadata"]
    capacity_1st_level = meta.get("L1CAPACITY")
    capacity_2nd_level = meta.get("L2CAPACITY")
    num_vehicles_1st = meta.get("L1FLEET")
    num_vehicles_2nd = meta.get("L2FLEET")
    
    # Satellite capacity isn't standard in the header, check if it exists or default to None
    # In datasets of 2, 3 is the satellite capacity not given
    satellite_capacity = meta.get("SATELLITE_CAPACITY", 100)

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

print(parse_file(filepath))