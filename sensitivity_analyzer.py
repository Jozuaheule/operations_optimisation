import csv
import subprocess
import sys
import json
import os

# --- Configuration ---
# The script to run for each analysis
TARGET_SCRIPT = "two_echelon_vrp_sa.py"
# Datasets to analyze - TYPE Y (50 customers)
DATASETS = [
    "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/manilakbay-2E-EVRP-Instances-95ae99e/Type_y/Customer_50/C202_C50y.txt",
    "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/manilakbay-2E-EVRP-Instances-95ae99e/Type_y/Customer_50/R202_C50y.txt",
    "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/manilakbay-2E-EVRP-Instances-95ae99e/Type_y/Customer_50/RC202_C50y.txt",
]

# Parameters to vary for sensitivity analysis
# Format: "parameter_name": [value1, value2, ...]
PARAMETERS_TO_VARY = {
    "vehicle_capacity": [0.8, 0.9, 1.0, 1.1, 1.2],
    "demand_capacity": [0.8, 0.9, 1.0, 1.1, 1.2],
    "vehicle_amount": [-15, -5, 0, 5, 15],
    "satellite_capacity": [-3, -1, 0, 1, 3]
}

# Output file for the results
OUTPUT_CSV = "sensitivity_analysis_results.csv"

def execute_run(dataset, param_name=None, param_value1=None, param_value2=None):
    """
    Executes a single run of the VRP model script with given parameters.
    """
    command = ["/Library/Frameworks/Python.framework/Versions/3.12/bin/python3.12", TARGET_SCRIPT, "--dataset", dataset]
    if param_name is not None and param_value1 is not None and param_value2 is not None:
        command.extend(["--param_name", param_name, "--sensitivity_multiplier1", str(param_value1), "--sensitivity_multiplier2", str(param_value2)])
    
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, check=False, timeout=600
        )
        
        if result.returncode != 0:
            return {"error": f"Exit code {result.returncode}", "stderr": result.stderr}

        stdout_clean = result.stdout.strip()
        if "{" in stdout_clean:
            stdout_clean = stdout_clean[stdout_clean.find("{"):]
        
        try:
            return json.loads(stdout_clean)
        except json.JSONDecodeError:
            return {"error": f"Invalid JSON output: {stdout_clean[:100]}"}

    except Exception as e:
        return {"error": str(e)}

def run_analysis():
    fieldnames = [
        "dataset",
        "parameter_modified",
        "parameter_value1",
        "parameter_value2",
        "total_cost",
        "execution_time",
        "is_feasible",
        "error",
    ]

    with open(OUTPUT_CSV, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for dataset in DATASETS:
            ds_name = os.path.basename(dataset)
            print(f"--- Analyzing Dataset: {ds_name} ---")

            for param, values in PARAMETERS_TO_VARY.items():
                print(f"  Varying {param}...")
                
                if param in ['demand_capacity', 'satellite_capacity']:
                    for level1 in values:
                        print(f"    Running: {param} with value {level1}...")
                        res = execute_run(dataset, param, level1, 0)
                        writer.writerow({
                            "dataset": dataset,
                            "parameter_modified": param,
                            "parameter_value1": level1,
                            "parameter_value2": 0,
                            "total_cost": res.get("total_cost"),
                            "execution_time": res.get("execution_time"),
                            "is_feasible": res.get("is_feasible", False),
                            "error": res.get("error"),
                        })
                else:
                    for level1 in values:
                        for level2 in values:
                            print(f"    Running: {param} with values {level1}, {level2}...")
                            res = execute_run(dataset, param, level1, level2)
                            writer.writerow({
                                "dataset": dataset,
                                "parameter_modified": param,
                                "parameter_value1": level1,
                                "parameter_value2": level2,
                                "total_cost": res.get("total_cost"),
                                "execution_time": res.get("execution_time"),
                                "is_feasible": res.get("is_feasible", False),
                                "error": res.get("error"),
                            })

    print(f"\nAnalysis complete. Results saved to {OUTPUT_CSV}")

if __name__ == "__main__":
    run_analysis()
