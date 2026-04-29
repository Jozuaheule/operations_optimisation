import csv
import subprocess
import sys
import json

# --- Configuration ---
# The script to run for each analysis
TARGET_SCRIPT = "two_echelon_vrp_sa.py"

# Datasets to analyze
DATASETS = [
    # "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/manilakbay-2E-EVRP-Instances-95ae99e/Type_x/Customer_15/C103_C15x.txt",
    # "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/manilakbay-2E-EVRP-Instances-95ae99e/Type_x/Customer_15/R102_C15x.txt",
    # "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/manilakbay-2E-EVRP-Instances-95ae99e/Type_x/Customer_15/RC103_C15x.txt",
    "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/manilakbay-2E-EVRP-Instances-95ae99e/Type_x/Customer_50/C101_C50x.txt",
    "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/manilakbay-2E-EVRP-Instances-95ae99e/Type_x/Customer_50/R101_C50x.txt",
    "/Users/m.j.j.heule/Documents/4. TU Delft/TU Master/Operations optimisation/operations_optimisation/manilakbay-2E-EVRP-Instances-95ae99e/Type_x/Customer_50/RC101_C50x.txt",
]

# Parameters to vary for sensitivity analysis
# Format: "parameter_name": [value1, value2, ...]
PARAMETERS_TO_VARY = {
    "vehicle_capacity": [0.75, 0.9, 0.95, 1.0, 1.05, 1.1, 1.25],
    "demand_capacity": [0.75, 0.9, 0.95, 1.0, 1.05, 1.1, 1.25],
    "vehicle_amount": [0.75, 0.9, 0.95, 1.0, 1.05, 1.1, 1.25]
}

# Output file for the results
OUTPUT_CSV = "sensitivity_analysis_results.csv"



def execute_run(dataset, param_name=None, param_value1=None, param_value2=None):
    """
    Executes a single run of the VRP model script with given parameters.

    Args:
        dataset (str): The path to the dataset file.
        param_name (str, optional): The name of the parameter to modify.
        param_value (any, optional): The value for the parameter.

    Returns:
        dict: A dictionary containing the results from the run.
    """
    command = ["/Library/Frameworks/Python.framework/Versions/3.12/bin/python3.12", TARGET_SCRIPT, "--dataset", dataset]
    if param_name and param_value1 and param_value2 is not None:
        command.extend(["--param_name", param_name, "--sensitivity_multiplier1", str(param_value1), "--sensitivity_multiplier2", str(param_value2)])
    
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, check=False, timeout=600
        )
        
        # Always check stderr, even if the process didn't fail
        if result.returncode != 0:
            error_message = (
                f"Script execution failed with exit code {result.returncode}. "
                f"Stderr: '{result.stderr}'. Stdout: '{result.stdout}'."
            )
            return {"error": error_message}

        try:
            output = json.loads(result.stdout.strip())
            return output
        except json.JSONDecodeError:
            # More specific error for JSON decoding failure
            error_message = (
                "Failed to decode JSON from script output. "
                f"Stdout: '{result.stdout}'. Stderr: '{result.stderr}'."
            )
            return {"error": error_message}

    except subprocess.TimeoutExpired as e:
        # More specific error for timeout
        error_message = (
            "Script execution timed out. "
            f"Stdout: '{e.stdout}'. Stderr: '{e.stderr}'."
        )
        return {"error": error_message}
    except Exception as e:
        return {"error": f"An unexpected error occurred: {str(e)}"}


def run_two_echelon(dataset, param, level1, level2):

    print(f"  Varying {param}: {[level1, level2]}")
    run_results = execute_run(dataset, param, level1, level2)
    
    # Prepare row for CSV
    row_data = {
        "dataset": dataset,
        "parameter_modified": param,
        "parameter_value1": level1,
        "parameter_value2": level2,
        "total_cost": run_results.get("total_cost"),
        "execution_time": run_results.get("execution_time"),
        "is_feasible": run_results.get("is_feasible"),
        "error": run_results.get("error"),
    }

    return row_data
    

def run_analysis():
    """
    Runs the sensitivity analysis by iterating through datasets and parameters.
    """
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
            print(f"--- Analyzing Dataset: {dataset} ---")

            # Vary each parameter
            for param, values in PARAMETERS_TO_VARY.items():

                print('---')
                print(f'Sensitivity of {param} has begon')
                print('---')

                for level1 in values:
                        
                    if param == 'demand_capacity':

                        row_data = run_two_echelon(dataset, param, level1, 0)
                        writer.writerow(row_data)

                    else:
                        
                        for level2 in values:

                            row_data = run_two_echelon(dataset, param, level1, level2)
                            writer.writerow(row_data)

    print(f"\nAnalysis complete. Results saved to {OUTPUT_CSV}")


if __name__ == "__main__":
    run_analysis()
