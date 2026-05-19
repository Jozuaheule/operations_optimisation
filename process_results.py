import pandas as pd
import os

def process_results(input_filename='sensitivity_analysis_results.csv', output_filename='cleaned_sensitivity_analysis_results.csv'):
    """
    Reads the results CSV and cleans it for visualization.
    """
    if not os.path.exists(input_filename):
        print(f"Error: {input_filename} not found.")
        return

    df = pd.read_csv(input_filename)
    
    # Rename demand_capacity to demand_capacity_factor to match existing visualization logic
    df.loc[df['parameter_modified'] == 'demand_capacity', 'parameter_modified'] = 'demand_capacity_factor'
    
    # Handle satellite_capacity name if needed (it matches already)
    
    # Handle infeasible runs (total_cost is null or 0 if script failed to find ObjVal)
    # Note: Our sensitivity_analyzer already records total_cost from ObjVal
    df['total_cost'] = pd.to_numeric(df['total_cost'], errors='coerce')
    df.loc[df['total_cost'].isna(), 'total_cost'] = float('inf')
    
    # Create dataset_name for cleaner labels
    df['dataset_name'] = df['dataset'].apply(lambda x: os.path.basename(str(x)))
    
    # Save cleaned data
    df.to_csv(output_filename, index=False)
    print(f"Successfully created {output_filename}")

if __name__ == "__main__":
    process_results()
