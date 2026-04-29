
import re
import json
import pandas as pd
import os
import csv # Import the csv module

def process_log_file(input_filename='sensitivity_analysis_results.csv', output_filename='cleaned_sensitivity_analysis_results.csv'):
    """
    Reads a log file containing messy CSV and JSON data, cleans it,
    and saves it as a proper CSV file.
    """
    parsed_data = []

    # Regex to find a JSON part (non-greedy, dotall to match newlines)
    json_finder_pattern = re.compile(r'({.*?})', re.DOTALL)

    with open(input_filename, 'r', newline='') as infile: # Use newline='' for csv module
        reader = csv.reader(infile)
        header = next(reader) # Skip header

        for row in reader:
            if not row: # Skip empty rows
                continue

            dataset_path = row[0] # Dataset path is the first element
            error_field = row[-1] # The problematic 'error' field is the last element

            json_str = None
            all_json_matches = list(json_finder_pattern.finditer(error_field))
            if all_json_matches:
                json_match = all_json_matches[-1] # Take the last match
                json_str = json_match.group(0)

                # Fix the JSON string by replacing "" with "
                json_str = json_str.replace('""', '"')

                try:
                    data_dict = json.loads(json_str)

                    data_dict['dataset'] = os.path.basename(dataset_path)

                    # Handle infeasible solutions based on the error message
                    if data_dict.get('error') is not None and "Unable to retrieve attribute 'ObjVal'" in data_dict.get('error'):
                        data_dict['total_cost'] = float('inf')

                    if data_dict.get('parameter_modified') == 'demand_capacity':
                        data_dict['parameter_modified'] = 'demand_capacity_factor'

                    if data_dict.get('parameter_modified') in ['vehicle_capacity', 'vehicle_amount']:
                        data_dict['parameter_value'] = (data_dict.get('parameter_value1'), data_dict.get('parameter_value2'))
                    else: # demand_capacity_factor
                        data_dict['parameter_value'] = data_dict.get('parameter_value1')

                    parsed_data.append(data_dict)
                except json.JSONDecodeError as e:
                    print(f"Could not decode JSON from row: {row}")
                    print(f"Problematic JSON string: {json_str}")
                    print(f"Error: {e}")
            else:
                print(f"No JSON object found in the error field of row: {row}")

    df = pd.DataFrame(parsed_data)
    df.to_csv(output_filename, index=False)
    print(f"Successfully created {output_filename}")

if __name__ == "__main__":
    process_log_file()

