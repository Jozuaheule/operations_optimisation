import pandas as pd
import ast
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import os

def load_data(filename='cleaned_sensitivity_analysis_results.csv'):
    """Loads the sensitivity analysis results from a CSV file."""
    try:
        df = pd.read_csv(filename)
        return df
    except FileNotFoundError:
        print(f"Error: The file {filename} was not found.")
        return None

def create_demand_capacity_heatmap(df, output_dir='illustrations'):
    """Creates and saves a heatmap for demand capacity analysis."""
    print("Generating Demand Capacity Analysis heatmap...")
    demand_df = df[df['parameter_modified'] == 'demand_capacity_factor'].copy()
    if demand_df.empty:
        print("No data available for demand_capacity_factor.")
        return

    demand_df['parameter_value'] = pd.to_numeric(demand_df['parameter_value'], errors='coerce')
    demand_df.dropna(subset=['parameter_value'], inplace=True)

    table = demand_df.pivot_table(
        index='dataset',
        columns='parameter_value',
        values='total_cost'
    )
    
    plt.figure(figsize=(12, 8))
    cmap = plt.get_cmap('viridis_r').copy()
    cmap.set_bad(color='grey')
    ax = sns.heatmap(table.replace(np.inf, np.nan), cmap=cmap, annot=False) # Turn off automatic annotation
    
    # Manually add annotations
    for i in range(table.shape[0]):
        for j in range(table.shape[1]):
            value = table.iloc[i, j]
            text = 'inf' if np.isinf(value) else f'{value:.1f}'
            ax.text(j + 0.5, i + 0.5, text, ha='center', va='center', color='w')
    plt.title('Demand Capacity Analysis (Total Cost)')
    plt.xlabel('Demand Capacity Factor')
    plt.ylabel('Dataset')
    
    output_path = os.path.join(output_dir, 'demand_capacity_analysis.png')
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f"Saved Demand Capacity Analysis heatmap to {output_path}")

def create_combined_heatmaps(df, output_dir='illustrations'):
    """Creates and saves combined heatmaps for vehicle capacity and amount analysis for each dataset."""
    
    vehicle_capacity_df = df[df['parameter_modified'] == 'vehicle_capacity'].copy()
    vehicle_amount_df = df[df['parameter_modified'] == 'vehicle_amount'].copy()

    datasets = df['dataset'].unique()

    for dataset in datasets:
        print(f"Generating combined heatmap for dataset: {dataset}...")
        
        # Prepare data for vehicle capacity
        cap_df = vehicle_capacity_df[vehicle_capacity_df['dataset'] == dataset].copy()
        if not cap_df.empty:
            cap_df['parsed_values'] = cap_df['parameter_value'].apply(ast.literal_eval)
            cap_df[['layer1', 'layer2']] = pd.DataFrame(cap_df['parsed_values'].tolist(), index=cap_df.index)
            cap_table = cap_df.pivot_table(index='layer1', columns='layer2', values='total_cost')
        else:
            cap_table = pd.DataFrame() # Empty dataframe

        # Prepare data for vehicle amount
        amt_df = vehicle_amount_df[vehicle_amount_df['dataset'] == dataset].copy()
        if not amt_df.empty:
            amt_df['parsed_values'] = amt_df['parameter_value'].apply(ast.literal_eval)
            amt_df[['layer1', 'layer2']] = pd.DataFrame(amt_df['parsed_values'].tolist(), index=amt_df.index)
            amt_table = amt_df.pivot_table(index='layer1', columns='layer2', values='total_cost')
        else:
            amt_table = pd.DataFrame() # Empty dataframe

        if cap_table.empty and amt_table.empty:
            print(f"No vehicle capacity or amount data for dataset: {dataset}")
            continue

        fig, axes = plt.subplots(1, 2, figsize=(20, 8))
        fig.suptitle(f'Vehicle Analysis for Dataset: {dataset}', fontsize=16)

        if not cap_table.empty:
            cmap = plt.get_cmap('viridis_r').copy()
            cmap.set_bad(color='grey')
            sns.heatmap(cap_table.replace(np.inf, np.nan), cmap=cmap, annot=False, ax=axes[0])
            for i in range(cap_table.shape[0]):
                for j in range(cap_table.shape[1]):
                    value = cap_table.iloc[i, j]
                    text = 'inf' if np.isinf(value) else f'{value:.1f}'
                    axes[0].text(j + 0.5, i + 0.5, text, ha='center', va='center', color='w')
            axes[0].set_title('Vehicle Capacity Analysis')
            axes[0].set_xlabel('Layer 2 Capacity Multiplier')
            axes[0].set_ylabel('Layer 1 Capacity Multiplier')
        else:
            axes[0].text(0.5, 0.5, 'No Data', ha='center', va='center')
            axes[0].set_title('Vehicle Capacity Analysis')


        if not amt_table.empty:
            cmap = plt.get_cmap('viridis_r').copy()
            cmap.set_bad(color='grey')
            sns.heatmap(amt_table.replace(np.inf, np.nan), cmap=cmap, annot=False, ax=axes[1])
            for i in range(amt_table.shape[0]):
                for j in range(amt_table.shape[1]):
                    value = amt_table.iloc[i, j]
                    text = 'inf' if np.isinf(value) else f'{value:.1f}'
                    axes[1].text(j + 0.5, i + 0.5, text, ha='center', va='center', color='w')
            axes[1].set_title('Vehicle Amount Analysis')
            axes[1].set_xlabel('Layer 2 Amount Multiplier')
            axes[1].set_ylabel('Layer 1 Amount Multiplier')
        else:
            axes[1].text(0.5, 0.5, 'No Data', ha='center', va='center')
            axes[1].set_title('Vehicle Amount Analysis')

        output_path = os.path.join(output_dir, f'vehicle_analysis_{dataset}.png')
        plt.savefig(output_path, bbox_inches='tight')
        plt.close()
        print(f"Saved combined heatmap for {dataset} to {output_path}")

def main():
    """Main function to generate and display tables."""
    output_dir = 'illustrations'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    df = load_data()
    if df is not None:
        create_demand_capacity_heatmap(df, output_dir)
        create_combined_heatmaps(df, output_dir)

if __name__ == "__main__":
    main()
