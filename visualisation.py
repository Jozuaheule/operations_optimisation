import pandas as pd
import ast
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import os
from dataset_processer import parse_file

def load_data(filename='cleaned_sensitivity_analysis_results.csv'):
    """Loads the sensitivity analysis results from a CSV file."""
    try:
        df = pd.read_csv(filename)
        df['dataset_name'] = df['dataset'].apply(lambda x: os.path.basename(str(x)))
        return df
    except FileNotFoundError:
        print(f"Error: The file {filename} was not found.")
        return None

def get_dataset_baselines(df):
    """Finds the baseline cost for each dataset (multiplier=1.0 or offset=0.0)."""
    baselines = {}
    for dataset in df['dataset'].unique():
        ds_df = df[df['dataset'] == dataset]
        neutral = ds_df[
            (ds_df['parameter_value1'].isin([1.0, 0.0])) & 
            (ds_df['parameter_value2'].fillna(0.0).isin([1.0, 0.0]))
        ]
        if not neutral.empty:
            baselines[dataset] = neutral['total_cost'].mode().iloc[0]
    return baselines

def get_baseline_values(df):
    """Parses datasets to find original parameter values for elasticity calculation."""
    baselines = {}
    unique_datasets = df['dataset'].unique()
    for ds_path in unique_datasets:
        try:
            data = parse_file(ds_path)
            baselines[os.path.basename(ds_path)] = {
                'vehicle_amount_l1': data['num_vehicles_1st'],
                'vehicle_amount_l2': data['num_vehicles_2nd'],
                'satellite_capacity': data['satellite_capacity']
            }
        except Exception as e:
            print(f"Warning: Could not parse baseline for {ds_path}: {e}")
    return baselines

def create_single_param_heatmap(df, param_name, baseline_costs, title, xlabel, output_name, output_dir='illustrations'):
    """Creates a heatmap for single-parameter analysis showing % cost change."""
    print(f"Generating {title} heatmap...")
    sub_df = df[df['parameter_modified'] == param_name].copy()
    if sub_df.empty: return

    sub_df['parameter_value1'] = pd.to_numeric(sub_df['parameter_value1'], errors='coerce')
    sub_df['total_cost'] = pd.to_numeric(sub_df['total_cost'], errors='coerce')

    def calc_pct(row):
        base = baseline_costs.get(row['dataset'])
        if base and base > 0 and not np.isinf(row['total_cost']):
            return (row['total_cost'] - base) / base * 100
        return np.nan

    sub_df['cost_pct_change'] = sub_df.apply(calc_pct, axis=1)
    
    table = sub_df.pivot_table(index='dataset_name', columns='parameter_value1', values='cost_pct_change')
    
    plt.figure(figsize=(12, 8))
    cmap = plt.get_cmap('RdYlGn_r').copy()
    cmap.set_bad(color='grey')
    ax = sns.heatmap(table.replace([np.inf, -np.inf], np.nan), cmap=cmap, center=0, annot=True, fmt=".1f", 
                     annot_kws={"size": 12})
    if ax.texts:
        for t in ax.texts: t.set_text(t.get_text() + "%")
    
    plt.title(f'{title} (% Cost Change)', fontsize=16)
    plt.xlabel(xlabel, fontsize=14)
    plt.ylabel('Dataset Name', fontsize=14)
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    plt.savefig(os.path.join(output_dir, f'{output_name}.png'), bbox_inches='tight')
    plt.close()

def create_combined_heatmaps(df, baseline_costs, output_dir='illustrations'):
    """Creates and saves combined heatmaps for vehicle capacity and amount analysis."""
    datasets = df['dataset'].unique()
    for dataset in datasets:
        ds_name = os.path.basename(dataset)
        ds_all = df[df['dataset'] == dataset]
        base_cost = baseline_costs.get(dataset)
        if not base_cost or np.isinf(base_cost): continue

        def get_2d_table(param_name):
            sub = ds_all[ds_all['parameter_modified'] == param_name].copy()
            if sub.empty: return pd.DataFrame()
            sub['pct_change'] = (sub['total_cost'] - base_cost) / base_cost * 100
            return sub.pivot_table(index='parameter_value1', columns='parameter_value2', values='pct_change')

        cap_table = get_2d_table('vehicle_capacity')
        amt_table = get_2d_table('vehicle_amount')

        if cap_table.empty and amt_table.empty: continue

        fig, axes = plt.subplots(1, 2, figsize=(20, 8))
        fig.suptitle(f'Vehicle Analysis for {ds_name} (% Cost Change)', fontsize=16)
        cmap = plt.get_cmap('RdYlGn_r').copy()
        cmap.set_bad(color='grey')

        for i, (table, title, xl, yl) in enumerate([
            (cap_table, 'Vehicle Capacity Analysis (Multipliers)', 'L2 Capacity Multiplier', 'L1 Capacity Multiplier'),
            (amt_table, 'Vehicle Amount Analysis (Offsets)', 'L2 Amount Offset', 'L1 Amount Offset')
        ]):
            if not table.empty:
                sns.heatmap(table.replace([np.inf, -np.inf], np.nan), cmap=cmap, center=0, annot=True, fmt=".1f", ax=axes[i],
                            annot_kws={"size": 12})
                if axes[i].texts:
                    for t in axes[i].texts: t.set_text(t.get_text() + "%")
                axes[i].set_title(title, fontsize=14)
                axes[i].set_xlabel(xl, fontsize=12)
                axes[i].set_ylabel(yl, fontsize=12)
                axes[i].tick_params(axis='both', which='major', labelsize=10)
            else:
                axes[i].text(0.5, 0.5, 'No Data', ha='center', va='center', fontsize=14)
        plt.savefig(os.path.join(output_dir, f'vehicle_analysis_{ds_name}.png'), bbox_inches='tight')
        plt.close()

def create_elasticity_analysis(df, output_dir='illustrations'):
    """Calculates and plots the elasticity of each parameter."""
    print("Generating Elasticity Analysis...")
    baselines_info = get_baseline_values(df)
    elasticity_results = []

    for dataset in df['dataset'].unique():
        ds_name = os.path.basename(dataset)
        ds_df = df[df['dataset'] == dataset].copy()
        ds_info = baselines_info.get(ds_name)
        if not ds_info: continue

        for param in ds_df['parameter_modified'].unique():
            param_df = ds_df[ds_df['parameter_modified'] == param].copy()
            neutral = param_df[
                (param_df['parameter_value1'].isin([1.0, 0.0])) & 
                (param_df['parameter_value2'].fillna(0.0).isin([1.0, 0.0]))
            ]
            if neutral.empty: continue
            cost0 = neutral.iloc[0]['total_cost']
            if np.isinf(cost0) or cost0 <= 0: continue

            for _, row in param_df.iterrows():
                cost = row['total_cost']
                v1, v2 = row['parameter_value1'], row.get('parameter_value2', 0)
                
                # Identify if this IS the neutral row (skip it for calculation)
                if param in ['vehicle_capacity', 'demand_capacity_factor']:
                    if v1 == 1.0 and (np.isnan(v2) or v2 == 1.0 or v2 == 0): is_neutral = True
                    else: is_neutral = False
                else:
                    if v1 == 0.0 and (np.isnan(v2) or v2 == 0.0): is_neutral = True
                    else: is_neutral = False
                
                if is_neutral: continue
                
                # Calculate Delta Cost (handle infeasible as a large penalty or skip)
                if np.isinf(cost): 
                    continue # Infeasible doesn't give a good elasticity number
                
                pct_delta_cost = (cost - cost0) / cost0
                
                # Calculate Delta Param
                if param in ['vehicle_capacity', 'demand_capacity_factor']:
                    d1 = abs(v1 - 1.0)
                    d2 = abs(v2 - 1.0) if (not np.isnan(v2) and v2 != 0) else 0
                    pct_delta_param = max(d1, d2)
                else:
                    if param == 'satellite_capacity': denom = ds_info['satellite_capacity']
                    elif param == 'vehicle_amount': denom = (ds_info['vehicle_amount_l1'] + ds_info['vehicle_amount_l2']) / 2
                    else: denom = 1
                    
                    d1 = abs(v1) / denom if denom != 0 else 0
                    d2 = abs(v2) / denom if (not np.isnan(v2) and v2 != 0) else 0
                    pct_delta_param = max(d1, d2)

                if pct_delta_param > 0:
                    elasticity_results.append({
                        'Parameter': param,
                        'Dataset': ds_name,
                        'Elasticity': abs(pct_delta_cost / pct_delta_param)
                    })

    if not elasticity_results: return
    elast_df = pd.DataFrame(elasticity_results)
    avg_elast = elast_df.groupby('Parameter')['Elasticity'].mean().sort_values(ascending=False).reset_index()
    
    plt.figure(figsize=(10, 6))
    sns.barplot(data=avg_elast, x='Elasticity', y='Parameter', palette='viridis', hue='Parameter', legend=False)
    plt.axvline(x=1.0, color='red', linestyle='--', label='Unit Elasticity (1.0)')
    plt.title('Parameter Sensitivity Ranking (Elasticity)', fontsize=16)
    plt.xlabel('Elasticity (|% ΔCost / % ΔParam|)', fontsize=14)
    plt.ylabel('Parameter', fontsize=14)
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    plt.grid(axis='x', alpha=0.3)
    plt.savefig(os.path.join(output_dir, 'parameter_elasticity_ranking.png'), bbox_inches='tight')
    plt.close()

def main():
    output_dir = 'illustrations'
    if not os.path.exists(output_dir): os.makedirs(output_dir)
    df = load_data()
    if df is not None:
        baseline_costs = get_dataset_baselines(df)
        create_single_param_heatmap(df, 'demand_capacity_factor', baseline_costs, 'Demand Capacity', 'Multiplier', 'demand_capacity_analysis')
        create_single_param_heatmap(df, 'satellite_capacity', baseline_costs, 'Satellite Capacity', 'Offset', 'satellite_capacity_analysis')
        create_combined_heatmaps(df, baseline_costs, output_dir)
        create_elasticity_analysis(df, output_dir)

if __name__ == "__main__":
    main()
