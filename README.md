# Two-Echelon Capacitated Vehicle Routing Problem (2E-CVRP) Optimization

This project provides a comprehensive framework for solving and analyzing the **Two-Echelon Capacitated Vehicle Routing Problem (2E-CVRP)**. It utilizes mathematical optimization (Gurobi) to minimize transportation costs in a two-level distribution network consisting of a central depot, intermediate satellites, and final customers.

## Overview

The 2E-CVRP is a distribution problem where freight is delivered from a central depot to customers via intermediate facilities called satellites. This structure is common in urban logistics to reduce the impact of large vehicles in city centers.

### Key Components:
1.  **1st Echelon:** Depot → Satellites (using larger, high-capacity vehicles).
2.  **2nd Echelon:** Satellites → Customers (using smaller, city-friendly vehicles).

## Project Features

-   **Mathematical Optimization:** Implements a Mixed-Integer Linear Programming (MILP) model using `gurobipy`.
-   **Benchmark Support:** Parses standard 2E-EVRP benchmark datasets (Manilakbay instances).
-   **Sensitivity Analysis:** Tools to analyze the impact of various parameters (vehicle capacity, number of vehicles, satellite capacity, demand) on total costs.
-   **Elasticity Ranking:** Calculates the sensitivity of cost to parameter changes.
-   **Advanced Visualization:** 
    *   Spatial route maps showing both distribution levels.
    *   Sensitivity heatmaps for parameter interaction.
    *   Parameter elasticity ranking charts.

## Project Structure

-   `two_echelon_vrp_example.py`: Main script to solve a 2E-CVRP instance and visualize the solution.
-   `two_echelon_vrp_sa.py`: Specialized version of the solver optimized for batch execution in sensitivity analysis.
-   `sensitivity_analyzer.py`: Orchestrates batch runs across multiple datasets and parameters.
-   `dataset_processer.py`: Utility module used by the solvers to parse benchmark dataset files and define default problem constraints (capacities, vehicle counts).
-   `visualisation.py`: Generates analytical plots (heatmaps, elasticity rankings) from sensitivity results.
-   `process_results.py`: Data cleaning and preprocessing for sensitivity analysis results.
-   `manilakbay-2E-EVRP-Instances-95ae99e/`: Contains benchmark datasets (Type X and Type Y).
-   `illustrations/`: Directory containing generated analytical visualizations.

## Prerequisites

-   Python 3.12+
-   [Gurobi Optimizer](https://www.gurobi.com/) (Requires a valid license)
-   Python Libraries:
    ```bash
    pip install gurobipy numpy matplotlib pandas seaborn
    ```

## Usage

### 1. Solving a Single Instance
To run the standard example and visualize the routes:
```bash
python two_echelon_vrp_example.py
```
This will generate `2echelon_vrp_solution.png`.

### 2. Running Sensitivity Analysis
To perform a broad analysis across multiple parameters and datasets:
```bash
python sensitivity_analyzer.py
```
Results will be saved to `sensitivity_analysis_results.csv`.

### 3. Data Cleaning
Before visualization, the raw results need to be preprocessed to handle infeasible solutions and normalize parameter names:
```bash
python process_results.py
```
This generates `cleaned_sensitivity_analysis_results.csv`, which is required by the visualization script.

### 4. Visualizing Results
After cleaning the results:
```bash
python visualisation.py
```
This will populate the `illustrations/` folder with heatmaps and elasticity rankings.

## Configuration

> [!IMPORTANT]
> Some scripts (`sensitivity_analyzer.py`, `two_echelon_vrp_example.py`) contain absolute file paths for datasets. Before running, ensure you update the `filepath` or `DATASETS` variables to point to the correct locations on your local machine.

## Datasets

The project uses the **Two-Echelon Electric Vehicle Routing Problem (2E-EVRP)** benchmark instances. Detailed information about the dataset structure can be found in [manilakbay-2E-EVRP-Instances-95ae99e/README.md](manilakbay-2E-EVRP-Instances-95ae99e/README.md).

---
*Developed as part of the Operations Optimisation course at TU Delft.*
