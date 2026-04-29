# Sensitivity Analysis Plan for the Two-Echelon VRP

This document outlines a plan for conducting a sensitivity analysis on the two-echelon vehicle routing problem (VRP) model.

## 1. Objective

The primary objective of this sensitivity analysis is to understand how the solution of the two-echelon VRP model changes in response to variations in key input parameters. This will help in assessing the robustness of the model and identifying the most critical factors and bottlenecks that influence the outcome.

## 2. Methodology

The sensitivity analysis will be conducted by systematically varying one parameter at a time while keeping others constant. This will be done for each of the three provided datasets.

### 2.1. Baseline Measurement

First, we will establish a baseline by running the model with the original, unmodified datasets (`Dataset1_example.dat`, `Dataset2_example.dat`, and `Dataset3_example.dat`). The results from these runs will serve as a reference point for comparison.

### 2.2. Parameter Variation

We will focus on the following key parameters for the sensitivity analysis:

*   **Vehicle Capacity:** We will vary the capacity of the vehicles to observe its impact on the routing and total cost.
*   **Customer Demand:** We will adjust the demand of customers to see how it affects the delivery routes and feasibility.
*   **Number of Vehicles:** We will change the number of available vehicles at the depots to analyze its effect on the solution.

For each parameter, we will test a range of values (e.g., ±10%, ±20% of the original value).

### 2.3. Automated Analysis

To facilitate the analysis, we will develop a Python script (`sensitivity_analyzer.py`). This script will:

1.  Read a specified dataset.
2.  Modify one parameter in the dataset according to a predefined range.
3.  Execute the `two_echelon_vrp_example.py` script with the modified data.
4.  Capture and store the output from the model.

### 2.4. Data Collection

For each run, we will collect the following metrics:

*   **Input Dataset:** The name of the dataset used.
*   **Parameter Modified:** The parameter that was varied.
*   **Parameter Value:** The value of the parameter for that run.
*   **Total Cost:** The objective function value from the model.
*   **Execution Time:** The time taken to find a solution.
*   **Feasibility:** Whether a feasible solution was found.

### 2.5. Results Storage

The collected data will be stored in a CSV file named `sensitivity_analysis_results.csv`. This will allow for easy analysis and visualization of the results. Each row in the CSV will represent a single experiment with its corresponding inputs and outputs.

## 3. Implementation Steps

1.  **Create `sensitivity_analyzer.py`:** Develop the script to automate the testing process.
2.  **Modify `two_echelon_vrp_example.py`:** Modify the main script to:
    *   Allow parameters to be passed as arguments.
    *   Output results in a structured format.
3.  **Run the Analysis:** Execute the `sensitivity_analyzer.py` script.
4.  **Analyze Results:** Analyze the `sensitivity_analysis_results.csv` file to draw conclusions about parameter sensitivity.
