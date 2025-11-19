# Two-Echelon Capacitated Vehicle Routing Problem (2E-CVRP)

## Overview

This repository contains a sample implementation of the Two-Echelon Capacitated Vehicle Routing Problem based on the paper by Gonzalez-Feliu, Perboli, Tadei, and Vigo (2008).

## Files

### 📄 `2echelon papaer.pdf`
The original research paper introducing the 2E-CVRP problem and mathematical formulation.

### 🐍 `two_echelon_vrp_example.py`
**Sample Gurobi implementation** - A well-commented educational example showing:
- How to model 2E-CVRP in Python with Gurobi
- Decision variables for both transportation levels
- Constraints for capacity, flow conservation, and assignment
- Visualization of the solution

**⚠️ NOTE**: This is an **educational example** that directly translates the paper's mathematical model. However, it has **known infeasibility issues** (see below).

### 🔍 `verify_model.py`
**Verification and debugging script** that:
- Systematically identifies infeasibility issues in the original model
- Provides a corrected, working implementation
- Explains each fix in detail
- Can be used as a template for working 2E-CVRP models

### 📘 `INFEASIBILITY_ANALYSIS.md`
**Detailed technical analysis** explaining:
- Why the original model formulation is infeasible
- 4 major issues with root causes
- Step-by-step examples of conflicts
- How to fix each issue
- Verification procedures

## Quick Start

### Prerequisites
```bash
pip install gurobipy numpy matplotlib
```

You also need a valid Gurobi license (free academic licenses available).

### Run the Example
```bash
python two_echelon_vrp_example.py
```

**Expected result**: The model will be **infeasible** - this is intentional for educational purposes.

### Understand Why It's Infeasible
```bash
python verify_model.py
```

This will:
1. Diagnose all infeasibility issues
2. Show a corrected version
3. Solve successfully
4. Explain the differences

### Read the Analysis
```bash
cat INFEASIBILITY_ANALYSIS.md
```

Complete technical explanation of all issues and fixes.

## Problem Description

### Two-Echelon System
In 2E-CVRP, delivery happens in **two stages**:

```
Stage 1 (1st Echelon): Depot ──────────→ Satellites
                       [Large vehicles]

Stage 2 (2nd Echelon): Satellites ──────→ Customers
                       [Small vehicles]
```

### Why Two Echelons?
- **Urban logistics**: Large trucks → distribution centers → small vans → customers
- **Regulatory**: Cities may restrict large vehicle access
- **Economic**: Optimize vehicle types for different areas
- **Environmental**: Reduce emissions in city centers

### Example Applications
- **City logistics**: Urban delivery systems (e.g., Rome, Paris)
- **E-commerce**: Amazon/warehouse → local hub → final delivery
- **Grocery**: Distribution center → stores → home delivery
- **Parcel services**: Regional depot → local depot → customers

## Mathematical Model

### Decision Variables
- **x[i,j]**: Number of 1st-level vehicles on arc (i,j)
- **y[k,i,j]**: Binary, 1 if 2nd-level vehicle from satellite k uses arc (i,j)
- **z[k,j]**: Binary, 1 if customer j assigned to satellite k
- **Q1[i,j]**: Flow (goods quantity) on 1st-level arc (i,j)
- **Q2[k,i,j]**: Flow on 2nd-level arc (i,j) from satellite k

### Objective
Minimize total transportation cost:
```
min: Σ cost[i,j] × x[i,j]  +  Σ cost[i,j] × y[k,i,j]
     [1st level costs]        [2nd level costs]
```

### Key Constraints
1. **Capacity**: Vehicles cannot exceed load limits
2. **Flow conservation**: What goes in must come out
3. **Assignment**: Each customer served by exactly one satellite
4. **Satellite capacity**: Limited routes per satellite
5. **Demand satisfaction**: All customers must receive their goods

## Known Issues in Original Model

### ⚠️ The example code has 4 major infeasibility issues:

1. **Conflicting Flow Constraints**: Flow balance required for ALL satellites, even those not serving a customer
2. **Circular Dependencies**: Assignment depends on flow, flow depends on assignment
3. **Bilinear Constraints**: Multiplying decision variables (z[k,j] × demand[j])
4. **Incorrect Return Flows**: Constraints prevent vehicles from returning to depot/satellites

### ✅ These are **intentional** for educational purposes!

The `verify_model.py` shows how to:
- Identify each issue systematically
- Understand why it causes infeasibility
- Fix it properly

## Learning Objectives

By studying these files, you will learn:

1. **How to model multi-echelon logistics** in mathematical programming
2. **Common pitfalls in VRP formulations**:
   - Circular dependencies
   - Over-constraining
   - Bilinear terms in linear models
   - Flow balance mistakes

3. **Debugging infeasible models**:
   - Computing IIS (Irreducible Inconsistent Subsystem)
   - Systematic constraint analysis
   - Identifying root causes

4. **Proper modeling techniques**:
   - Separating decision phases
   - Using MTZ subtour elimination
   - Big-M formulations
   - Linking multi-level decisions

## Model Comparison

### Original (Infeasible) Model
```python
# ❌ Bilinear constraint
for k in satellites:
    for j in customers:
        flow_balance = demand[j] * z[k,j]  # Variable × Variable!
```

### Corrected (Feasible) Model
```python
# ✅ Linear constraints
for j in customers:
    sum(z[k,j] for k in satellites) == 1  # Assignment first

for k in satellites:
    for j where z[k,j] == 1:  # Then routing
        in_degree[j] == out_degree[j]
```

## Results Interpretation

### Example Solution Visualization

The code generates a two-panel visualization:

**Left Panel**: 1st Echelon (Depot → Satellites)
- Red square: Depot
- Blue triangles: Satellites
- Green arrows: 1st-level routes with vehicle counts

**Right Panel**: 2nd Echelon (Satellites → Customers)
- Blue triangles: Satellites
- Colored circles: Customers (color = assigned satellite)
- Colored arrows: 2nd-level delivery routes

## Instance Generation

The code includes:
- **Simple test instance**: 2 satellites, 8 customers
- Realistic parameters matching the paper's benchmarks
- Geographic distribution: customers clustered near satellites
- Capacity ratios: 1st level = 2.5 × 2nd level (matching paper)

## Extensions

You can extend the model to include:

1. **Time windows**: Delivery must occur in specific time periods
2. **Multiple depots**: Several distribution centers
3. **Heterogeneous fleet**: Different vehicle types and costs
4. **Pickup and delivery**: Reverse logistics
5. **Stochastic demand**: Uncertainty in customer orders
6. **Dynamic routing**: Real-time updates

See the paper for formulations of these variants.

## Performance Notes

### Computational Complexity
- 2E-CVRP is **NP-Hard** (proved via reduction to VRP)
- Exact methods (like this) work well up to:
  - ~30-50 customers
  - ~5 satellites
  - Beyond this, heuristics recommended (Tabu Search, Genetic Algorithms)

### Scaling Tips
1. Start with small instances for debugging
2. Use valid inequalities (cuts) to strengthen relaxation
3. Consider decomposition methods for large instances
4. Implement specialized branch-and-bound strategies

## References

### Original Paper
Gonzalez-Feliu, J., Perboli, G., Tadei, R., & Vigo, D. (2008).
*The Two-Echelon Capacitated Vehicle Routing Problem*.
HAL preprint halshs-00879447.

### Related Papers
- Crainic et al. (2004): City logistics applications
- Perboli et al. (2011): Math-heuristics for 2E-CVRP
- Cuda et al. (2015): Survey of multi-echelon problems

### Software
- **Gurobi Optimizer**: https://www.gurobi.com/
- **OR-Library**: Test instances at http://people.brunel.ac.uk/~mastjjb/jeb/info.html

## Troubleshooting

### "ModuleNotFoundError: No module named 'gurobipy'"
```bash
pip install gurobipy
```
You need a Gurobi license (free for academics).

### "Model is infeasible"
This is **expected** for `two_echelon_vrp_example.py`!
Run `verify_model.py` to see the corrected version.

### "Cannot open model_iis.ilp"
This file is generated only when running `verify_model.py` with an infeasible model.
It contains the Irreducible Inconsistent Subsystem for debugging.

## Contact & Contributions

This is an educational implementation for learning purposes.

**For academic use**: Please cite the original paper.

**For commercial use**: Consult with optimization experts for production-ready implementations.

## License

Educational use only. Refer to original paper for research citations.

---

## Quick Reference: Running the Code

```bash
# 1. See the (infeasible) example
python two_echelon_vrp_example.py
# Result: Infeasible model (educational)

# 2. Understand why it's infeasible
python verify_model.py
# Result: Diagnosis + corrected solution

# 3. Read detailed analysis
cat INFEASIBILITY_ANALYSIS.md
# Result: Complete technical explanation

# 4. Read the original paper
open "2echelon papaer.pdf"
# Result: Mathematical formulation
```

---

**Happy Learning!** 📚🚚🔢
