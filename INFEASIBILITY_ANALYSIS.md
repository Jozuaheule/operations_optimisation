# Two-Echelon VRP Model: Infeasibility Analysis

## Executive Summary

The original model implementation has **4 major infeasibility issues** that make it impossible to find a solution. These stem from conflicting constraints and circular dependencies in the mathematical formulation.

---

## Problem Overview

In 2E-CVRP:
- **Depot** → **Satellites** (1st level, larger vehicles)
- **Satellites** → **Customers** (2nd level, smaller vehicles)
- **Objective**: Minimize total transportation cost
- **Constraints**: Vehicle capacities, satellite capacities, demand satisfaction

---

## Why the Original Model is Infeasible

### **ISSUE 1: Conflicting Flow Conservation Constraints** ❌

**Location**: Constraints (9) and (10) in the original model

**The Problem**:
```python
# Constraint (9) - Flow balance at each customer for EVERY satellite k
for k in range(n_satellites):
    for j in range(n_customers):
        inflow - outflow = z[k,j] * demand[j]
```

**Why it's wrong**:
- This constraint applies to **ALL** satellites, not just the one serving customer j
- If customer j is assigned to satellite 0 (z[0,j] = 1), then:
  - At satellite 0: inflow - outflow = demand[j] ✓ (correct)
  - At satellite 1: inflow - outflow = 0 (because z[1,j] = 0)
  - But constraint (9) still requires flow balance at satellite 1 for customer j
  - This creates impossible flow requirements

**Example**:
```
Customer C1 assigned to Satellite S0 (z[0,1] = 1, z[1,1] = 0)
Demand = 10 units

At S0: Flow_in - Flow_out = 10 ✓  (makes sense)
At S1: Flow_in - Flow_out = 0   (but no flow should exist!)
```

The constraint forces flows to exist even where they shouldn't, creating conflicts.

---

### **ISSUE 2: Over-Constrained Linking Between Levels** ❌

**Location**: Constraint (8) - linking y and z variables

**The Problem**:
```python
# Constraint (8)
y[k,i,j] <= z[k,j]  # If arc (i,j) used from satellite k, customer j must be assigned to k
```

**Why it's wrong**:
- This seems logical, but combined with flow conservation (Issue 1), it's too restrictive
- Flow variables need flexibility to balance flows across the network
- This constraint prevents necessary "pass-through" flows

**The Circular Dependency**:
```
1. y[k,i,j] needs z[k,j] = 1 to be non-zero
2. But z[k,j] is determined by optimization
3. Flow balance requires y values before z is known
4. Circular dependency → infeasible
```

---

### **ISSUE 3: Incorrect Satellite Demand Calculation** ❌

**Location**: Constraint (11) - linking 1st and 2nd levels

**The Problem**:
```python
# Constraint (11) - 1st level flow to satellite k must equal its demand
for k in range(n_satellites):
    demand_at_k = sum(demand[j] * z[k,j] for j in customers)
    Flow_into_k - Flow_out_of_k = demand_at_k
```

**Why it's wrong**:
- Uses z[k,j] (a **decision variable**) to calculate demand
- Creates a non-linear constraint (multiplying variables: z[k,j] * another variable)
- The flow is supposed to **determine** the assignment, not the other way around
- This is a chicken-and-egg problem

**What happens**:
```
Satellite S0 demand = demand[0]*z[0,0] + demand[1]*z[0,1] + ...
                    = 10*z[0,0] + 8*z[0,1] + 12*z[0,2] + ...

But z values are UNKNOWN! They're what we're solving for!
This creates a bilinear constraint (variable × variable)
Gurobi sees this as infeasible in a linear model.
```

---

### **ISSUE 4: Incorrect Return Flow Constraints** ❌

**Location**: Constraints (11) and (12)

**The Problem**:
```python
# Constraint (11) - No flow returns to depot from satellites
sum(Q1[i, depot] for i in satellites) = 0

# Constraint (12) - No flow returns to satellites from customers
sum(Q2[j, satellite_k, satellite_k] for j in customers) = 0
```

**Why it's wrong**:
- In reality, vehicles **DO** return to depot/satellites after deliveries
- These constraints say "vehicles never come back"
- This violates the basic VRP requirement that routes are **cycles**

**What should happen**:
```
Correct routing:
Depot → Satellite 1 → Satellite 2 → DEPOT (vehicle returns!)
Satellite 1 → Customer A → Customer B → SATELLITE 1 (vehicle returns!)

Wrong (what the model forces):
Depot → Satellite 1 → Satellite 2 → ??? (vehicle disappears!)
```

---

## Detailed Example of Infeasibility

Let's trace through a simple example:

### Setup:
- 1 Depot, 2 Satellites (S0, S1), 2 Customers (C0, C1)
- Demands: C0=10, C1=10
- Vehicle capacity (1st level): 50
- Vehicle capacity (2nd level): 20

### Attempted Solution:
1. **Assignment**:
   - C0 → S0 (z[0,0] = 1, z[1,0] = 0)
   - C1 → S1 (z[0,1] = 0, z[1,1] = 1)

2. **Flow at S0 for C0** (correct):
   - Flow in - Flow out = 10 * z[0,0] = 10 ✓

3. **Flow at S0 for C1** (incorrect due to Issue 1):
   - Flow in - Flow out = 10 * z[0,1] = 10 * 0 = 0
   - But S0 isn't serving C1!
   - The constraint still applies and creates impossible requirements

4. **1st Level Demand** (Issue 3):
   - Demand at S0 = 10*z[0,0] + 10*z[0,1] = 10*1 + 10*0 = 10
   - But z values are variables, not constants!
   - Gurobi cannot handle: Flow = 10*z[0,0] (bilinear)

5. **Result**: **INFEASIBLE** ❌

---

## How to Fix the Model

### **Fix 1: Separate Assignment from Flow**

**Before** (wrong):
```python
# Flow depends on assignment
inflow - outflow = demand[j] * z[k,j]  # Bilinear!
```

**After** (correct):
```python
# First determine assignment
sum(z[k,j] for k in satellites) == 1  # Customer assigned to one satellite

# Then enforce flow only where assigned
if z[k,j] == 1:
    inflow - outflow >= 1  # Must visit if assigned
else:
    inflow == 0  # No flow if not assigned
```

### **Fix 2: Simplify Flow Constraints**

**Before** (wrong):
```python
# Complex flow balance with circular dependencies
for all satellites k:
    for all customers j:
        inflow[k,j] - outflow[k,j] = demand[j] * z[k,j]
```

**After** (correct):
```python
# Simple degree constraints
for customers j assigned to satellite k:
    in_degree[j] == out_degree[j]  # What goes in must come out
    in_degree[j] >= 1  # Must be visited
```

### **Fix 3: Correct Capacity Constraints**

**Before** (wrong):
```python
# Using variable demand
flow_to_satellite_k == sum(demand[j] * z[k,j] for j in customers)
```

**After** (correct):
```python
# Use Big-M formulation or MTZ subtour elimination
for route from satellite k:
    total_load <= vehicle_capacity
```

### **Fix 4: Remove Return Flow Constraints**

**Before** (wrong):
```python
sum(Q1[i, depot]) == 0  # Vehicles never return!
```

**After** (correct):
```python
# Let vehicles return naturally through flow conservation
# Flow in == Flow out at all nodes (including depot)
```

---

## Verification Steps

To verify the model is feasible, check:

### ✓ **Capacity Check**:
```
Total demand: 20 units
1st level capacity: 2 vehicles × 50 = 100 ✓
2nd level capacity: 2 vehicles × 20 = 40 ✓
```

### ✓ **Route Feasibility**:
```
Customers: 2
Routes available: 2 satellites × 2 routes/satellite = 4 ✓
Minimum needed: 2 (one per customer) ✓
```

### ✓ **No Circular Dependencies**:
```
Assignment (z) → determines → Demand at satellites → determines → 1st level routes
No feedback loops ✓
```

### ✓ **Constraint Consistency**:
```
Flow balance: in-degree = out-degree ✓
No conflicting requirements ✓
Vehicles can return ✓
```

---

## Summary of Root Causes

| Issue | Root Cause | Impact | Fix |
|-------|-----------|--------|-----|
| **Issue 1** | Flow constraints apply to ALL satellites | Impossible flow requirements | Apply only to assigned satellite |
| **Issue 2** | Over-linking y and z variables | Circular dependencies | Separate assignment and routing |
| **Issue 3** | Bilinear constraints (z × demand) | Non-linear in linear model | Use Big-M or pre-compute |
| **Issue 4** | No return flows allowed | Violates cycle requirement | Remove these constraints |

---

## Corrected Model Structure

```python
# PHASE 1: Assignment
for j in customers:
    sum(z[k,j] for k in satellites) == 1  # Assign each customer

# PHASE 2: 1st Level Routing
for k in satellites:
    vehicles_to_k * capacity >= sum(demand[j] * z[k,j] for j in customers)

# PHASE 3: 2nd Level Routing
for k in satellites:
    for j in customers where z[k,j] == 1:
        in_degree[j] == out_degree[j]  # Flow balance
        in_degree[j] >= 1  # Must visit

# PHASE 4: Capacity (MTZ subtour elimination)
for routes:
    cumulative_load <= vehicle_capacity
```

---

## Running the Corrected Model

The `verify_model.py` script:
1. ✓ Diagnoses all 4 issues
2. ✓ Builds a corrected model
3. ✓ Solves successfully
4. ✓ Explains each fix

**Result**: The corrected model is **FEASIBLE** and finds optimal solutions! ✓✓✓

---

## Key Takeaways

1. **Circular dependencies** kill feasibility - break them by separating decisions
2. **Bilinear constraints** (variable × variable) don't work in LP/MIP models
3. **Over-constraining** creates conflicts - only enforce what's necessary
4. **Test with small instances** to identify issues before scaling up
5. **Check IIS** (Irreducible Inconsistent Subsystem) when infeasible

---

## References

- Original paper: Gonzalez-Feliu et al. (2008)
- Gurobi documentation on infeasibility: https://www.gurobi.com/documentation/
- MTZ subtour elimination: Miller-Tucker-Zemlin (1960)
