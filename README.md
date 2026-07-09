# Time Constrained Symmetric Traveling Salesman Problem (STSPTW)

A **Mixed Integer Linear Programming (MILP)** model in **Python** for the **Time Constrained Symmetric Traveling Salesman Problem**, built with the **[Pyomo](http://www.pyomo.org/)** optimization framework and solved via the **IBM ILOG CPLEX** solver.

## Overview

The Time Constrained Symmetric Traveling Salesman Problem (STSPTW) is a variant of the classic Traveling Salesman Problem in Operations Research. A traveler starts from a depot, visits a set of nodes exactly once, and returns to the depot — respecting a time window at each node. The problem is called **symmetric** because the travel time between any two nodes is the same in both directions: $t_{ij} = t_{ji}$.

The objective is **makespan minimization**: finding the route that minimizes the total duration of the tour, measured as the difference between the tour completion time and the departure time from the depot.

## Repository Contents

| File | Description |
|---|---|
| `Time_Constrained_Symmetric_Traveling_Salesman_Problem.py` | Python script implementing and solving the STSPTW via Pyomo and CPLEX |
| `Time_Constrained_Symmetric_Traveling_Salesman_Problem_Results.txt` | Solver output: status, optimal makespan, and visit times for each node |
| `Time Constrained Symmetric Traveling Salesman Problem _Math Formulation.pdf` | Mathematical formulation of the problem |

## Mathematical Formulation

### Sets

Let $G = (V, A)$ be a complete graph where:

- $V = \{0, 1, \dots, n\}$ = set of nodes (node $0$ is the depot)
- $A = \{(i,j) : i \neq j;\ i, j \in V\}$ = set of undirected arcs

Every node $i \in V$ has an associated time window $TW_i = [l_i,\ u_i]$, where $l_i$ is the release time and $u_i$ is the deadline. Each arc $(i,j) \in A$ has an associated travel time $t_{ij} \ge 0$, derived from a distance $d_{ij}$ and a speed $v$: $t_{ij} = d_{ij} / v$. The problem is symmetric since $t_{ij} = t_{ji}$ for all arcs.

### Parameters

- $l_i$ = lower bound of the time window for node $i$; $\forall i \in V$
- $u_i$ = upper bound of the time window for node $i$; $\forall i \in V$
- $t_{ij}$ = travel time from node $i$ to node $j$; $\forall (i,j) \in A$
- $M_{ij} = u_i - l_j + t_{ij}$ = big-M coefficient for the time-ordering constraints

### Variables

- $t_i$ = time instant at which node $i$ is visited; $\forall i \in V$
- $t_0$ = departure time from the depot (fixed to $0$)
- $t_{n+1}$ = time at which the tour is completed (makespan)
- $y_{ij}$ = binary routing variable:

$$
y_{ij} = \begin{cases} 1 & \text{if node } j \text{ follows node } i \text{ in the tour} \\ 0 & \text{otherwise} \end{cases}
$$

### Objective Function

**(1)** — Minimize the total tour duration

$$
\min \ t_{n+1} - t_0
$$

### Constraints

**(2)** — Visit time at each node is at least the travel time from the depot

$$
t_i - t_0 \ge t_{0i} \qquad i = 1, 2, \dots, n
$$

**(3)** — The makespan is at least the return time from each node to the depot

$$
t_{n+1} - t_i \ge t_{i0} \qquad i = 1, 2, \dots, n
$$

**(4)** — Time ordering (forward direction, big-M formulation)

$$
t_i - t_j + M_{ij} \cdot y_{ij} \ge t_{ij} \qquad \forall\, i, j = 0, 1, \dots, n,\ i \neq j
$$

**(5)** — Time ordering (reverse direction, big-M formulation)

$$
t_j - t_i + (1 - y_{ij}) \cdot M_{ij} \ge t_{ij} \qquad \forall\, i, j = 0, 1, \dots, n,\ i \neq j
$$

**(6)** — Time window feasibility at each node (lower bound)

$$
t_i \ge l_i \qquad i = 1, 2, \dots, n
$$

**(7)** — Time window feasibility at each node (upper bound)

$$
t_i \le u_i \qquad i = 1, 2, \dots, n
$$

**(8)** — Departure time from the depot is fixed

$$
t_0 = 0
$$

**(9)** — Binary routing variables

$$
y_{ij} \in \{0,1\} \qquad \forall\, (i,j) \in A
$$

**(10)** — Non-negative visit times

$$
t_i \ge 0 \qquad \forall\, i = 0, 1, \dots, n+1
$$

where $M_{ij} = u_i - l_j + t_{ij}$.

> **Note on symmetry:** Constraints (4) and (5) together model undirected arcs. For any pair $(i,j)$, exactly one of the two directions is active: if $y_{ij} = 1$ (node $j$ follows $i$) then constraint (4) enforces $t_i + t_{ij} \le t_j$, while constraint (5) becomes inactive; if $y_{ij} = 0$ (node $i$ follows $j$) the roles are reversed. This differs from the asymmetric formulation, which uses separate directed flow-conservation constraints.

A copy of this formulation is also available as a standalone PDF in this repository.

## Example Instance

The script ships with a sample instance featuring:

- **8 nodes** (1 depot + 7 nodes to visit), with the following symmetric distance matrix:

$$
D = \begin{pmatrix}
0 & 27.9 & 54.6 & 42.0 & 56.5 & 37.0 & 30.9 & 34.1 \\
27.9 & 0 & 67.2 & 25.6 & 28.8 & 48.4 & 57.4 & 21.6 \\
54.6 & 67.2 & 0 & 60.5 & 95.8 & 18.8 & 60.4 & 52.1 \\
42.0 & 25.6 & 60.5 & 0 & 39.4 & 43.1 & 70.2 & 12.2 \\
56.5 & 28.8 & 95.8 & 39.4 & 0 & 77.2 & 84.5 & 44.4 \\
37.0 & 48.4 & 18.8 & 43.1 & 77.2 & 0 & 51.6 & 34.0 \\
30.9 & 57.4 & 60.4 & 70.2 & 84.5 & 51.6 & 0 & 59.3 \\
34.1 & 21.6 & 52.1 & 12.2 & 44.4 & 34.0 & 59.3 & 0
\end{pmatrix}
$$

- Travel speed: $v = 1$ (travel time equals distance)
- Time windows: lower bounds $l_i = 0$, upper bounds $u_i = 1000$ for all nodes
- Departure time from the depot: $t_0 = 0$

## Requirements

Install the required Python packages via pip:

```bash
pip install pyomo numpy pandas networkx
```

**IBM ILOG CPLEX** must also be installed separately on your system. An academic license is available free of charge through the [IBM Academic Initiative](https://www.ibm.com/academic).

## Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/Diego-Fabbri/Time_Constrained_Symmetric_Traveling_Salesman_Problem_Py.git
   cd Time_Constrained_Symmetric_Traveling_Salesman_Problem_Py
   ```

2. Run the script:
   ```bash
   python Time_Constrained_Symmetric_Traveling_Salesman_Problem.py
   ```

## Output

When executed, the script:
- Builds the MILP model using Pyomo's `ConcreteModel`
- Solves it via CPLEX and prints the full model structure to the console
- Records the solver execution time
- Writes the results to `Time_Constrained_Symmetric_Traveling_Salesman_Problem_Results.txt`, including:
  - Execution time in seconds
  - Solver status and termination condition
  - Optimal makespan $t_{n+1} - t_0$ (objective value)
  - Visit time $t_i$ at each node in the tour
