import sys
import pandas as pd
import time, numpy as np
import pyomo.environ as pyo
from pyomo.environ import *
import math 
import random
import pandas as pd
import networkx as nx

n = 7 #• nodes to visit
N = n+1 # Problem size (depot + nodes to visit = 1+ n)

To = 0 # t_0

v = 1.00 # Speed
    #matrix of distances
dist = np.array([[0, 27.9, 54.6, 42.0, 56.5, 37.0, 30.9, 34.1], #Node 0 depot
                 [27.9, 0, 67.2, 25.6, 28.8, 48.4, 57.4, 21.6],
                 [54.6, 67.2, 0, 60.5, 95.8, 18.8, 60.4, 52.1],
                 [42.0, 25.6, 60.5, 0, 39.4, 43.1, 70.2, 12.2],
                 [56.5, 28.8, 95.8, 39.4, 0, 77.2, 84.5, 44.4],
                 [37.0, 48.4, 18.8, 43.1, 77.2, 0, 51.6, 34.0],
                 [30.9, 57.4, 60.4, 70.2, 84.5, 51.6, 0, 59.3],
                 [34.1, 21.6, 52.1, 12.2, 44.4, 34.0, 59.3, 0]]) # Node 7

t = np.zeros((N,N)) #matrix of travel time t_ij

range_n = range(0,n)
range_N = range(0,N)

t = dist/v # Compute travel time t_ij

        
LB = np.array([0, 0, 0, 0, 0, 0, 0, 0])
UB = np.array([1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000])

#Create Model
model = pyo.ConcreteModel()

#Define variables
model.T = pyo.Var(range(0,N+1), # index i
                  bounds = (0,None),
                  initialize=0)

model.y = pyo.Var(range(0,N), # index i
                  range(0,N), # index j
                  within = Binary,
                  initialize=0)

T = model.T
y = model.y

# Constraints
model.C1 = pyo.ConstraintList() 
for i in range(1,n+1):
    model.C1.add(expr = T[i] - T[0] >= t[0][i])
    
# Constraints
model.C2 = pyo.ConstraintList() 
for i in range(1,n+1):
    model.C2.add(expr = T[n+1] - T[i]  >= t[i][0])

model.C3 = pyo.ConstraintList() 
for i in range_N:
    for j in range_N:
        if i!= j:
            M = UB[i]-LB[j]+t[i][j]
            model.C3.add(expr = T[i] - T[j] +y[i,j]*M >= t[i][j])
    
model.C4 = pyo.ConstraintList() 
for i in range_N:
    for j in range_N:
        if i!= j:
            M = UB[i]-LB[j]+t[i][j]
            model.C4.add(expr = T[j] - T[i] +(1-y[i,j])*M >= t[i][j])
            
model.C5 = pyo.ConstraintList() 
for i in range(1,n+1):
    model.C5.add(expr = T[i] >= LB[i])

model.C6 = pyo.ConstraintList() 
for i in range(1,n+1):
    model.C6.add(expr = T[i] <= UB[i])

model.C7 = pyo.Constraint(expr= T[0] == 0)


# Define Objective Function
model.obj = pyo.Objective(expr = T[n+1]-T[0], 
                          sense = minimize)

begin = time.time()
opt = SolverFactory('cplex')
results = opt.solve(model)

deltaT = time.time() - begin # Compute Exection Duration

model.pprint()

sys.stdout = open("Time_Constrained_Symmetric_Traveling_Salesman_Problem_Results.txt", "w") #Print Results on a .txt file

print('Time =', np.round(deltaT,2))

if (results.solver.status == SolverStatus.ok) and (results.solver.termination_condition == TerminationCondition.optimal):

    print('Makespan (Obj value) =', pyo.value(model.obj))
    print('Solver Status is =', results.solver.status)
    print('Termination Condition is =', results.solver.termination_condition)
    print(" " )
    for i in range(0,N+1):
      print('---> Node',i, 'is visited at time', round(pyo.value(T[i]),2) , ' (t[',i,'])')
elif (results.solver.termination_condition == TerminationCondition.infeasible):
   print('Model is unfeasible')
  #print('Solver Status is =', results.solver.status)
   print('Termination Condition is =', results.solver.termination_condition)
else:
    # Something else is wrong
    print ('Solver Status: ',  result.solver.status)
    print('Termination Condition is =', results.solver.termination_condition)
    
sys.stdout.close()