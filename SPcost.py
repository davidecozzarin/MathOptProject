import gurobipy as gp
from gurobipy import GRB

# =====================================================
# Title: Cost Optimization for Second-Level Delivery Routes in VRP
# Description: This script implements a subproblem (SP_cost)
#              to determine the minimum cost for deliveryman routes 
#              within a cluster as part of a Vehicle Routing Problem (VRP) 
#              with Time Windows and Multiple Deliverymen.
#              The model calculates the optimal second-level route
#              within a cluster, considering the cost minimization perspective.
# =====================================================

# Define global parameters
ML = 3  # Maximum number of deliverymen per vehicle  
cd = 1     # Cost coefficient for deliveryman routing distance     

def solve_sp_cost(i, data):
    # Create the model
    sp_model = gp.Model(f"SP_cost_{i}")

    Ai = data['Ai (set of arcs related to the second-level routes inside cluster i)']
    tij = data['tij (Travel time between first-level nodes i and j)']
    dihk = data['dihk (Distance between second-level nodes h and k of cluster i)']
    tihk = data['tihk (Travel time between second-level nodes h and k of cluster i)']
    N = data['N (set of cluster indices)']
    Ni = data['Ni (set of customer nodes in cluster i)']
    N0i = data['N0i (set of nodes including depot start and end)']
    Mihk = data['Mihk']
    sh = data['sh (Service time of customer h in cluster i)']
    ah = data['ah (Start of time window of customer h in cluster i)']
    bh = data['bh (End of time window of customer h in cluster i)']
    Nr = [i]
    Ar = [(0,i), (i,len(N)+1)]

    # Decision variables
    x = sp_model.addVars([(i, (h, k)) for i in Nr for (h, k) in Ai[i]], vtype=GRB.BINARY, name="y") # whetheer a delman travels from h to k in Ai within cluster i
    w = sp_model.addVars([(i, h) for i in Nr for h in N0i[i]], vtype=GRB.CONTINUOUS, name="w")  # wh time when service at node h in cluster i begins

    # Objective function
    sp_model.setObjective(cd * gp.quicksum(dihk[i][(h, k)] * x[i,(h, k)] for i in Nr for (h, k) in Ai[i]), GRB.MINIMIZE)

    # Constraints
    sp_model.addConstrs((gp.quicksum(x[i,(h, k)] for h in N0i[i] if (h, k) in Ai[i]) == 1 for i in Nr for k in Ni[i]), "c1")

    for i in Nr:
            for k in Ni[i]:
                sp_model.addConstr((gp.quicksum(x[i, (h, k)] for h in N0i[i] if (h, k) in Ai[i]) == gp.quicksum(x[i, (k, h)] for h in N0i[i] if (k, h) in Ai[i])), name=f"c2_{i}_{k}")

    for i in Nr:
        sp_model.addConstr((gp.quicksum(x[i, (0, h)] for h in Ni[i]) == gp.quicksum(x[i, (h, len(Ni[i]) + 1)] for h in Ni[i])), name=f"c3_{i}")
    
    for i in Nr:
        for (h, k) in Ai[i]:
            sp_model.addConstr(w[i, k] >= w[i, h] + sh[i][h] + tihk[i][(h, k)] - Mihk[i][(h, k)] * (1 - x[i, (h, k)]), name=f"c4_{i}_{h}_{k}")

    for i in Nr:
        sp_model.addConstr((gp.quicksum(x[i, (0, h)] for h in Ni[i]) <= ML), "c5")

    for (i,j) in Ar:
        if i in N and j in N:
            sp_model.addConstr((w[j, 0] >= w[i, len(Ni[i] + 1)] + tij[i,j]), "c6")
    
    for i in Nr:
        for h in N0i[i]:
            sp_model.addConstr((ah[i][h] <= w[i, h]), "c7_lower")
            sp_model.addConstr((w[i,h] <= bh[i][h]), "c7_upper")
    
    # Optimize model
    sp_model.optimize()

    # Print model status and optimization output
    print(f"Optimization status for cluster {i}: {sp_model.status}")
    if sp_model.status == GRB.OPTIMAL:
        return sp_model.ObjVal
    else:
        print(f"Model for cluster {i} is infeasible or unbounded.")
        return float('inf')
