import gurobipy as gp
from gurobipy import GRB

# =====================================================
# Title: Time-Minimization Subproblem for VRP with Deliverymen
# Description: This script implements a subproblem (SP_time)
#              to determine the minimum time required for deliveryman routes 
#              within a cluster as part of the Vehicle Routing Problem (VRP)
#              with Time Windows and Multiple Deliverymen.
#              The model focuses on minimizing the total time spent
#              in a cluster considering the deliveryman routes.
# =====================================================

# Define global parameters
ML = 3  # Maximum number of deliverymen per vehicle  
cd = 1     # Cost coefficient for deliveryman routing distance 

def solve_sp_time(i, l, data):
    # Create the model
    sp_model = gp.Model(f"SP_time_{i}_{l}")

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
    x = sp_model.addVars([(i, (h, k)) for i in N for (h, k) in Ai[i]], vtype=GRB.BINARY, name="y") 
    w = sp_model.addVars([(i, h) for i in N for h in N0i[i]], vtype=GRB.CONTINUOUS, name="w") 

    # Objective function
    sp_model.setObjective(w[i, len(Ni[i]) + 1] - w[i, 0], GRB.MINIMIZE)

    # Constraints
    sp_model.addConstrs((gp.quicksum(x[i,(h, k)] for h in N0i[i] if (h, k) in Ai[i]) == 1 for i in Nr for k in Ni[i]), "c1")

    for i in Nr:
            for k in Ni[i]:
                sp_model.addConstr((gp.quicksum(x[i, (h, k)] for h in N0i[i] if (h, k) in Ai[i]) == gp.quicksum(x[i, (k, h)] for h in N0i[i] if (k, h) in Ai[i])), name=f"c2_{i}_{k}")

    for i in Nr:
        sp_model.addConstr((gp.quicksum(x[i, (0, h)] for h in Ni[i]) == gp.quicksum(x[i, (h, len(Ni[i]) + 1)] for h in Ni[i])), name=f"c3_{i}")
    
    sp_model.addConstrs((w[i, k] >= w[i, h] + sh[i][h] + tihk[i][(h, k)] - Mihk[i][(h, k)] * (1 - x[i,(h, k)]) for i in Nr for (h, k) in Ai[i]), "c4")

    for i in Nr:
        sp_model.addConstr((gp.quicksum(x[i, (0, h)] for h in Ni[i]) <= l), "c5")

    for (i,j) in Ar:
        if i in N and j in N:
            sp_model.addConstr((w[j, 0] >= w[i, len(Ni[i] + 1)] + tij[i,j]), "c6")
    
    for i in Nr:
        sp_model.addConstrs((ah[i][h] <= w[i, h] for h in N0i[i]), "c7_lower")
        sp_model.addConstrs((w[i,h] <= bh[i][h] for h in N0i[i]), "c7_upper")
    
    # Optimize model
    sp_model.optimize()

    if sp_model.status == GRB.OPTIMAL:
        return sp_model.ObjVal, True
    else:
        return float('inf'), False
    