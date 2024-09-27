import gurobipy as gp
from gurobipy import GRB
import itertools
import math

# =====================================================
# Title: Master Problem Formulation for VRP
# Description: This script defines the Master Problem (MP) for a Vehicle Routing Problem (VRP) with multiple deliverymen.
# =====================================================

# Define global parameters
ML = 3  # Maximum number of deliverymen per vehicle  
fv = 1000  # Fixed cost of using a vehicle
fd = 100   # Additional cost per deliveryman
cv = 10    # Cost coefficient for vehicle routing distance

def define_rmp(data):
    N = data['N (set of cluster indices)']
    N0 = data['N0 (Set of nodes including depot start and end)'] 
    Ni = data['Ni (set of customer nodes in cluster i)']
    N0i = data['N0i (set of nodes including depot start and end)']
    L = range(1, ML + 1)  
    Q = data['vehicle_capacity']
    qi = data['qi (Demand of cluster i)']
    A = data['A (Set of arcs for first-level routes)']
    dij = data['dij (Distance between first-level nodes i and j)']
    tij = data['tij (Travel time between first-level nodes i and j)']
    Mij = data['Mij']
    ah = data['ah (Start of time window of customer h in cluster i)']
    bh = data['bh (End of time window of customer h in cluster i)']
    eil = data['eil']
    mi = data['mi']
    eta_ = data['eta_']
    
    # Create the model
    model = gp.Model("RMP")

    # Decision variables
    x = model.addVars([((i, j), l) for (i, j) in A for l in L], vtype=GRB.BINARY, name="x")
    eta = model.addVars(N, vtype=GRB.CONTINUOUS, name="eta")
    w = model.addVars([(i, h) for i in N for h in N0i[i]], vtype=GRB.CONTINUOUS, name="w") 

    # Objective function
    model.setObjective(
        gp.quicksum((fv + l * fd) * x[(0, j), l] for j in N for l in L) +
        cv * gp.quicksum(dij[(i, j)] * x[(i, j), l] for (i, j) in A for l in L) +
        gp.quicksum(eta[i] for i in N),
        GRB.MINIMIZE
    )

    # Constraint (2): Ensure exactly one delivery man visits each cluster
    for j in N:
        model.addConstr((gp.quicksum(x[(i, j), l] for i in N0 if (i, j) in A for l in L) == 1), name=f"c2_{j}")

    # Constraint (3): Flow balance constraints
    for j in N:
        for l in L:
            model.addConstr((gp.quicksum(x[(i, j), l] for i in N0 if (i, j) in A) == gp.quicksum(x[(j, i), l] for i in N0 if (j, i) in A)), name=f"c3_{j}_{l}")

    # Constraint (4): Depot flow constraints
    for l in L:
        model.addConstr((gp.quicksum(x[(0, i), l] for i in N) == gp.quicksum(x[(i, len(N) + 1), l] for i in N)), name=f"c4_{l}")

    
    # Constraint (5): Time constraints at first-level nodes
    # Replaced by (34)
    # for (i, j) in A:
    #    model.addConstr((u[j] >= u[i] + data['qi (Demand of cluster i)'][j] - Q * (1 - gp.quicksum(x[(i, j), l] for l in L))), name=f"c5_{i}_{j}")
    

    # Constraint (10): First-level vehicle time constraints       
    for(i,j) in A:
        if i in N and j in N:
            model.addConstr((w[j, 0] >= w[i, len(Ni[i]) + 1] + tij[(i, j)] - Mij[(i, j)] * (1 - gp.quicksum(x[(i, j), l] for l in L))), name=f"c10_{i}_{j}")

    # Constraint (13): Binary constraints for x
    # These constraints are already implicit in the definition of the x variables as binary

    # Constraint (14): Capacity constraints
    # Replaced by (34)
    # for i in N0:
    #     model.addConstr((data['qi (Demand of cluster i)'][i] <= u[i]), name=f"c14_lower_{i}")
    #     model.addConstr((u[i] <= Q), name=f"c14_upper_{i}")
    
    # Constraint (20): Define a lower bound on the number of vehicles needed to serve all the clusters based on the total cluster demands and vehicle capacity
    total_demand = gp.quicksum(qi[i] for i in N)
    lower_bound_vehicles = math.ceil(total_demand.getValue() / Q)
    model.addConstr((gp.quicksum(x[(0, j), l] for j in N for l in L) >= lower_bound_vehicles), name="c20")

    # Constraint (21): Eliminate subtours for sets of two and three clusters in first-level routes
    for subset_size in [2, 3]:
        for subset in itertools.combinations(N, subset_size):
            subset_arcs = [(i, j) for i in subset for j in subset if i != j and (i, j) in A]
            if subset_arcs:
                model.addConstr((gp.quicksum(x[arc, l] for arc in subset_arcs for l in L) <= subset_size - 1), name=f"c21_{subset}")

    # Constraint (22): Eliminate first-level arcs that are infeasible due to vehicle capacity or time windows incompatibility
    for (i, j) in A:
        for l in L:
            if i in Ni and j in Ni:
                if qi[i] + qi[j] > Q or ah[i][len(Ni[i])+1] + tij[i, j] > bh[j][0]:
                    model.addConstr((x[(i, j), l] == 0), name=f"c22_{i}_{j}_{l}")

    # Constraint (23): Estimate minimum time spent in each cluster
    for i in N:
        model.addConstr((w[i, len(Ni[i])+1] >= w[i, 0] + gp.quicksum(eil[i][l] * x[(i, j), l] for j in N0 if (i, j) in A for l in L)), name=f"c23_{i}")

    # Constraint (24): Forbid the visit of the cluster by a vehicle with fewer deliverymen than needed to serve it
    for (i, j) in A:
        for l in L:
            if i in mi and j in mi:
                if l < mi[i] or l < mi[j]:
                    model.addConstr((x[(i, j), l] == 0), name=f"c24_{i}_{j}_{l}")

    # Constraint (29): Time window constraints at second-level nodes
    for i in N:
        # Start node (0_i)
        model.addConstr(ah[i][0] <= w[i, 0], name=f"c29_{i}_0_lower")
        model.addConstr(w[i, 0] <= bh[i][0], name=f"c29_{i}_0_upper")
    
        # End node (n_i + 1)
        max_ord_cust_no = len(Ni[i]) + 1
        model.addConstr(ah[i][max_ord_cust_no] <= w[i, max_ord_cust_no], name=f"c29_{i}_{max_ord_cust_no}_lower")
        model.addConstr(w[i, max_ord_cust_no] <= bh[i][max_ord_cust_no], name=f"c29_{i}_{max_ord_cust_no}_upper")
    
    # Constraint (30): Lower bound on eta
    for i in N:
        model.addConstr((eta[i] >= eta_[i]), name=f"c30_{i}")

    return model, x, eta, w

