import gurobipy as gp
from gurobipy import GRB
import itertools

# =====================================================
# Title: Subproblem Solver for VRP with Multiple Deliverymen
# Description: This script defines and solves the Subproblem (SP) for a Vehicle Routing Problem (VRP) 
#              with multiple deliverymen. The SP optimizes the deliveryman routes within each cluster, 
#              minimizing the total second-level distances and determining feasibility based on time 
#              and capacity constraints.
# =====================================================

# Define global parameters
ML = 3  # Maximum number of deliverymen per vehicle  
cd = 1  # Cost coefficient for deliveryman routing distance  

def solve_subproblem(data, r, l, Nr, Ar,):
    Ai = data['Ai (set of arcs related to the second-level routes inside cluster i)']   
    tij = data['tij (Travel time between first-level nodes i and j)']
    dihk = data['dihk (Distance between second-level nodes h and k of cluster i)']
    sh = data['sh (Service time of customer h in cluster i)']
    tihk = data['tihk (Travel time between second-level nodes h and k of cluster i)']
    Mihk = data['Mihk']
    Ni = data['Ni (set of customer nodes in cluster i)']
    N0i = data['N0i (set of nodes including depot start and end)']
    ah = data['ah (Start of time window of customer h in cluster i)']
    bh = data['bh (End of time window of customer h in cluster i)']
    N = data['N (set of cluster indices)']
    eil = data['eil']

    # Create the model for the subproblem
    sp_model = gp.Model("SP")

    # Filter Nr to remove depot nodes from the primary route
    NrFiltered = [i for i in Nr if i in N]

    # Decision variables for the subproblem
    x = sp_model.addVars([(i, (h, k)) for i in NrFiltered for (h, k) in Ai[i]], vtype=GRB.BINARY, name="x")
    w = sp_model.addVars([(i, h) for i in NrFiltered for h in N0i[i]], vtype=GRB.CONTINUOUS, name="w")

    # Objective function
    sp_model.setObjective(cd * gp.quicksum(dihk[i][(h, k)] * x[i, (h, k)] for i in NrFiltered for (h, k) in Ai[i]), GRB.MINIMIZE)

    # Constraints for the subproblem
    for i in NrFiltered:
        for k in Ni[i]:
            sp_model.addConstr(gp.quicksum(x[i, (h, k)] for h in N0i[i] if (h, k) in Ai[i]) == 1, name=f"c36_{i}_{k}")  # Constraint (36)
            sp_model.addConstr(gp.quicksum(x[i, (h, k)] for h in N0i[i] if (h, k) in Ai[i]) == gp.quicksum(x[i, (k, h)] for h in N0i[i] if (k, h) in Ai[i]), name=f"c37_{i}_{k}")  # Constraint (37)

        sp_model.addConstr(gp.quicksum(x[i, (0, h)] for h in Ni[i]) == gp.quicksum(x[i, (h, len(Ni[i]) + 1)] for h in Ni[i]), name=f"c38_{i}")  # Constraint (38)

        for (h, k) in Ai[i]:
            sp_model.addConstr(w[i, k] >= w[i, h] + sh[i][h] + tihk[i][(h, k)] - Mihk[i][(h, k)] * (1 - x[i, (h, k)]), name=f"c39_{i}_{h}_{k}")  # Constraint (39)

        sp_model.addConstr(gp.quicksum(x[i, (0, h)] for h in Ni[i]) <= l, name=f"c40_{i}")  # Constraint (40)
    
    # Constraint (41)
    for (i, j) in Ar: 
        if i != 0 and j != (len(N) + 1):
            sp_model.addConstr(w[j, 0] >= w[i, len(Ni[i]) + 1] + tij[i, j], name=f"c41_{i}_{j}")

    # Constraint (42): Binary constraints for x
    # These constraints are already implicit in the definition of the x variables as binary

    # Constraint (43)
    for i in NrFiltered:
        sp_model.addConstrs((ah[i][h] <= w[i, h] for h in N0i[i]), name=f"c43a_{i}")
        sp_model.addConstrs((w[i, h] <= bh[i][h] for h in N0i[i]), name=f"c43b_{i}")

    # Constraint (17): Ensure at least one deliveryman leaves each parking location.
    for i in NrFiltered:
        sp_model.addConstr((gp.quicksum(x[i, (0, h)] for h in Ni[i]) >= 1), name=f"c17_{i}")

    # Constraint (18): Eliminate small subtours of two and three customers in second-level routes.
    for i in NrFiltered:
        for subset_size in [2, 3]:
            for subset in itertools.combinations(Ni[i], subset_size):
                subset_arcs = [(h, k) for h in subset for k in subset if h != k and (h, k) in Ai[i]]
                if subset_arcs:
                    sp_model.addConstr((gp.quicksum(x[i, arc] for arc in subset_arcs) <= subset_size - 1), name=f"c18_{i}_{subset}")

    # Constraint (19): Remove infeasible second-level arcs due to time window incompatibility.
    for i in NrFiltered:
        for (h, k) in Ai[i]:
            if ah[i][h] + sh[i][h] + tihk[i][(h, k)] > bh[i][k]:
                sp_model.addConstr((x[i, (h, k)] == 0), name=f"c19_{i}_{h}_{k}")

    # Constraint (44)
    for i in NrFiltered:
        sp_model.addConstr((w[i, len(Ni[i]) + 1] >= w[i, 0] + eil[i][l]), name=f"c44{i}")

    # Optimize subproblem
    sp_model.optimize()

    if sp_model.status == GRB.Status.OPTIMAL:
        optimality_cut = (r, l, sp_model.ObjVal)
        feasibility_cut = None
        crl = sp_model.ObjVal
    else:
        optimality_cut = None
        feasibility_cut = (r, l)
        crl = None

    print(f"Subproblem status: {sp_model.status}, optimality cut: {optimality_cut}, feasibility cut: {feasibility_cut}")
    return optimality_cut, feasibility_cut, crl,
