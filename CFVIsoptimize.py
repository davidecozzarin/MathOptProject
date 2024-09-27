import gurobipy as gp
from gurobipy import GRB
import itertools
import math

# =====================================================
# Title: Enhanced Compact Formulation with Valid Constraints for VRPTWMD2R
# Description: This script implements the enhanced compact formulation (CF+VCs)
#              for the Vehicle Routing Problem with Time Windows and Multiple 
#              Deliverymen and Second-Level Routes (VRPTWMD2R). The formulation
#              includes additional valid constraints to improve the model's 
#              performance by tightening the linear relaxation, thus enabling 
#              more efficient solving of complex routing problems.
# =====================================================

# Define global parameters
ML = 3  # Maximum number of deliverymen per vehicle  
fv = 1000  # Fixed cost of using a vehicle
fd = 100   # Additional cost per deliveryman
cv = 10    # Cost coefficient for vehicle routing distance  
cd = 1     # Cost coefficient for deliveryman routing distance  

def run_CFVIsoptimize(instance_name, data):
    N = data['N (set of cluster indices)']
    N0 = data['N0 (Set of nodes including depot start and end)'] 
    customers = data['customers (total clients)']
    Ni = data['Ni (set of customer nodes in cluster i)']
    N0i = data['N0i (set of nodes including depot start and end)']
    Q = data['vehicle_capacity']
    qi = data['qi (Demand of cluster i)']
    L = range(1, ML + 1)
    A = data['A (Set of arcs for first-level routes)']
    Ai = data['Ai (set of arcs related to the second-level routes inside cluster i)']
    dij = data['dij (Distance between first-level nodes i and j)']    
    dihk = data['dihk (Distance between second-level nodes h and k of cluster i)']
    tij = data['tij (Travel time between first-level nodes i and j)']
    tihk = data['tihk (Travel time between second-level nodes h and k of cluster i)']
    ah = data['ah (Start of time window of customer h in cluster i)']
    bh = data['bh (End of time window of customer h in cluster i)']
    sh = data['sh (Service time of customer h in cluster i)']
    Mihk = data['Mihk']
    Mij = data['Mij']
    eil = data['eil']
    mi = data['mi']

    # Create the model
    m = gp.Model(instance_name)

    # Decision variables
    x = m.addVars([((i, j), l) for (i, j) in A for l in L], vtype=GRB.BINARY, name="x")   # xijl  whether a vehicle travels from node i to j with l delman (i,j) in A, l in L 
    y = m.addVars([(i, (h, k)) for i in N for (h, k) in Ai[i]], vtype=GRB.BINARY, name="y") # whetheer a delman travels from h to k in Ai within cluster i
    u = m.addVars(N0, vtype=GRB.CONTINUOUS, name="u")  # ui vehicle load after leaving node i in N0
    w = m.addVars([(i, h) for i in N for h in N0i[i]], vtype=GRB.CONTINUOUS, name="w")  # wh time when service at node h in cluster i begins

    # Objective function
    m.setObjective(
        gp.quicksum((fv + l * fd) * x[(0, j), l] for j in N for l in L) +
        cv * gp.quicksum(dij[(i, j)] * x[(i, j), l] for (i, j) in A for l in L) +
        cd * gp.quicksum(dihk[i][(h, k)] * y[i, (h, k)] for i in N for (h, k) in Ai[i]),
        GRB.MINIMIZE
    )

    # Add constraints
    # Constraint (2): Ensure exactly one delivery man visits each cluster
    for j in N:
        m.addConstr((gp.quicksum(x[(i, j), l] for i in N0 if (i, j) in A for l in L) == 1), name=f"c2_{j}")

    # Constraint (3): Flow balance constraints
    for j in N:
        for l in L:
            m.addConstr((gp.quicksum(x[(i, j), l] for i in N0 if (i, j) in A) == gp.quicksum(x[(j, i), l] for i in N0 if (j, i) in A)), name=f"c3_{j}_{l}")

    # Constraint (4): Depot flow constraints
    for l in L:
        m.addConstr((gp.quicksum(x[(0, i), l] for i in N) == gp.quicksum(x[(i, len(N) + 1), l] for i in N)), name=f"c4_{l}")

    # Constraint (5): Time constraints at first-level nodes
    for (i, j) in A:
            m.addConstr((u[j] >= u[i] + qi[j] - Q * (1 - gp.quicksum(x[(i, j), l] for l in L))), name=f"c5_{i}_{j}")

    # Constraint (6): Ensure exactly one visit at the second-level nodes
    for i in N:
        for k in Ni[i]:
            m.addConstr((gp.quicksum(y[i, (h, k)] for h in N0i[i] if (h, k) in Ai[i]) == 1), name=f"c6_{i}_{k}")
    
    # Constraint (7): Flow balance at second-level nodes
    for i in N:
        for k in Ni[i]:
            m.addConstr((gp.quicksum(y[i, (h, k)] for h in N0i[i] if (h, k) in Ai[i]) == gp.quicksum(y[i, (k, h)] for h in N0i[i] if (k, h) in Ai[i])), name=f"c7_{i}_{k}")
    
    # Constraint (8): Depot flow constraints at second level
    for i in N:
        m.addConstr((gp.quicksum(y[i, (0, h)] for h in Ni[i]) == gp.quicksum(y[i, (h, len(Ni[i]) + 1)] for h in Ni[i])), name=f"c8_{i}")
    
    # Constraint (9): Second-level time constraints
    for i in N:
        for (h, k) in Ai[i]:
            m.addConstr((w[i, k] >= w[i, h] + sh[i][h] + tihk[i][(h, k)] - Mihk[i][(h, k)] * (1 - y[i, (h, k)])), name=f"c9_{i}_{h}_{k}")

    # Constraint (10): First-level vehicle time constraints       
    for(i,j) in A:
        if i in N and j in N:
            m.addConstr((w[j, 0] >= w[i, len(Ni[i]) + 1] + tij[i, j] - Mij[(i, j)] * (1 - gp.quicksum(x[(i, j), l] for l in L))), name=f"c10_{i}_{j}")
    
    # Constraint (11): Maximum one delivery man at each cluster
    for j in N:
        m.addConstr((gp.quicksum(y[j, (0, h)] for h in Ni[j]) <= gp.quicksum(l * x[(i, j), l] for i in N0 if (i, j) in A for l in L)), name=f"c11_{j}")

    # Constraint (12): Initial conditions
    m.addConstr(u[0] == 0, "c12_u0")

    # Constraint (13): Binary constraints for x
    # These constraints are already implicit in the definition of the x variables as binary

    # Constraint (14): Capacity constraints
    for i in N0:
        m.addConstr((qi[i] <= u[i]), name=f"c14_lower_{i}")
        m.addConstr((u[i] <= Q), name=f"c14_upper_{i}")

    # Constraint (15): Binary constraints for y
    # These constraints are already implicit in the definition of the x variables as binary

    # Constraint (16): Time window constraints
    for i in N:
        for h in N0i[i]:
            m.addConstr((ah[i][h] <= w[i, h]), name=f"c16_lower_{i}_{h}")
            m.addConstr((w[i, h] <= bh[i][h]), name=f"c16_upper_{i}_{h}")

    # Constraint (17): Ensure at least one deliveryman leaves each parking location.
    for i in N:
        m.addConstr((gp.quicksum(y[i, (0, h)] for h in Ni[i]) >= 1), name=f"c17_{i}")

    # Constraint (18): Eliminate small subtours of two and three customers in second-level routes.
    for i in N:
        for subset_size in [2, 3]:
            for subset in itertools.combinations(Ni[i], subset_size):
                subset_arcs = [(h, k) for h in subset for k in subset if h != k and (h, k) in Ai[i]]
                if subset_arcs:
                    m.addConstr((gp.quicksum(y[i, arc] for arc in subset_arcs) <= subset_size - 1), name=f"c18_{i}_{subset}")

    # Constraint (19): Remove infeasible second-level arcs due to time window incompatibility.
    for i in N:
        for (h, k) in Ai[i]:
            if ah[i][h] + sh[i][h] + tihk[i][(h, k)] > bh[i][k]:
                m.addConstr((y[i, (h, k)] == 0), name=f"c19_{i}_{h}_{k}")
    
    # Constraint (20): Define a lower bound on the number of vehicles needed to serve all the clusters based on the total cluster demands and vehicle capacity.
    total_demand = gp.quicksum(qi[i] for i in N)
    lower_bound_vehicles = math.ceil(total_demand.getValue() / Q)
    m.addConstr((gp.quicksum(x[(0, j), l] for j in N for l in L) >= lower_bound_vehicles), name="c20")

    # Constraint (21): Eliminate subtours for sets of two and three clusters in first-level routes.
    for subset_size in [2, 3]:
        for subset in itertools.combinations(N, subset_size):
            subset_arcs = [(i, j) for i in subset for j in subset if i != j and (i, j) in A]
            if subset_arcs:
                m.addConstr((gp.quicksum(x[arc, l] for arc in subset_arcs for l in L) <= subset_size - 1), name=f"c21_{subset}")

    # Constraint (22): Eliminate first-level arcs that are infeasible due to vehicle capacity or time windows incompatibility.
    for (i, j) in A:
        for l in L:
            if i in Ni and j in Ni:
                if qi[i] + qi[j] > Q or ah[i][len(Ni[i])+1] + tij[i, j] > bh[j][0]:
                    m.addConstr((x[(i, j), l] == 0), name=f"c22_{i}_{j}_{l}")

    # Constraint (23): Provide an estimation on the minimum time spent on the cluster.
    for i in N:
        m.addConstr((w[i, len(Ni[i])+1] >= w[i, 0] + gp.quicksum(eil[i][l] * x[(i, j), l] for j in N0 if (i, j) in A for l in L)), name=f"c23_{i}")

    # Constraint (24): Forbid the visit of the cluster by a vehicle with fewer deliverymen than needed to serve it.
    for (i, j) in A:
        for l in L:
            if i in mi and j in mi:
                if l < mi[i] or l < mi[j]:
                    m.addConstr((x[(i, j), l] == 0), name=f"c24_{i}_{j}_{l}")

    # Constraint (25): Ensure that the number of deliverymen leaving a parking location respects its lower bound.
    for i in N:
        m.addConstr((gp.quicksum(y[i, (0, h)] for h in Ni[i]) >= mi[i]), name=f"c25_{i}")

    # Optimize model
    m.setParam('TimeLimit', 7200)  # Set time limit for the optimization
    m.optimize()

    # Check the result and output
    if m.status == GRB.INFEASIBLE:
        print(f"Model {instance_name} is infeasible")
        m.computeIIS()
        m.write(f"{instance_name}_iis.ilp")
        status = "Infeasible"

        print(f"Infeasible constraints for {instance_name}:")
        for c in m.getConstrs():
            if c.IISConstr:
                print(f"Constraint {c.constrName} is in the IIS.")
    else:
        if m.status == GRB.Status.OPTIMAL:
            print("Optimal solution found")
            status = "Optimal"
        elif m.status == GRB.Status.TIME_LIMIT:
            print("Time limit reached, best solution found:")
            status = "Feasible"
        else:
            print("Optimization was stopped with status ", m.status)

        # Output the results
        vehicles_used = sum(sum(x[(0, j), l].x for j in N) for l in L)
        delivery_men_used = sum(sum(x[(0, j), l].x * l for j in N) for l in L)
        first_level_distance = sum(sum(dij[(i, j)] * x[(i, j), l].x for (i, j) in A) for l in L)
        second_level_distance = sum(sum(dihk[i][(h, k)] * y[i, (h, k)].x for (h, k) in Ai[i]) for i in N)

        print(f"==> Vehicles used: {vehicles_used}")
        print(f"==> Delivery men used: {delivery_men_used}")
        print(f"==> First level distance: {first_level_distance}")
        print(f"==> Second level distance: {second_level_distance}")

        results = {
            "instance_name": instance_name,
            "vehicle_number": data['vehicle_number'],
            "vehicle_capacity": data['vehicle_capacity'],
            "clusters": data['clusters (parking locations)'],
            "clients": data['customers (total clients)'],
            "objective_value": m.ObjVal,
            "x": m.getAttr('x', x),
            "y": m.getAttr('x', y),
            "u": m.getAttr('x', u),
            "w": m.getAttr('x', w)
        }

        print(f"Results for instance {instance_name}:")
        print(f"Objective value: {results['objective_value']}")
        print(f"Vehicle number: {results['vehicle_number']}")
        print(f"Vehicle capacity: {results['vehicle_capacity']}")
        print(f"Clusters: {results['clusters']}")
        print(f"Clients: {results['clients']}")
        print("Variables x:")
        for key, value in results['x'].items():
            if value > 0.5:
                print(f"  {key}: {value}")
        print("Variables y:")
        for key, value in results['y'].items():
            if value > 0.5:
                print(f"  {key}: {value}")
        print("Variables u:")
        for key, value in results['u'].items():
            print(f"  {key}: {value}")
        print("Variables w:")
        for key, value in results['w'].items():
            print(f"  {key}: {value}")
        print("-" * 50)

        return {
        "instance_name": instance_name,
        "algorithm": "CF+VI's",
        "status": status,
        "vehicles_used": vehicles_used,
        "delivery_men_used": delivery_men_used,
        "first_level_distance": first_level_distance,
        "second_level_distance": second_level_distance,
        "objective_value": m.ObjVal,
        "best_bound": m.ObjBound,
        "gap":  m.MIPGap * 100
    }