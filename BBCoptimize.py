import gurobipy as gp
from gurobipy import GRB
from MPoptimize import define_rmp
from SPoptimize import solve_subproblem
import itertools
import math

# =====================================================
# Title: Branch-and-Benders-Cut Algorithm for VRP
# Description: This script implements a Branch-and-Benders-Cut (BBC) 
#              algorithm to solve a Vehicle Routing Problem (VRP) 
#              with multiple deliverymen, focusing on optimizing 
#              second-level delivery routes to minimize overall distance.
# =====================================================

# Define global parameters for the problem
ML = 3       # Maximum number of delivery men

# Initialize counters for cuts and callbacks
num_optimality_cuts = 0
num_feasibility_cuts = 0
counter = 0
RCIsCounter = 0

# Custom callback function to be called during the optimization process
def custom_callback(model, where):
    global num_optimality_cuts, num_feasibility_cuts, counter, RCIsCounter

    # Callback triggered when an integer solution is found
    if where == GRB.Callback.MIPSOL:
        counter = counter + 1
        print(f"MIPSOL callback triggered: {counter} time(s)")
        sol = model.cbGetSolution(model.getVars())
        sol_dict = {var.VarName: sol[i] for i, var in enumerate(model.getVars())}

        # Check rounded capacity inequalities (RCIs)
        rci_satisfied, violating_subset = check_rci(model, model._data, sol_dict)

        if not rci_satisfied:
            expr = gp.quicksum(model._x[(i, j), l] for (i, j) in model._A if i not in violating_subset and j in violating_subset for l in model._L)
            model.cbLazy(expr >= math.ceil(sum(model._qi[i] for i in violating_subset) / model._Q))
            print(f"Added RCI cut for subset {violating_subset}")
            RCIsCounter += 1

        # Separate the solution by vehicle routes
        routes = {}
        for (i, j), l in model._x.keys():
            if sol_dict[model._x[(i, j), l].VarName] > 0.5:
                if l not in routes:
                    routes[l] = []
                routes[l].append((i, j))

        print(f"Routes: {routes}")

        # Reconstruct the complete route for each vehicle
        complete_routes = {}
        for l, arcs in routes.items():
            complete_routes[l] = []
            for arc in arcs:
                if arc[0] == 0:  # Start from depot
                    current_route = [arc]
                    next_node = arc[1]
                    while next_node != len(model._N) + 1:  # Until reaching the end depot
                        found_next_arc = False
                        for next_arc in arcs:
                            if next_arc[0] == next_node:
                                current_route.append(next_arc)
                                next_node = next_arc[1]
                                found_next_arc = True
                                break
                                
                        if not found_next_arc:
                            break
                    complete_routes[l].append(current_route)

        print(f"Complete Routes: {complete_routes}")

        # Solve SPs and add cuts
        for l, route_list in complete_routes.items():
            for r, route in enumerate(route_list):
                Nr = {node for arc in route for node in arc}
                Ar = route
                Ar_hat = [(i, j) for (i, j) in Ar if i != 0 and j != len(model._N) + 1]
                print(f"Number Route: {r}, number l: {l}: Nr = {Nr}, Ar = {Ar}, Ar_hat = {Ar_hat}")
                optimality_cut, feasibility_cut, crl = solve_subproblem(model._data, r, l, Nr, Ar,)

                if crl != None:
                    # Store the second-level distance for this route
                    key = (tuple(Ar), l)
                    if key in model._route_distance_dict:
                        if model._route_distance_dict[key]['distance'] > crl:
                            model._route_distance_dict[key] = {'distance': crl, 'route': Ar, 'l': l}
                    else:
                        model._route_distance_dict[key] = {'distance': crl, 'route': Ar, 'l': l}

                if optimality_cut:  # Add optimality cut (31)
                    expr = gp.quicksum(model._eta[i] for i in Nr if i != 0 and i != (len(model._N) + 1)) >= crl * (gp.quicksum(model._x[(i, j), l] for (i, j) in Ar_hat) - len(Ar_hat) + 1)
                    model.cbLazy(expr)
                    print(f"Added optimality cut: {expr}")
                    num_optimality_cuts += 1 

                # Check on len(Ar_hat) - 1
                #lhs_value = len(Ar_hat) - 1
                #if lhs_value < 0:
                #    lhs_value = 0

                #if feasibility_cut: #  Add feasibility cut (33)
                #    expr = gp.quicksum(model._x[(i, j), l_] for (i, j) in Ar_hat for l_ in model._L if l_ <= l) <= lhs_value
                #    model.cbLazy(expr)
                #    print(f"Added feasibility cut: {expr}")
                #    num_feasibility_cuts += 1

                if feasibility_cut: #  Add feasibility cut (28)/(33)
                    lhs_value = len(Ar_hat) - 1
                    if lhs_value < 0:
                        expr = gp.quicksum(model._x[(i, j), l] for (i, j) in Ar) <= (len(Ar) - 1)
                        model.cbLazy(expr)
                        print(f"Added feasibility cut: {expr}")
                        num_feasibility_cuts += 1
                    else:
                        expr = gp.quicksum(model._x[(i, j), l_] for (i, j) in Ar_hat for l_ in model._L if l_ <= l) <= lhs_value
                        model.cbLazy(expr)
                        print(f"Added feasibility cut: {expr}")
                        num_feasibility_cuts += 1

# Function to check rounded capacity inequalities (RCIs)
def check_rci(model, data, sol_dict):
    N = data['N (set of cluster indices)']
    qi = data['qi (Demand of cluster i)']
    Q = data['vehicle_capacity']
    A = data['A (Set of arcs for first-level routes)']
    L = range(1, ML + 1)

    for S in range(1, len(N)):
        for subset in itertools.combinations(N, S):
            lhs = sum(sol_dict[model._x[(i, j), l].VarName] for (i, j) in A if i not in subset and j in subset for l in L)
            rhs = math.ceil(sum(qi[i] for i in subset) / Q)
            #print(f"Debug: Checking subset {subset} with lhs = {lhs} and rhs = {rhs}")
            if lhs < rhs:
                return False, subset
    return True, None

# Main BBC algorithm function
def run_BBCoptimize(instance_name, data):
    model, x, eta, w = define_rmp(data)

    # Initialize the dictionary to store second-level distances for each route
    model._route_distance_dict = {}

    # Set attributes to the model
    model._data = data
    model._x = x
    model._eta = eta
    model._w = w 
    model._vars = model.getVars()
    model._N = data['N (set of cluster indices)']
    model._qi = data['qi (Demand of cluster i)']
    model._Q = data['vehicle_capacity']
    model._A = data['A (Set of arcs for first-level routes)']
    model._L = range(1, ML + 1)
    model._dihk = data['dihk (Distance between second-level nodes h and k of cluster i)']
    model._Ai = data['Ai (set of arcs related to the second-level routes inside cluster i)']
    
    # Solve the Master Problem (MP) with the callback
    model.setParam(GRB.Param.TimeLimit, 7200)
    model.setParam(GRB.Param.LazyConstraints, 1)
    model.optimize(custom_callback)
    
    # Check the result and output
    if model.status == GRB.INFEASIBLE:
        print(f"Model {instance_name} is infeasible")
        model.computeIIS()
        model.write(f"{instance_name}_iis.ilp")
        status = "Infeasible"
    else:
        if model.status == GRB.Status.OPTIMAL:
            print(f"Objective Value: {model.ObjVal}")
            status = "Optimal"
            print("Solution:")
            for var in model.getVars():
                if var.X > 0:
                    print(f"{var.VarName}: {var.X}")            
        elif model.status == GRB.Status.TIME_LIMIT:
            print("Time limit reached, best solution found:")
            status = "Feasible"
        else:
            print("Optimization was stopped with status ", model.status)

    # Extract and print key metrics
    vehicles_used = sum(x[(0, j), l].x for j in model._N for l in model._L)
    delivery_men_used = sum(x[(0, j), l].x * l for j in model._N for l in model._L)
    first_level_distance = sum(data['dij (Distance between first-level nodes i and j)'][(i, j)] * x[(i, j), l].x for (i, j) in model._A for l in model._L)

    print(f"==> Model Objective Value: {model.ObjVal}")
    print(f"==> Vehicles used: {vehicles_used}")
    print(f"==> Delivery men used: {delivery_men_used}")
    print(f"==> First level distance: {first_level_distance}")
    
    total_second_level_distance = 0
    for (i, j), l in model._x.keys():
        if model._x[(0, j), l].x > 0.5:
            current_route = [(i, j)]
            next_node = j
            while next_node != len(model._N) + 1:
                found_next_arc = False
                for (k, m), l_ in model._x.keys():
                    if model._x[(k, m), l_].x > 0.5 and k == next_node:
                        current_route.append((k, m))
                        next_node = m
                        found_next_arc = True
                        break
                if not found_next_arc:
                    break

            key = (tuple(current_route), l)
            if key in model._route_distance_dict:
                distance = model._route_distance_dict[key]['distance']
                if distance is not None:
                    total_second_level_distance += distance

    print(f"==> Total second level distance: {total_second_level_distance}")
    print(f"Number of optimality cuts: {num_optimality_cuts}")
    print(f"Number of feasibility cuts: {num_feasibility_cuts}") 
    print(f"Callback counter: {counter}")
    print(f"RCIs counter: {RCIsCounter}")

    return {
        "instance_name": instance_name,
        "algorithm": "BBC",
        "status": status,
        "vehicles_used": vehicles_used,
        "delivery_men_used": delivery_men_used,
        "first_level_distance": first_level_distance,
        "second_level_distance": total_second_level_distance,
        "objective_value": model.ObjVal,
        "best_bound": model.ObjBound,
        "gap":  model.MIPGap * 100
    }   
        

