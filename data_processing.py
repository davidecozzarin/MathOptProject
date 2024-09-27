import os
from SPcost import solve_sp_cost
from SPtime import solve_sp_time

def read_instance(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()

    # Remove any empty lines
    lines = [line.strip() for line in lines if line.strip()]

    instance_name = lines[0].strip()
    num_clusters, num_customers = map(int, lines[1].split())
    # The depot is considered a customer, so we don't need to subtract 1 from the total customers
    vehicle_number = int(lines[4].split()[0])  
    vehicle_capacity = int(lines[4].split()[1])  

    clusters = []   
    customers = []

    cluster_start_index = 6
    cluster_end_index = cluster_start_index + num_clusters + 1

    # Find the index where customer data starts
    customer_start_index = cluster_end_index
    while not lines[customer_start_index].startswith('CUST'):
        customer_start_index += 1
    customer_start_index += 1  

    for line in lines[cluster_start_index:cluster_end_index]:
        parts = line.split()
        cluster = {
            'id': int(parts[0]),
            'x': float(parts[1]),
            'y': float(parts[2]),
            'demand': int(parts[3]),
            'ready_time': int(parts[4]),
            'due_date': int(parts[5]),
            'service_time': int(parts[6])
        }
        clusters.append(cluster)
    
    # Add the final cluster which is identical to the first (depot)
    clusters.append({
            'id': len(clusters),
            'x': clusters[0]['x'], 
            'y': clusters[0]['y'],
            'demand': clusters[0]['demand'],
            'ready_time': clusters[0]['ready_time'],
            'due_date': clusters[0]['due_date'],
            'service_time': clusters[0]['service_time']
    })

    # Counters for ord_cust_no
    cluster_counters = {cluster['id']: 1 for cluster in clusters}

    for line in lines[customer_start_index:]:
        parts = line.split()
        cluster_id = int(parts[7])
        customer = {
            'id': int(parts[0]),
            'x': float(parts[1]),
            'y': float(parts[2]),
            'demand': int(parts[3]),
            'ready_time': int(parts[4]),
            'due_date': int(parts[5]),
            'service_time': int(parts[6]),
            'cluster': cluster_id,
            'ord_cust_no': cluster_counters[cluster_id]
        }
        customers.append(customer)
        cluster_counters[cluster_id] += 1

    # Add start and end nodes for each cluster
    for cluster_id in cluster_counters.keys():
        if cluster_id != 0 and cluster_id != len(clusters) - 1:
            max_ord_cust_no = cluster_counters[cluster_id] - 1
            start_node = {
                'id': max([cust['id'] for cust in customers]) + 1,
                'x': clusters[cluster_id]['x'],
                'y': clusters[cluster_id]['y'],
                'demand': 0,
                'ready_time': clusters[cluster_id]['ready_time'],
                'due_date': clusters[cluster_id]['due_date'],
                'service_time': 0,
                'cluster': cluster_id,
                'ord_cust_no': 0
            }
            end_node = {
                'id': max([cust['id'] for cust in customers]) + 1,
                'x': clusters[cluster_id]['x'],
                'y': clusters[cluster_id]['y'],
                'demand': 0,
                'ready_time': clusters[cluster_id]['ready_time'],
                'due_date': clusters[cluster_id]['due_date'],
                'service_time': 0,
                'cluster': cluster_id,
                'ord_cust_no': max_ord_cust_no + 1
            }
            customers.append(start_node)
            customers.append(end_node)

    return instance_name, vehicle_number, vehicle_capacity, clusters, customers, num_clusters, num_customers

def euclidean_distance(x1, y1, x2, y2):
    return int(((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5)

def read_and_process_instances(instances_dir):
    instance_files = [f for f in os.listdir(instances_dir) if f.endswith('.txt')]

    instances = []

    for instance_file in instance_files:
        file_path = os.path.join(instances_dir, instance_file)
        instance_name, vehicle_number, vehicle_capacity, clusters, customers, num_clusters, num_customers, = read_instance(file_path)

        # Define sets N, N0, and A
        N = list(range(1, len(clusters) - 1))
        N0 = [cluster['id'] for cluster in clusters]
        A = [(i, j) for i in range(len(clusters) - 1 ) for j in range(len(clusters)) if i != j and i != len(clusters) and j != 0]

        # Calculate distances between clusters
        dij = {}
        tij = {}
        for i, cluster_i in enumerate(clusters):
            for j, cluster_j in enumerate(clusters):
                if i != j and i != len(clusters) - 1 and j != 0:
                    distance = euclidean_distance(cluster_i['x'], cluster_i['y'], cluster_j['x'], cluster_j['y'])
                    dij[(i, j)] = distance
                    tij[(i, j)] = distance

        # Calculate distances and travel times between customers within clusters
        dihk = {}
        tihk = {}
        Ai = {}
        Ni = {}
        N0i = {}
        qi = {}
        sh = {}
        ah = {}
        bh = {}
        eta_ = {}


        for cluster in clusters:
            i = cluster['id']
            if i != 0 and i != len(clusters) - 1:
                Ai[i] = []
                dihk[i] = {}
                tihk[i] = {}
                qi[i] = cluster['demand']
                sh[i] = {}
                ah[i] = {}
                bh[i] = {}
                
                cluster_customers = [cust for cust in customers if cust['cluster'] == i]
                
                # Define Ni and N0i
                if i != 0 and i != len(clusters) - 1:
                    max_ord_cust_no = max([cust['ord_cust_no'] for cust in cluster_customers])
                    Ni[i] = [cust['ord_cust_no'] for cust in cluster_customers if cust['ord_cust_no'] != 0 and cust['ord_cust_no'] != max_ord_cust_no]
                    N0i[i] = [0] + Ni[i] + [max_ord_cust_no]

                total_service_time = 0  # For calculating eil
                total_demand = 0  # For calculating mi

                for cust_i in cluster_customers:
                    h = cust_i['ord_cust_no']
                    sh[i][h] = cust_i['service_time']  
                    ah[i][h] = cust_i['ready_time'] 
                    bh[i][h] = cust_i['due_date'] 
                    total_service_time += cust_i['service_time'] 
                    total_demand += cust_i['demand'] 
                    for cust_j in cluster_customers:
                        if cust_i['ord_cust_no'] != cust_j['ord_cust_no']:
                            k = cust_j['ord_cust_no']
                            distance = euclidean_distance(cust_i['x'], cust_i['y'], cust_j['x'], cust_j['y'])
                            if h != max_ord_cust_no and k != 0: 
                                Ai[i].append((h, k))
                                dihk[i][(h, k)] = distance
                                tihk[i][(h, k)] = distance * 3  
                            
                        if cust_i['id'] != cust_j['id']:
                            k = cust_j['ord_cust_no']

        for i in range(len(clusters)):            
            if i in Ni:
                max_bh = max(bh[i].values())
                max_bh_customer = max(bh[i], key=bh[i].get)
                travel_time_to_parking = tihk[i][(max_bh_customer, len(Ni[i]) + 1)]
                max_ord_cust_no = len(Ni[i]) + 1
                bh[i][max_ord_cust_no] = max_bh + travel_time_to_parking

        Mihk = {}
        for i in N:
            Mihk[i] = {}
            for (h, k) in Ai[i]:
                Mihk[i][(h, k)] = max(0, bh[i][h] + sh[i][h] + tihk[i][(h, k)] - ah[i][k])

        Mij = {}
        for (i, j) in A:
            try:
                if i == 0:
                    Mij[(i, j)] = max(0, clusters[0]['due_date'] + tij[(i, j)] - ah[j][0])
                else:
                    max_ord_cust_no = max([cust['ord_cust_no'] for cust in customers if cust['cluster'] == i])
                    Mij[(i, j)] = max(0, bh[i][max_ord_cust_no] + tij[(i, j)] - ah[j][0])
            except KeyError as e:
                print(f"Warning: Missing key {e} in calculation of Mij for arc ({i}, {j}) in instance {instance_name}")

        # Add qi for the initial and final nodes
        qi[0] = 0  
        qi[len(clusters) - 1] = 0  

        instance_data = {
            'vehicle_number': vehicle_number,
            'vehicle_capacity': vehicle_capacity,
            'clusters (parking locations)': num_clusters,
            'customers (total clients)': num_customers,
            'dij (Distance between first-level nodes i and j)': dij,
            'tij (Travel time between first-level nodes i and j)': tij,
            'dihk (Distance between second-level nodes h and k of cluster i)': dihk,
            'tihk (Travel time between second-level nodes h and k of cluster i)': tihk,
            'Ai (set of arcs related to the second-level routes inside cluster i)': Ai,
            'N0i (set of nodes including depot start and end)': N0i,
            'qi (Demand of cluster i)': qi,
            'sh (Service time of customer h in cluster i)': sh,
            'ah (Start of time window of customer h in cluster i)': ah,
            'bh (End of time window of customer h in cluster i)': bh,
            'N0 (Set of nodes including depot start and end)': N0,
            'A (Set of arcs for first-level routes)': A,
            'N (set of cluster indices)': N,
            'Ni (set of customer nodes in cluster i)': Ni,
            'Mihk': Mihk,
            'Mij': Mij,
            'eta_': eta_,
        }

        # Calculate lower bounds 
        instance_data['eil'] = {}
        instance_data['mi'] = {}

        ML = 3
        L = range(1, (ML + 1))
        for i in N:
            instance_data['mi'][i] = 1
            for l in L:
                _, feasible = solve_sp_time(i, l, instance_data)
                if not feasible:
                    instance_data['mi'][i] = l + 1
                    break
            
            instance_data['eil'][i] = {}
            for l in L:
                sum_service_times = sum(sh[i][h] for h in Ni[i])
                max_time = max(tihk[i][(0, h)] + sh[i][h] + tihk[i][(h, len(Ni[i]) + 1)] for h in Ni[i])
                instance_data['eil'][i][l] = max(sum_service_times / l, max_time)
                                 
            # Calculate eta (lower bound cost of deliveryman routes)
            instance_data['eta_'][i] = solve_sp_cost(i, instance_data)

        instances.append((instance_name, instance_data))

    return instances

