from data_processing import read_and_process_instances

# A test class that I need to validate the result of the preprocessing performed by data_processing.py

# Directory containing instances
instances_dir = "./test_istances"

# Function to test the read_and_process_instances function
def test_read_and_process_instances(instances_dir):
    instances = read_and_process_instances(instances_dir)

    for instance_name, data in instances:
        print(f"Instance: {instance_name}")
        print(f"Vehicle Number: {data['vehicle_number']}")
        print(f"Vehicle Capacity: {data['vehicle_capacity']}")
        print(f"Number of Clusters: {data['clusters (parking locations)']}")
        print(f"Number of Customers: {(data['customers (total clients)'])}")
        print(f"N: {data['N (set of cluster indices)']}")
        print(f"N0: {data['N0 (Set of nodes including depot start and end)']}")
        print(f"A: {data['A (Set of arcs for first-level routes)']}")
        print(f"First-level distances (dij): {data['dij (Distance between first-level nodes i and j)']}") 
        print(f"First-level travel times (tij): {data['tij (Travel time between first-level nodes i and j)']}")        
        print (f"Ni: {data['Ni (set of customer nodes in cluster i)']}")
        print(f"N0i:{data['N0i (set of nodes including depot start and end)']}")
        print(f"Ai:{data['Ai (set of arcs related to the second-level routes inside cluster i)']}")
        print(f"Second-level distances (dihk): {data['dihk (Distance between second-level nodes h and k of cluster i)']}")  
        print(f"Second-level travel times (tihk): {list(data['tihk (Travel time between second-level nodes h and k of cluster i)'].items())[:2]}")  
        print(f"ah:{data['ah (Start of time window of customer h in cluster i)']}")
        print(f"bh:{data['bh (End of time window of customer h in cluster i)']}")
        print(f"sh:{data['sh (Service time of customer h in cluster i)']}")
        print(f"Mij:{data['Mij']}")
        print(f"Mihk:{data['Mihk']}")
        print(f"qi:{data['qi (Demand of cluster i)']}")
        print(f"eil:{data['eil']}")
        print(f"mi:{data['mi']}")
        print(f"eta_:{data['eta_']}")
        print("=" * 50)

# Run the test
test_read_and_process_instances(instances_dir)
