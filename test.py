from BBCoptimize import run_BBCoptimize
from CFoptimize import run_CFoptimize
from CFVIsoptimize import run_CFVIsoptimize
from data_processing import read_and_process_instances 

instances_dir = "./test_istances"

instances = read_and_process_instances(instances_dir)

results = []

def run_all_algorithms_on_instance(instance_name, instance_data):
    instance_results = []
    
    print(f"\nRunning CFoptimize on {instance_name}...")
    cf_results = run_CFoptimize(instance_name, instance_data)
    instance_results.append(cf_results)
    
    print(f"Running CFVIsoptimize on {instance_name}...")
    cf_vc_results = run_CFVIsoptimize(instance_name, instance_data)
    instance_results.append(cf_vc_results)
    
    print(f"Running BBC Algorithm on {instance_name}...")
    bbc_results = run_BBCoptimize(instance_name, instance_data)
    instance_results.append(bbc_results)
    
    return instance_results

for instance_name, instance_data in instances:
    results.extend(run_all_algorithms_on_instance(instance_name, instance_data))

print("\n=== FINAL RESULTS ===")
for result in results:
    print(f"Instance: {result['instance_name']}")
    print(f"Algorithm: {result['algorithm']}")
    print(f"Status: {result['status']}")
    print(f"Vehicles used: {result['vehicles_used']}")
    print(f"Delivery men used: {result['delivery_men_used']}")
    print(f"First level distance: {result['first_level_distance']}")
    print(f"Second level distance: {result['second_level_distance']}")
    print("-" * 50)
