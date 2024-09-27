import time
import csv
import os
from BBCoptimize import run_BBCoptimize
from CFoptimize import run_CFoptimize
from CFVIsoptimize import run_CFVIsoptimize
from data_processing import read_and_process_instances

instances_dir = "./scalability_istances"

def split_instance_name(instance_name):
    parts = instance_name.split('_', 1) 
    instance = parts[0]
    size = parts[1] if len(parts) > 1 else ''
    return instance, size

def run_algorithm(algorithm, instance_name, instance_data):
    start_time = time.time()
    result = algorithm(instance_name, instance_data)
    end_time = time.time()
    result['computation_time'] = round(end_time - start_time, 2)

    instance, size = split_instance_name(result['instance_name'])
    result['instance'] = instance
    result['size'] = size
    
    return result

def save_results_to_csv(results_list, filename):
    file_exists = os.path.isfile(filename)
    
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file, delimiter=';')
        
        if not file_exists:
            writer.writerow(["Instance", "Size", "Algorithm", "Status", "Objective Value", "UB",
                             "Gap (%)", "Vehicles used", "Delivery men used",
                             "First level distance", "Second level distance", "Time(s)",])
                   
        for result in results_list:
            writer.writerow([result['instance'],result['size'], result['algorithm'],result['status'], result['objective_value'], 
                             result['best_bound'], result['gap'], result['vehicles_used'], result['delivery_men_used'], 
                             result['first_level_distance'], result['second_level_distance'], result['computation_time'],])

def print_results(results_list):
    for result in results_list:
        print(f"Instance: {result['instance_name']}")
        print(f"Algorithm: {result['algorithm']}")
        
        print(f"Objective Value: {result['objective_value']}")
        print(f"Status: {result['status']}")
        print(f"Upper Bound: {result['best_bound']}")
        print(f"Gap: {result['gap']}%")
        print(f"Vehicles used: {result['vehicles_used']}")
        print(f"Delivery men used: {result['delivery_men_used']}")
        print(f"First level distance: {result['first_level_distance']}")
        print(f"Second level distance: {result['second_level_distance']}")
        print(f"Computation time: {result['computation_time']} seconds")
        print("-" * 50)

def run_all_algorithms_on_instances(instances):
    all_results = []
    
    for instance_name, instance_data in instances:
        print(f"\nRunning all algorithms on instance: {instance_name}\n")
        
        # CF Algorithm
        print(f"Running CF Algorithm on {instance_name}...")
        cf_results = run_algorithm(run_CFoptimize, instance_name, instance_data)
        all_results.append(cf_results)

        # CF + VIs Algorithm
        print(f"Running CF+VIs Algorithm on {instance_name}...")
        cf_vis_results = run_algorithm(run_CFVIsoptimize, instance_name, instance_data) 
        all_results.append(cf_vis_results)

        # BBC Algorithm
        print(f"Running BBC Algorithm on {instance_name}...")
        bbc_results = run_algorithm(run_BBCoptimize, instance_name, instance_data)
        all_results.append(bbc_results)
    
    return all_results

# Load and preprocess instances
instances = read_and_process_instances(instances_dir)

# Execute all 3 models on each instance
all_results = run_all_algorithms_on_instances(instances)

print("\n")
print("=" * 50)
print("SCALABILITY ANALYSIS RESULTS")
print("=" * 50)

print_results(all_results)

save_results_to_csv(all_results, "scalability_results.csv")
print("Results saved to scalability_results.csv")
