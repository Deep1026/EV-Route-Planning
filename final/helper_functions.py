import math
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np

INF = 1e9

class Node:
    def __init__(self, id, type, x, y, demand, ready_time, due_date, service_time):
        self.id = id
        self.type = type # 'd' depot, 'c' customer, 'f' fuel/charging station
        self.x = x
        self.y = y
        self.demand = demand
        self.ready_time = ready_time
        self.due_date = due_date
        self.service_time = service_time

def euclidean_distance(n1, n2):
    return math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

def parse_custom_schneider_dataset(filepath):
    """
    Parses the specific Schneider format where metadata is defined 
    with slashes (e.g., 'Q Vehicle fuel tank capacity /77.75/').
    """
    depot = None
    customers = []
    stations = []
    metadata = {}
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    for line in lines:
        line = line.strip()
        if not line: 
            continue
            
        # 1. Parse Metadata (Look for the '/' delimiters)
        if '/' in line:
            parts = line.split('/')
            if len(parts) >= 2:
                # The text before the first slash usually starts with the parameter key
                description = parts[0].strip()
                try:
                    value = float(parts[1].strip())
                    
                    # Map the starting character to our metadata dictionary
                    if description.startswith('Q'): metadata['fuel_capacity'] = value
                    elif description.startswith('C'): metadata['load_capacity'] = value
                    elif description.startswith('r'): metadata['consumption_rate'] = value
                    elif description.startswith('g'): metadata['inverse_refueling_rate'] = value
                    elif description.startswith('v'): metadata['velocity'] = value
                except ValueError:
                    pass # Ignore if it wasn't a valid number between the slashes
            continue

        # 2. Parse Nodes
        parts = line.split()
        # Ensure it's a data row by checking if it has enough columns 
        # and if the second column is a valid node type ('d', 'f', 'c')
        if len(parts) >= 8 and parts[1] in ['d', 'f', 'c']:
            node = Node(
                id=parts[0],
                type=parts[1],
                x=float(parts[2]),
                y=float(parts[3]),
                demand=float(parts[4]),
                ready_time=float(parts[5]),
                due_date=float(parts[6]),
                service_time=float(parts[7])
            )
            
            if node.type == 'd':
                depot = node
            elif node.type == 'f':
                stations.append(node)
            elif node.type == 'c':
                customers.append(node)

    return depot, customers, stations, metadata

def simple_sorted_insertion(depot, customers, vehicle_speed):
    """
    Builds ONE feasible route sequence by sorting customers by their due dates 
    (Earliest Due Date first) and appending them until capacity or time is exhausted.
    """
    # Sort customers by their ending times (due_date)
    sorted_customers = sorted(customers, key=lambda c: c.due_date)
    
    route = []
    current_load = 0
    current_time = 0
    current_node = depot
    
    for customer in sorted_customers:
        # Time Window Feasibility Check
        # Calculate arrival time from the last visited node
        travel_time = euclidean_distance(current_node, customer) / vehicle_speed
        arrival_time = current_time + travel_time
        
        # If we arrive before the ready_time, we wait
        arrival_time = max(arrival_time, customer.ready_time) 
        
        # Only add to the route if we can service it before the due_date
        if arrival_time <= customer.due_date:
            route.append(customer)
            current_load += customer.demand
            current_time = arrival_time
            current_node = customer
            
    return route

def prepare_pipeline_inputs(depot, route_customers, stations):
    """Maps the generated route back to your expected fixed-path pipeline format."""
    delivery_points = len(route_customers)
    charging_stations = len(stations)
    
    time_windows = [[c.ready_time, c.due_date] for c in route_customers]

    cargo_weight = [c.demand for c in route_customers]
    
    # Distance between points (n + 1 legs)
    distance_between_points = []
    if delivery_points > 0:
        distance_between_points.append(euclidean_distance(depot, route_customers[0]))
        for i in range(len(route_customers) - 1):
            distance_between_points.append(euclidean_distance(route_customers[i], route_customers[i+1]))
        distance_between_points.append(euclidean_distance(route_customers[-1], depot))
    
    distance_to_stations = []
    
    # From depot
    distance_to_stations.append([euclidean_distance(depot, s) for s in stations])
    
    # From each customer node
    for c in route_customers:
        distance_to_stations.append([euclidean_distance(c, s) for s in stations])
        
    return {
        "delivery_points": delivery_points,
        "charging_stations": charging_stations,
        "time_windows": time_windows,
        "distance_between_points": distance_between_points,
        "distance_to_stations": distance_to_stations,
        "cargo_weight": cargo_weight
    }

def plot_charge_vs_time(path_data, save_path = None):
    times = []
    charges = []

    # Parse the sequence into plottable points
    for step in path_data:
        if step['type'] == 'direct':
            times.extend([step['time_at_from'], step['time_at_to']])
            charges.extend([step['charge_at_from'], step['charge_at_to']])
        elif step['type'] == 'via_station':
            times.extend([
                step['time_at_from'],
                step['time_at_station_arrive'],
                step['time_at_station_depart'],
                step['time_at_to']
            ])
            charges.extend([
                step['charge_at_from'],
                step['charge_at_station_arrive'],
                step['charge_at_station_depart'],
                step['charge_at_to']
            ])

    # Create the figure
    plt.figure(figsize=(12, 7))
    plt.plot(times, charges, marker='o', linestyle='-', color='#1f77b4', linewidth=2, markersize=5, label='Route')

    # Add specific styling for Charging segments
    for step in path_data:
        if step['type'] == 'via_station':
            plt.plot(
                [step['time_at_station_arrive'], step['time_at_station_depart']],
                [step['charge_at_station_arrive'], step['charge_at_station_depart']],
                color='green', linewidth=3, 
                label='Charging' if 'Charging' not in plt.gca().get_legend_handles_labels()[1] else ""
            )

    annotated_times = set()
    for step in path_data:
        if step['time_at_from'] not in annotated_times:
            plt.text(step['time_at_from'], step['charge_at_from'] + 0.2, f"Node {step['from_node']}", 
                     fontsize=9, ha='center', va='bottom', rotation=45)
            annotated_times.add(step['time_at_from'])

        if step['type'] == 'via_station':
            if step['time_at_station_arrive'] not in annotated_times:
                plt.text(step['time_at_station_arrive'], step['charge_at_station_arrive'] - 0.2, f"Arr Stn {step['station']}", 
                         fontsize=9, color='green', ha='center', va='top')
                annotated_times.add(step['time_at_station_arrive'])
                
            if step['time_at_station_depart'] not in annotated_times:
                plt.text(step['time_at_station_depart'], step['charge_at_station_depart'] + 0.2, f"Dep Stn {step['station']}", 
                         fontsize=9, color='green', ha='center', va='bottom')
                annotated_times.add(step['time_at_station_depart'])

        if step['time_at_to'] not in annotated_times:
            plt.text(step['time_at_to'], step['charge_at_to'] + 0.2, f"Node {step['to_node']}", 
                     fontsize=9, ha='center', va='bottom', rotation=45)
            annotated_times.add(step['time_at_to'])

    # Titles, limits, and labels
    plt.title('Charge vs. Time during Delivery Sequence', fontsize=14)
    plt.xlabel('Time', fontsize=12)
    plt.ylabel('Charge', fontsize=12)
    plt.ylim(0, max(charges) + 2) 
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()

    # Save the plot
    plt.savefig('charge_vs_time.png')
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
        plt.close()
    else:
        plt.show()

def format_path_pretty(path_list):
    output = []
    output.append("-" * 60)
    for i, step in enumerate(path_list):
        output.append(f"Step {i + 1}: {step['type'].replace('_', ' ').title()}")
        
        if step['type'] == 'direct':
            output.append(f"  Route:  Node {step['from_node']} -> Node {step['to_node']}")
            output.append(f"  Time:   {step['time_at_from']:.2f}  -> {step['time_at_to']:.2f}")
            output.append(f"  Charge: {step['charge_at_from']} -> {step['charge_at_to']}")
            
        elif step['type'] == 'via_station':
            output.append(f"  Route:  Node {step['from_node']} -> Station {step['station']} -> Node {step['to_node']}")
            output.append(f"  Time:   {step['time_at_from']:.2f} (Dep) -> {step['time_at_station_arrive']:.2f} (Arr Sta) -> {step['time_at_station_depart']:.2f} (Dep Sta) -> {step['time_at_to']:.2f} (Arr)")
            output.append(f"  Charge: {step['charge_at_from']} -> {step['charge_at_station_arrive']} (Arr Sta) [+ {step['charge_added']} Added] -> {step['charge_at_station_depart']} (Dep Sta) -> {step['charge_at_to']} (Arr)")
            
        output.append("-" * 60)
    return "\n".join(output)

def save_inputs_to_file(filename, delivery_points, charging_stations, time_windows, distance_between_points, distance_to_stations, cargo_weight,  time_per_charge, fuel_capacity, charge_loss_for_d1):
    with open(filename, 'w') as f:

        f.write("[ PROBLEM DIMENSIONS ]\n")
        f.write(f"{'- Delivery Points':<25}: {delivery_points}\n")
        f.write(f"{'- Charging Stations':<25}: {charging_stations}\n")
        f.write(f"time_per_charge: {time_per_charge}\n")
        f.write(f"fuel_capacity: {fuel_capacity}\n")
        f.write(f"charge_loss_for_d1: {charge_loss_for_d1}\n\n")

        # Cargo Weights
        f.write("[ CARGO WEIGHTS ]\n")
        for i, weight in enumerate(cargo_weight):
            f.write(f"  Point {i+1:<4} : {weight}\n")
        f.write("\n")

        # Time Windows
        f.write("[ TIME WINDOWS ]\n")
        for i, tw in enumerate(time_windows):
            start = f"{tw[0]}"
            end = "INF" if tw[1] == INF else f"{tw[1]}"
            f.write(f"  Point {i:<4} : [{start}, {end}]\n")
        f.write("\n")

        # Distances Between Points
        f.write("[ DISTANCES BETWEEN POINTS (Path A -> B) ]\n")
        for i, dist in enumerate(distance_between_points):
            f.write(f"  Node {i:<2} -> Node {i+1:<2} : {dist}\n")
        f.write("\n")

        # Distances to Charging Stations
        f.write("[ DISTANCES TO CHARGING STATIONS ]\n")
        for node_idx, stations in enumerate(distance_to_stations):
            f.write(f"  From Node {node_idx}:\n")
            for station_idx, dist in enumerate(stations):
                f.write(f"    -> Station {station_idx:<2} : {dist}\n")
        f.write("\n")

def save_to_file(filename, data):
    with open(filename, 'w') as f:
        # Writing a header for clarity
        f.write("Nodes,Time(s)\n")
        for row in data:
            f.write(f"{row[0]},{row[1]}\n")

def get_averaged_data(data):
    node_dict = defaultdict(list)
    for nodes, time in data:
        node_dict[nodes].append(time)
    
    # Calculate the average and sort by the number of nodes
    aggregated = []
    for nodes in sorted(node_dict.keys()):
        avg_time = np.mean(node_dict[nodes])
        aggregated.append([nodes, avg_time])
        
    return aggregated