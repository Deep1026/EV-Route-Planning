import math
import numpy as np
import time
import os
import shutil
import helper_functions
import glob
from docplex.mp.model import Model
import matplotlib.pyplot as plt
import csv

INF = 1e9
time_per_charge = 3
charge_quantums = 100 # This acts as 'B' (Battery Capacity) in CPLEX
vehicle_mass = 60
fuel_consumption_rate = 0.005
vehicle_speed = 1
fuel_capacity = 100

dp_time = []
cplex_time = []

def _time_for_distance(dist: float) -> float:
    return dist / vehicle_speed

def _charge_loss_for_distance(dist: float, weight: float) -> int:
    consumption_factor = (vehicle_mass + weight) * fuel_consumption_rate
    energy_consumed = dist * consumption_factor
    energy_per_quantum = fuel_capacity / charge_quantums
    quantums_lost = math.ceil(energy_consumed / energy_per_quantum)
    
    return quantums_lost

# DP
def _min_time_with_path(delivery_points, charging_stations, time_windows,
                         distance_between_points, distance_to_stations, cumulative_weight):

    dp_points   = np.full((delivery_points, charge_quantums + 1), INF)
    dp_stations = np.full((delivery_points, charging_stations, charge_quantums + 1), INF)

    parent_points_type    = np.full((delivery_points, charge_quantums + 1), -1, dtype=int)
    parent_points_station = np.full((delivery_points, charge_quantums + 1), -1, dtype=int)
    parent_points_charge  = np.full((delivery_points, charge_quantums + 1), -1, dtype=int)

    station_arrival_charge   = np.full((delivery_points, charging_stations, charge_quantums + 1), -1, dtype=int)
    station_src_node_charge  = np.full((delivery_points, charging_stations, charge_quantums + 1), -1, dtype=int)

    dp_points[0, charge_quantums] = 0

    for node in range(delivery_points - 1):
        current_weight = cumulative_weight[node]

        time_to_next_node        = _time_for_distance(distance_between_points[node])
        charge_loss_to_next_node = _charge_loss_for_distance(distance_between_points[node], current_weight)

        for charge in range(charge_quantums + 1):
            current_time = dp_points[node][charge]
            current_time = max(current_time, time_windows[node][0])
            if current_time > time_windows[node][1]:
                continue

            # A → B 
            if charge >= charge_loss_to_next_node:
                future_charge = charge - charge_loss_to_next_node
                new_time      = current_time + time_to_next_node
                if new_time < dp_points[node + 1, future_charge]:
                    dp_points[node + 1, future_charge]    = new_time
                    parent_points_type[node + 1, future_charge]    = 0   # direct
                    parent_points_station[node + 1, future_charge] = -1
                    parent_points_charge[node + 1, future_charge]  = charge

            # A → Station
            for station in range(charging_stations):
                charge_to_station = _charge_loss_for_distance(distance_to_stations[node][station], current_weight)
                time_to_station   = _time_for_distance(distance_to_stations[node][station])

                if charge >= charge_to_station:
                    arrival_charge = charge - charge_to_station
                    new_time       = current_time + time_to_station
                    if new_time < dp_stations[node, station, arrival_charge]:
                        dp_stations[node, station, arrival_charge]      = new_time
                        station_arrival_charge[node, station, arrival_charge]  = arrival_charge
                        station_src_node_charge[node, station, arrival_charge] = charge

        for station in range(charging_stations):
            for charge in range(1, charge_quantums + 1):
                new_time = dp_stations[node, station, charge - 1] + time_per_charge
                if new_time < dp_stations[node, station, charge]:
                    dp_stations[node, station, charge] = new_time
                    
                    station_arrival_charge[node, station, charge] = \
                        station_arrival_charge[node, station, charge - 1]
                    
            charge_loss_station_to_next = _charge_loss_for_distance(
                distance_to_stations[node + 1][station], current_weight)
            time_station_to_next = _time_for_distance(distance_to_stations[node + 1][station])

            # Station → B
            for charge in range(charge_quantums + 1):
                if dp_stations[node, station, charge] == INF:
                    continue
                if charge >= charge_loss_station_to_next:
                    future_charge = charge - charge_loss_station_to_next
                    new_time      = dp_stations[node, station, charge] + time_station_to_next
                    if new_time < dp_points[node + 1, future_charge]:
                        dp_points[node + 1, future_charge]    = new_time
                        parent_points_type[node + 1, future_charge]    = 1   # via station
                        parent_points_station[node + 1, future_charge] = station
                        parent_points_charge[node + 1, future_charge]  = charge  # charge when leaving station

        # Prune impossible states
        min_time = INF
        for charge in range(charge_quantums, -1, -1):
            min_time = min(min_time, dp_points[node + 1, charge])
            if dp_points[node + 1, charge] > min_time:
                dp_points[node + 1, charge] = INF

    ans        = INF
    best_charge = -1
    for charge in range(charge_quantums + 1):
        if dp_points[delivery_points - 1, charge] < ans:
            ans         = dp_points[delivery_points - 1, charge]
            best_charge = charge

    if ans == INF:
        return None, None

    # Backtrack
    path   = []
    node   = delivery_points - 1
    charge = best_charge

    while node > 0:
        src_type    = parent_points_type[node, charge]
        src_station = parent_points_station[node, charge]
        src_charge  = parent_points_charge[node, charge]

        if src_type == 0:
            # Direct
            path.append({
                'type':           'direct',
                'from_node':      node - 1,
                'to_node':        node,
                'charge_at_from': src_charge,
                'charge_at_to':   charge,
                'time_at_from':   dp_points[node - 1, src_charge],
                'time_at_to':     dp_points[node, charge],
            })
            charge = src_charge

        elif src_type == 1:
            charge_when_leaving   = src_charge
            charge_when_arrived   = station_arrival_charge[node - 1, src_station, charge_when_leaving]
            charge_before_detour  = station_src_node_charge[node - 1, src_station, charge_when_arrived]
            charge_added          = charge_when_leaving - charge_when_arrived

            path.append({
                'type':                     'via_station',
                'from_node':                node - 1,
                'to_node':                  node,
                'station':                  src_station,
                'charge_at_from':           charge_before_detour,
                'charge_at_station_arrive': charge_when_arrived,
                'charge_added':             charge_added,
                'charge_at_station_depart': charge_when_leaving,
                'charge_at_to':             charge,
                'time_at_from':             dp_points[node - 1, charge_before_detour],
                'time_at_station_arrive':   dp_stations[node - 1, src_station, charge_when_arrived],
                'time_at_station_depart':   dp_stations[node - 1, src_station, charge_when_leaving],
                'time_at_to':               dp_points[node, charge],
            })
            charge = charge_before_detour

        node -= 1

    path.reverse()
    return ans, path

def solve_via_dp(delivery_points, charging_stations, time_windows, distance_between_points, distance_to_stations, cargo_weight):
    dp_pts = delivery_points + 2
    tw = [[0, INF]] + time_windows + [[0, INF]]
    dts = distance_to_stations + [distance_to_stations[0]]
    
    total_weight = sum(cargo_weight)
    cumulative_weights = [total_weight]
    for weight in cargo_weight:
        total_weight -= weight
        cumulative_weights.append(total_weight)

    return _min_time_with_path(dp_pts, charging_stations, tw, distance_between_points, dts, cumulative_weights)

# CPLEX
def precompute_cplex_data(delivery_points, charging_stations, time_windows, distance_between_points, distance_to_stations, cargo_weight):

    n = delivery_points + 2
    B = charge_quantums
    J_C = list(range(charging_stations))
    L = list(range(1, B + 1))
    
    tw = [[0, INF]] + time_windows + [[0, INF]]
    dts = distance_to_stations + [distance_to_stations[0]]
    
    total_weight = sum(cargo_weight)
    cum_weights = [total_weight]
    for w in cargo_weight:
        total_weight -= w
        cum_weights.append(total_weight)

    a = {k: tw[k][0] for k in range(n)}
    b = {k: tw[k][1] for k in range(n)}
    
    # Build Cost Matrices
    t_dir, e_dir = {}, {}
    t_to_c, e_to_c = {}, {}
    t_from_c, e_from_c = {}, {}
    
    for k in range(n - 1):
        w = cum_weights[k]
        
        # Direct
        d_dir = distance_between_points[k]
        t_dir[k, k+1] = _time_for_distance(d_dir)
        e_dir[k, k+1] = _charge_loss_for_distance(d_dir, w)
        
        # To and From charging stations
        for c in J_C:
            d_to = dts[k][c]
            t_to_c[k, c] = _time_for_distance(d_to)
            e_to_c[k, c] = _charge_loss_for_distance(d_to, w)
            
            d_from = dts[k+1][c]
            t_from_c[c, k+1] = _time_for_distance(d_from)
            e_from_c[c, k+1] = _charge_loss_for_distance(d_from, w)
            
    # Charging times
    CT = {l: l * time_per_charge for l in L}
    
    return n, B, J_C, L, a, b, t_dir, t_to_c, t_from_c, e_dir, e_to_c, e_from_c, CT

# CPLEX MILP MODEL
def build_and_solve_cplex(n, B, J_C, L, a, b, t_dir, t_to_c, t_from_c, e_dir, e_to_c, e_from_c, CT):
    mdl = Model(name='EV_Min_Arrival_Time')
    R = list(range(n))
    
    # Variables
    t = mdl.continuous_var_dict(R, name='t')
    t_dep = mdl.continuous_var_dict(R, name='t_dep')
    y = mdl.continuous_var_dict(R, lb=0, ub=B, name='y')
    
    Q_keys = [(k, c, l) for k in range(n-1) for c in J_C for l in L]
    Q = mdl.binary_var_dict(Q_keys, name='Q')

    # Objective
    mdl.minimize(t[n-1])

    # Constraints
    for k in range(n-1):
        mdl.add_constraint(
            mdl.sum(Q[k, c, l] for c in J_C for l in L) <= 1, 
            ctname=f'max_one_charge_leg_{k}'
        )

    mdl.add_constraint(t[0] == 0, ctname='init_time')
    mdl.add_constraint(y[0] == B, ctname='init_battery')

    for k in R:
        # mdl.add_constraint(t[k] >= a[k], ctname=f'tw_lb_{k}')
        mdl.add_constraint(t[k] <= b[k], ctname=f'tw_ub_{k}')
        mdl.add_constraint(t_dep[k] >= t[k], ctname=f'dep_after_arr_{k}')
        mdl.add_constraint(t_dep[k] >= a[k], ctname=f'dep_after_open_{k}')

    for k in range(n-1):
        sum_Q = mdl.sum(Q[k, c, l] for c in J_C for l in L)
        
        detour_time = mdl.sum(
            Q[k, c, l] * (t_to_c[k, c] + CT[l] + t_from_c[c, k+1])
            for c in J_C for l in L
        )
        
        mdl.add_constraint(
            t[k+1] >= t_dep[k] + (1 - sum_Q) * t_dir[k, k+1] + detour_time,
            ctname=f'arrival_time_propagation_{k}'
        )

    for k in range(n-1):
        detour_energy_change = mdl.sum(
            Q[k, c, l] * (e_dir[k, k+1] - e_to_c[k, c] + l - e_from_c[c, k+1])
            for c in J_C for l in L
        )
        
        mdl.add_constraint(
            y[k+1] == y[k] - e_dir[k, k+1] + detour_energy_change,
            ctname=f'battery_propagation_{k}'
        )

    for k in range(n-1):
        for c in J_C:
            for l in L:
                mdl.add_constraint(
                    y[k] >= e_to_c[k, c] - B * (1 - Q[k, c, l]),
                    ctname=f'charge_reachability_{k}_{c}_{l}'
                )

    for k in range(n-1):
        for c in J_C:
            for l in L:
                mdl.add_constraint(
                    y[k] + l <= B + e_to_c[k, c] + B * (1 - Q[k, c, l]),
                    ctname=f'charge_capacity_limit_{k}_{c}_{l}'
                )
    mdl.parameters.timelimit = 120

    solution = mdl.solve(log_output=False)
    
    if solution:
        status = mdl.get_solve_status()
        if status.name == 'JobSolveStatus.TIME_LIMIT':
            return None, None
        obj_val = solution.get_objective_value()
        path = []
        
        for k in range(n - 1):
            charge_before_detour = solution.get_value(y[k])
            charge_at_to = solution.get_value(y[k+1])
            time_at_from = solution.get_value(t_dep[k])
            time_at_to = solution.get_value(t[k+1])
            
            detour_taken = False
            for c in J_C:
                for l in L:
                    if solution.get_value(Q[k, c, l]) > 0.5:
                        detour_taken = True
                        
                        charge_when_arrived = charge_before_detour - e_to_c[k, c]
                        charge_added = l
                        charge_when_leaving = charge_when_arrived + charge_added
                        
                        time_at_station_arrive = time_at_from + t_to_c[k, c]
                        time_at_station_depart = time_at_station_arrive + CT[l]
                        
                        path.append({
                            'type':                     'via_station',
                            'from_node':                k,
                            'to_node':                  k + 1,
                            'station':                  c,
                            'charge_at_from':           round(charge_before_detour, 2),
                            'charge_at_station_arrive': round(charge_when_arrived, 2),
                            'charge_added':             charge_added,
                            'charge_at_station_depart': round(charge_when_leaving, 2),
                            'charge_at_to':             round(charge_at_to, 2),
                            'time_at_from':             round(time_at_from, 2),
                            'time_at_station_arrive':   round(time_at_station_arrive, 2),
                            'time_at_station_depart':   round(time_at_station_depart, 2),
                            'time_at_to':               round(time_at_to, 2),
                        })
                        break
                if detour_taken:
                    break

            if not detour_taken:
                path.append({
                    'type':           'direct',
                    'from_node':      k,
                    'to_node':        k + 1,
                    'charge_at_from': round(charge_before_detour, 2),
                    'charge_at_to':   round(charge_at_to, 2),
                    'time_at_from':   round(time_at_from, 2),
                    'time_at_to':     round(time_at_to, 2),
                })
                
        return obj_val, path
    else:
        return None, None
    
cs = []
c = 0

def run_pipeline(dataset_path):

    global time_per_charge
    global fuel_capacity
    global vehicle_speed
    global fuel_consumption_rate
    global dp_time
    global cplex_time
    global cs
    global c
   
    # dataset_path = "/Users/deepdas/Desktop/EVRP/final/evrptw_instances/rc207_21.txt"

    instance_name = os.path.basename(dataset_path).split('.')[0]
    output_dir = f"results_pot/results_{instance_name}"
    os.makedirs(output_dir, exist_ok=True)

    shutil.copy(dataset_path, output_dir)

    depot, customers, stations, metadata = helper_functions.parse_custom_schneider_dataset(dataset_path)

    if (len(customers) > 30):
        vehicle_speed = 10
        fuel_consumption_rate = 0.0001
    else:
        vehicle_speed = 1
        fuel_consumption_rate = 0.005

    fixed_route = helper_functions.simple_sorted_insertion(depot, customers, vehicle_speed)

    # print("Dataset Metadata:", metadata)
    
    fuel_capacity = metadata.get('fuel_capacity', 77.75)
    inverse_refueling_rate = metadata.get('inverse_refueling_rate', 3.3)
    time_per_charge = ( fuel_capacity / charge_quantums ) * inverse_refueling_rate
    
    pipeline_inputs = helper_functions.prepare_pipeline_inputs(depot, fixed_route, stations)

    delivery_points = pipeline_inputs["delivery_points"]
    charging_stations = pipeline_inputs["charging_stations"]
    time_windows = pipeline_inputs["time_windows"]
    distance_between_points = pipeline_inputs["distance_between_points"]
    distance_to_stations = pipeline_inputs["distance_to_stations"]
    cargo_weight = pipeline_inputs["cargo_weight"]

    # if delivery_points < 15:
    #     c+=1
    #     return
    
    # if(c>10):
    #     return

    # print(time_windows)
    # print(distance_between_points)

    print(f"Extracted a route with {delivery_points} customers.")

    dp_txt_path = os.path.join(output_dir, "dp_results.txt")
    if(delivery_points==0):
        with open(dp_txt_path, "w") as f:
            f.write("Total delivery points 0\n")
        return
    
    # print(distance_between_points[1]-distance_between_points[0])
    helper_functions.save_inputs_to_file(os.path.join(output_dir, "formatted_inputs.txt"), delivery_points,
                                    charging_stations, time_windows, distance_between_points,
                                    distance_to_stations, cargo_weight, time_per_charge, fuel_capacity,
                                    _charge_loss_for_distance(distance_between_points[0],sum(cargo_weight)))

    dp_start = time.time()
    dp_ans, dp_path = solve_via_dp(
        delivery_points, charging_stations, time_windows, 
        distance_between_points, distance_to_stations, cargo_weight
    )
    dp_end = time.time()
    dp_time_taken = dp_end - dp_start

    print(f"DP Optimal Arrival Time: {dp_ans}")
    print(f"Time Taken by DP: {dp_time_taken:.4f}s")

    dp_time.append([delivery_points, dp_time_taken])

    dp_txt_path = os.path.join(output_dir, "dp_results.txt")
    if(dp_path==None):
        with open(dp_txt_path, "w") as f:
            f.write("No valid solution found\n")
    else:
        dp_plot_path = os.path.join(output_dir, "dp_plot.png")
        helper_functions.plot_charge_vs_time(dp_path, save_path=dp_plot_path)
        with open(dp_txt_path, "w") as f:
            f.write("=== DYNAMIC PROGRAMMING RESULTS ===\n")
            f.write(f"Optimal Arrival Time: {dp_ans}\n")
            f.write(f"Execution Time:       {dp_time_taken:.4f} seconds\n\n")
            f.write("=== PATH DETAILS ===\n")
            f.write(helper_functions.format_path_pretty(dp_path))

    if(delivery_points > 15):
        return
    
    cplex_start = time.time()
    cplex_args = precompute_cplex_data(
        delivery_points, charging_stations, time_windows, 
        distance_between_points, distance_to_stations, cargo_weight
    )
    cplex_ans, cplex_path = build_and_solve_cplex(*cplex_args)
    cplex_end = time.time()
    cplex_time_taken = cplex_end - cplex_start

    cplex_time.append([delivery_points, cplex_time_taken])

    print(f"CPLEX Optimal Arrival Time: {cplex_ans}")
    print(f"Time Taken by CPLEX: {cplex_time_taken:.4f}s")

    cplex_txt_path = os.path.join(output_dir, "cplex_results.txt")
    if(cplex_path==None):
        with open(cplex_txt_path, "w") as f:
            f.write("No valid solution found\n")
    else:
        cplex_plot_path = os.path.join(output_dir, "cplex_plot.png")
        helper_functions.plot_charge_vs_time(cplex_path, save_path=cplex_plot_path)
        with open(cplex_txt_path, "w") as f:
            f.write("=== CPLEX RESULTS ===\n")
            f.write(f"Optimal Arrival Time: {cplex_ans}\n")
            f.write(f"Execution Time:       {cplex_time_taken:.4f} seconds\n\n")
            f.write("=== PATH DETAILS ===\n")
            f.write(helper_functions.format_path_pretty(cplex_path))

    print("\n--- Comparison ---")
    
    if dp_ans is not None and cplex_ans is not None:
        cs.append([instance_name, delivery_points, charging_stations, dp_ans, cplex_ans, (abs(dp_ans - cplex_ans) / cplex_ans) * 100, dp_time_taken, cplex_time_taken])
        diff = abs(dp_ans - cplex_ans)
        print(f"Difference: {diff:.4f} time units.")
    elif dp_ans is not None and cplex_ans is None:
        cs.append([instance_name, delivery_points, charging_stations, dp_ans, "NA", "NA", dp_time_taken, cplex_time_taken])
    else:
        cs.append([instance_name, delivery_points, charging_stations, "NA", "NA", "NA", dp_time_taken, cplex_time_taken])
        print("One or both models failed to find a feasible solution.")

def add_to_csv(filename, data):
    with open(filename, mode='a', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(data)


if __name__ == "__main__":
    target_folder = "/Users/deepdas/Desktop/EVRP/final/evrptw_instances/*.txt"

    test_files = glob.glob(target_folder)

    csv_filename = "/Users/deepdas/Desktop/EVRP/final/time_results/data_2.csv"

    header = ['Instance', 'Delivery Points', 'Charging Stations', 'DP_time', 'MILP_time', 'Gap', 'DP ET', 'MILP ET']

    with open(csv_filename, mode='w', newline='') as f:
        csv.writer(f).writerow(header)

    for file_path in test_files:
    # file_path = "/Users/deepdas/Desktop/EVRP/final/evrptw_instances/C103C5.txt"
        run_pipeline(file_path)

    for i in cs:
        add_to_csv(csv_filename, i)

    # time_direc = "time_results"
    # os.makedirs(time_direc, exist_ok=True)
    # data_parser.save_to_file(os.path.join(time_direc, "dp_execution_times.txt"), dp_time)
    # data_parser.save_to_file(os.path.join(time_direc, "cplex_execution_times.txt"), cplex_time)

    # dp_agg = data_parser.get_averaged_data(dp_time)
    # cplex_agg = data_parser.get_averaged_data(cplex_time)

    # dp_nodes, dp_times = zip(*dp_agg)
    # cplex_nodes, cplex_times = zip(*cplex_agg)

    # plt.figure(figsize=(10, 6))
    # plt.plot(dp_nodes, dp_times, marker='o', linestyle='-', color='#2ca02c', label='DP Solver (Avg)')

    # plt.title('Execution Time Scaling: Dynamic Programming')
    # plt.xlabel('Number of Nodes')
    # plt.ylabel('Average Execution Time (seconds)')
    # plt.legend()
    # plt.grid(True, linestyle=':', alpha=0.7)

    # # Save and close the figure so it doesn't bleed into the next one
    # plt.savefig(os.path.join(time_direc, "dp_execution_time.png"), dpi=300, bbox_inches='tight')
    # plt.close()

    # # --- 2. Plot CPLEX Solver ---
    # plt.figure(figsize=(10, 6))
    # plt.plot(cplex_nodes, cplex_times, marker='s', linestyle='-', color='#d62728', label='CPLEX Solver (Avg)')

    # plt.title('Execution Time Scaling: CPLEX')
    # plt.xlabel('Number of Nodes')
    # plt.ylabel('Average Execution Time (seconds)')
    # plt.legend()
    # plt.grid(True, linestyle=':', alpha=0.7)

    # # Save the second plot separately
    # plt.savefig(os.path.join(time_direc, "cplex_execution_time.png"), dpi=300, bbox_inches='tight')
    # plt.close()