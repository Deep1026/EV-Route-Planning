import math
from dataclasses import dataclass
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import numpy as np

INF = 1e9
TIME_PER_DISTANCE = 0.1
DISTANCE_PER_CHARGE = 5
TIME_PER_CHARGE = 0.05
CHARGE_QUANTUMS = 100
MAX_BATTERY = CHARGE_QUANTUMS 
BASE_CONSUMPTION = 1 / DISTANCE_PER_CHARGE
WEIGHT_PENALTY = 0.01

@dataclass
class Label:
    time: float
    consumed: int
    battery: float

def _calc_drive_time(dist):
    return TIME_PER_DISTANCE * dist

def _calc_consumption(dist, weight):
    factor = BASE_CONSUMPTION + (weight * WEIGHT_PENALTY)
    return math.ceil(dist * factor)

def prune_dominated(labels: list):
    if not labels:
        return []
    
    labels.sort(key=lambda x: x.time)
    
    pareto_front = []
    for candidate in labels:
        is_dominated = False
        for existing in pareto_front:
            if (existing.time <= candidate.time and 
                existing.consumed <= candidate.consumed and
                existing.battery >= candidate.battery
            ):
                is_dominated = True
                break
        
        if not is_dominated:
            pareto_front = [x for x in pareto_front if not (
                candidate.time <= x.time and 
                candidate.consumed <= x.consumed and 
                candidate.battery >= x.battery
            )]
            pareto_front.append(candidate)
            
    return pareto_front

def _solve_multiobjective(
    delivery_points,
    charging_stations,
    time_windows,
    dist_between_nodes,
    dist_to_stations,
    cargo_weights
):
    dp = [[] for _ in range(delivery_points)]
    
    # Start at Node 0 with Time=0, Consumed=0, Battery=MAX
    dp[0] = [Label(0, 0, MAX_BATTERY)]

    for i in range(delivery_points - 1):
        if not dp[i]: continue
        
        next_node = i + 1
        current_cargo = cargo_weights[i]
        
        candidates_next = []
        
        d_direct = dist_between_nodes[i]
        t_direct = _calc_drive_time(d_direct)
        cons_direct = _calc_consumption(d_direct, current_cargo)

        for label in dp[i]:
            if label.battery >= cons_direct:
                arrival_time = max(label.time + t_direct, time_windows[next_node][0])
                
                if arrival_time <= time_windows[next_node][1]:
                    candidates_next.append(Label(
                        arrival_time,
                        label.consumed + cons_direct,
                        label.battery - cons_direct
                    ))

        for s in range(charging_stations):
            d_to_s = dist_to_stations[i][s]
            d_from_s = dist_to_stations[next_node][s]
            
            t_to_s = _calc_drive_time(d_to_s)
            t_from_s = _calc_drive_time(d_from_s)
           
            consumption_to_s = _calc_consumption(d_to_s, current_cargo)
            consumption_from_s = _calc_consumption(d_from_s, current_cargo) 

            for label in dp[i]:
                if label.battery < consumption_to_s:
                    continue
                
                arrival_bat = label.battery - consumption_to_s
                arrival_time_at_s = label.time + t_to_s

                if consumption_from_s > MAX_BATTERY: continue
                
                min_charge = max(0, consumption_from_s - arrival_bat)
                max_charge = MAX_BATTERY - arrival_bat

                for charge_amt in range(min_charge, max_charge + 1):
                    
                    charge_time = charge_amt * TIME_PER_CHARGE
                    departure_time = arrival_time_at_s + charge_time
                    arrival_at_next = max(departure_time + t_from_s, time_windows[next_node][0])
                    
                    if arrival_at_next > time_windows[next_node][1]:
                        continue
                    
                    final_bat = arrival_bat + charge_amt - consumption_from_s
                    
                    candidates_next.append(Label(
                        arrival_at_next,
                        label.consumed + consumption_to_s + consumption_from_s,
                        final_bat
                    ))

        dp[next_node] = prune_dominated(candidates_next)

    final_labels = dp[delivery_points - 1]

    if final_labels:
        import numpy as np
        
        # Extract objectives
        times = np.array([label.time for label in final_labels])
        consumed = np.array([label.consumed for label in final_labels])

        # Sort by time (increasing)
        sorted_idx = np.argsort(times)
        times = times[sorted_idx]
        consumed = consumed[sorted_idx]

        plt.figure()

        # Plot discrete Pareto points
        plt.scatter(times, consumed)

        # Plot stepwise Pareto frontier
        plt.step(times, consumed, where='post')

        plt.xlabel("Total Time")
        plt.ylabel("Total Charge Consumed")
        plt.title("Pareto Front (Time vs Charge)")
        plt.grid(True)
        plt.show()

    return final_labels

def multiobjective(
    delivery_points: int,
    charging_stations: int,
    time_windows: list,
    distance_between_points: list,
    distance_to_stations: list,
    cargo_weight: list,
):
    # preprocess
    delivery_points += 2
    time_windows.insert(0, [0, INF])
    time_windows.append([0, INF])
    distance_to_stations.append(distance_to_stations[0])

    total_weight = sum(cargo_weight)

    cumulative_weights = []
    cumulative_weights.append(total_weight)

    for weight in cargo_weight:
        total_weight -= weight
        cumulative_weights.append(total_weight)

    return _solve_multiobjective(
        delivery_points,
        charging_stations,
        time_windows,
        distance_between_points,
        distance_to_stations,
        cumulative_weights
    )