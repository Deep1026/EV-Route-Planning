import math
import numpy as np
import time
from docplex.mp.model import Model

INF = 1e9
time_per_distance = 1
distance_per_charge = 2
time_per_charge = 0.05
charge_quantums = 50  # This acts as 'B' (Battery Capacity) in CPLEX
base_consumption_per_dist = 1 / distance_per_charge
weight_penalty_factor = 0.001

# 2. SHARED PHYSICS FUNCTIONS
def _time_for_distance(dist: float) -> float:
    return time_per_distance * dist

def _charge_loss_for_distance(dist: float, weight: float) -> int:
    consumption_factor = base_consumption_per_dist + weight * weight_penalty_factor
    return math.ceil(dist * consumption_factor)

# 3. DYNAMIC PROGRAMMING ALGORITHM
def _min_time(delivery_points, charging_stations, time_windows, distance_between_points, distance_to_stations, cumulative_weight):
    dp_points = np.full((delivery_points, charge_quantums + 1), INF)
    dp_stations = np.full((delivery_points, charging_stations, charge_quantums + 1), INF)

    dp_points[0, charge_quantums] = 0

    for node in range(delivery_points - 1):
        current_weight = cumulative_weight[node]
        time_to_next_node = _time_for_distance(distance_between_points[node])
        charge_loss_to_next_node = _charge_loss_for_distance(distance_between_points[node], current_weight)

        for charge in range(charge_quantums + 1):
            current_time = dp_points[node][charge]
            current_time = max(current_time, time_windows[node][0])
            if current_time > time_windows[node][1]:
                continue

            # Route: A -> B
            if charge >= charge_loss_to_next_node:
                future_charge = charge - charge_loss_to_next_node
                dp_points[node+1, future_charge] = min(dp_points[node+1, future_charge], current_time + time_to_next_node)
            
            # Route: A -> Station
            for station in range(charging_stations):
                charge_to_station = _charge_loss_for_distance(distance_to_stations[node][station], current_weight)
                time_to_station = _time_for_distance(distance_to_stations[node][station])
                if charge >= charge_to_station:
                    dp_stations[node, station, charge - charge_to_station] = min(
                        dp_stations[node, station, charge - charge_to_station], current_time + time_to_station)
        
        # At Station
        for station in range(charging_stations):
            # Charging
            for charge in range(1, charge_quantums + 1):
                dp_stations[node][station][charge] = min(
                    dp_stations[node, station, charge], dp_stations[node, station, charge - 1] + time_per_charge
                )

            charge_loss_to_next_node_from_station = _charge_loss_for_distance(distance_to_stations[node+1][station], current_weight)
            time_to_next_node_from_station = _time_for_distance(distance_to_stations[node+1][station])

            # Route: Station -> B
            for charge in range(charge_quantums + 1):
                if dp_stations[node, station, charge] == INF:
                    continue
                if charge >= charge_loss_to_next_node_from_station:
                    dp_points[node+1][charge - charge_loss_to_next_node_from_station] = min(
                        dp_points[node+1, charge - charge_loss_to_next_node_from_station],
                        dp_stations[node, station, charge] + time_to_next_node_from_station
                    )

        # Prune impossible states
        min_time = INF
        for charge in range(charge_quantums, -1, -1):
            min_time = min(min_time, dp_points[node+1, charge])
            if dp_points[node+1, charge] > min_time:
                dp_points[node+1, charge] = INF
        
    ans = INF
    for charge in range(charge_quantums+1):
        ans = min(ans, dp_points[delivery_points-1, charge])

    return ans if ans != INF else None

def solve_via_dp(delivery_points, charging_stations, time_windows, distance_between_points, distance_to_stations, cargo_weight):
    # Padding depot logic
    dp_pts = delivery_points + 2
    tw = [[0, INF]] + time_windows + [[0, INF]]
    dts = distance_to_stations + [distance_to_stations[0]]
    
    total_weight = sum(cargo_weight)
    cumulative_weights = [total_weight]
    for weight in cargo_weight:
        total_weight -= weight
        cumulative_weights.append(total_weight)

    return _min_time(dp_pts, charging_stations, tw, distance_between_points, dts, cumulative_weights)

# 4. PRECOMPUTATION BRIDGE FOR CPLEX
def precompute_cplex_data(delivery_points, charging_stations, time_windows, distance_between_points, distance_to_stations, cargo_weight):
    """
    Applies the exact same depot padding and weight math as the DP approach,
    baking it into static dictionaries that CPLEX can ingest.
    """
    n = delivery_points + 2
    B = charge_quantums
    J_C = list(range(charging_stations))
    L = list(range(1, B + 1))
    
    # 1. Pad inputs (matching DP depot logic)
    tw = [[0, INF]] + time_windows + [[0, INF]]
    dts = distance_to_stations + [distance_to_stations[0]]
    
    total_weight = sum(cargo_weight)
    cum_weights = [total_weight]
    for w in cargo_weight:
        total_weight -= w
        cum_weights.append(total_weight)
        
    # 2. Extract Time Windows
    a = {k: tw[k][0] for k in range(n)}
    b = {k: tw[k][1] for k in range(n)}
    
    # 3. Build Cost Matrices
    t_dir, e_dir = {}, {}
    t_to_c, e_to_c = {}, {}
    t_from_c, e_from_c = {}, {}
    
    for k in range(n - 1):
        w = cum_weights[k] # Current cargo weight on this leg
        
        # Direct leg
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

# 5. CPLEX MILP MODEL
def build_and_solve_cplex(n, B, J_C, L, a, b, t_dir, t_to_c, t_from_c, e_dir, e_to_c, e_from_c, CT):
    mdl = Model(name='EV_Min_Arrival_Time')
    R = list(range(n)) # Zero-indexed to match DP
    
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
        mdl.add_constraint(t[k] >= a[k], ctname=f'tw_lb_{k}')
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

    solution = mdl.solve(log_output=False)
    
    if solution:
        return solution.get_objective_value()
    else:
        return None

# 6. PIPELINE RUNNER
def run_pipeline():
   
    delivery_points = 3
    charging_stations = 2
    time_windows = [[0, 20], [20, 27], [30, 39]]
    distance_between_points = [15, 5, 10, 15] # n + 1 legs
    distance_to_stations = [
        [5, 10],  # From Depot start
        [10, 5],  # From Node 1
        [15, 8],  # From Node 2
        [10, 10], # From Node 3
    ]
    cargo_weight = [500, 300, 200]

    dp_start = time.time()
    dp_ans = solve_via_dp(
        delivery_points, charging_stations, time_windows, 
        distance_between_points, distance_to_stations, cargo_weight
    )
    dp_end = time.time()
    print(f"DP Optimal Arrival Time: {dp_ans}")
    print("Time Taken by DP: ", dp_end - dp_start)
    
    cplex_start = time.time()
    cplex_args = precompute_cplex_data(
        delivery_points, charging_stations, time_windows, 
        distance_between_points, distance_to_stations, cargo_weight
    )
    cplex_ans = build_and_solve_cplex(*cplex_args)
    cplex_end = time.time()
    print(f"CPLEX Optimal Arrival Time: {cplex_ans}")
    print("Time Taken by CPLEX: ", cplex_end - cplex_start)

    print("\n--- Comparison ---")
    
    if dp_ans is not None and cplex_ans is not None:
        diff = abs(dp_ans - cplex_ans)
        print(f"Difference: {diff:.4f} time units.")
    else:
        print("One or both models failed to find a feasible solution.")

if __name__ == "__main__":
    run_pipeline()