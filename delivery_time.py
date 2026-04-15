import math
import numpy as np

INF = 1e9
time_per_distance = 0.1
distance_per_charge = 5
time_per_charge = 0.05
charge_quantums = 50
MAX_DIS = charge_quantums * distance_per_charge
base_consumption_per_dist = 1 / distance_per_charge
weight_penalty_factor = 0.01

def _time_for_distance(dist: float):
    return time_per_distance * dist

def _charge_loss_for_distance(dist: float, weight: float) -> int:
    consumption_factor = base_consumption_per_dist + weight * weight_penalty_factor
    return math.ceil(dist * consumption_factor)

def _min_time(
    delivery_points: int,
    charging_stations: int,
    time_windows: list,
    distance_between_points: list,
    distance_to_stations: list,
    cumulative_weight: list,
):
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

            # A -> B
            if charge >= charge_loss_to_next_node:
                future_charge = charge - charge_loss_to_next_node
                dp_points[node+1, future_charge] = min(dp_points[node+1, future_charge],
                                                       current_time + time_to_next_node)
            
            # A -> S
            for station in range(charging_stations):
                charge_to_station = _charge_loss_for_distance(distance_to_stations[node][station],
                                                             current_weight)
                time_to_station = _time_for_distance(distance_to_stations[node][station])
                if charge >= charge_to_station:
                    dp_stations[node, station, charge - charge_to_station] = min(
                        dp_stations[node, station, charge - charge_to_station], current_time + time_to_station)
        
        # Station
        for station in range(charging_stations):
            # charging
            for charge in range(1, charge_quantums + 1):
                dp_stations[node][station][charge] = min(
                    dp_stations[node, station, charge], dp_stations[node, station, charge - 1] + time_per_charge
                )

            charge_loss_to_next_node_from_station = _charge_loss_for_distance(distance_to_stations[node+1][station], current_weight)
            time_to_next_node_from_station = _time_for_distance(distance_to_stations[node+1][station])

            # S -> B
            for charge in range(charge_quantums + 1):
                if dp_stations[node, station, charge] == INF:
                    continue
                if charge >= charge_loss_to_next_node_from_station:
                    dp_points[node+1][charge - charge_loss_to_next_node_from_station] = min(
                        dp_points[node+1, charge - charge_loss_to_next_node_from_station],
                        dp_stations[node, station, charge] + time_to_next_node_from_station
                    )

        # prune
        min_time = INF
        for charge in range(charge_quantums, -1, -1):
            min_time = min(min_time, dp_points[node+1, charge])
            if dp_points[node+1, charge] > min_time:
                dp_points[node+1, charge] = INF
        
    ans = INF
    for charge in range(charge_quantums+1):
        ans = min(ans, dp_points[delivery_points-1, charge])

    # print(dp_points)
    # for i in range(0, delivery_points):
    #     for j in range(0, charge_quantums + 1):
    #         if dp_points[i][j] == INF:
    #             print("100 ", end="")
    #         else:
    #             print(f"{dp_points[i][j]: 3.2f}", end=" ")
    #     print()

    if ans == INF:
        print("IMPOSSIBLE")
    else:
        print("Minimum time is ", ans)
    
    return ans

def minimum_time(
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

    return _min_time(
        delivery_points,
        charging_stations,
        time_windows,
        distance_between_points,
        distance_to_stations,
        cumulative_weights
    )