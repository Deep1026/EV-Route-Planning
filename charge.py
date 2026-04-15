import math
import heapq
import numpy as np
from dataclasses import dataclass

INF = 1e9
time_per_distance = 0.1
distance_per_charge = 5
time_per_charge = 0.05
charge_quantums = 500
MAX_DIS = charge_quantums * distance_per_charge
base_consumption_per_dist = 1 / distance_per_charge
weight_penalty_factor = 0.01

@dataclass(order=False)
class State:
    node: int
    current_charge: int
    time_taken: float
    total_charge_consumed: int

    def __lt__(self, other):
        if not isinstance(other, State):
            return NotImplemented
        if self.total_charge_consumed != other.total_charge_consumed:
            return self.total_charge_consumed < other.total_charge_consumed
        else:
            return self.time_taken < other.time_taken

def _time_for_distance(dist: float):
    return time_per_distance * dist

def _charge_loss_for_distance(dist: float, weight: float) -> int:
    consumption_factor = base_consumption_per_dist + weight * weight_penalty_factor
    return math.ceil(dist * consumption_factor)

def _min_charge(
    delivery_points: int,
    charging_stations: int,
    time_windows: list,
    distance_between_points: list,
    distance_to_stations: list,
    cumulative_weight: list,
):
    min_time = np.full((delivery_points, charge_quantums + 1), INF)
    pq = []
    heapq.heappush(pq, State(0, charge_quantums, 0.0, 0))
    min_time[0][charge_quantums] = 0

    min_total_charge = INF 
    total_time_taken_for_min_charge = INF

    while len(pq):
        current_state: State = heapq.heappop(pq)
        # print(current_state)

        if current_state.node == (delivery_points - 1):
            min_total_charge = current_state.total_charge_consumed
            total_time_taken_for_min_charge = current_state.time_taken
            break

        if current_state.time_taken > min_time[current_state.node][current_state.current_charge]:
            continue

        if current_state.node < (delivery_points - 1):

            # A -> B
            charge_loss_to_next = _charge_loss_for_distance(
                distance_between_points[current_state.node],
                cumulative_weight[current_state.node]
            )

            if current_state.current_charge >= charge_loss_to_next:
                travel_time = _time_for_distance(distance_between_points[current_state.node])
                arrival_time = max(time_windows[current_state.node+1][0], current_state.time_taken + travel_time)
                next_charge = current_state.current_charge - charge_loss_to_next

                if arrival_time <= time_windows[current_state.node+1][1]:
                    if arrival_time < min_time[current_state.node+1][next_charge]:
                        min_time[current_state.node+1][next_charge] = arrival_time
                        heapq.heappush(pq, State(
                            current_state.node + 1,
                            next_charge,
                            arrival_time,
                            current_state.total_charge_consumed + charge_loss_to_next
                        ))
            
            # A->S->B
            for station in range(charging_stations):
                charge_loss_to_station = _charge_loss_for_distance(
                    distance_to_stations[current_state.node][station],
                    cumulative_weight[current_state.node]
                )
                charge_loss_from_station_to_next_node = _charge_loss_for_distance(
                    distance_to_stations[current_state.node + 1][station],
                    cumulative_weight[current_state.node]
                )

                if current_state.current_charge >= charge_loss_to_station:
                    charge_at_station = current_state.current_charge - charge_loss_to_station
                    time_to_station = _time_for_distance(distance_to_stations[current_state.node][station])
                    time_from_station_to_next_node = _time_for_distance(distance_to_stations[current_state.node + 1][station])
                    for charge_added in range(0, charge_quantums - charge_at_station + 1):
                        time_to_charge = charge_added * time_per_charge
                        arrival_time = max(
                            time_windows[current_state.node+1][0],
                            current_state.time_taken + time_to_station + time_to_charge + time_from_station_to_next_node
                        )
                        if arrival_time <= time_windows[current_state.node+1][1]:
                            next_charge = charge_at_station + charge_added - charge_loss_from_station_to_next_node
                            if next_charge < 0:
                                continue
                            if arrival_time < min_time[current_state.node+1][next_charge]:
                                min_time[current_state.node+1][next_charge] = arrival_time
                                heapq.heappush(pq, State(
                                    current_state.node + 1,
                                    next_charge,
                                    arrival_time,
                                    current_state.total_charge_consumed + charge_loss_to_station + charge_loss_from_station_to_next_node
                                ))
    
    if min_total_charge == INF:
        print("IMPOSSIBLE")
    else:
        print("Minimum total charge used: ", min_total_charge)
        print("Time taken for minimum charge route: ", total_time_taken_for_min_charge)
    
    return min_total_charge

def minimum_charge(
    delivery_points: int,
    charging_stations: int,
    time_windows: list,
    distance_between_points: list,
    distance_to_stations: list,
    cargo_weight: list,
):
    # preprocess
    delivery_points += 2
    # print("time: ", len(time_windows))
    # print(time_windows)
    time_windows.insert(0, [0, INF])
    time_windows.append([0, INF])
    distance_to_stations.append(distance_to_stations[0])

    total_weight = sum(cargo_weight)

    cumulative_weights = []
    cumulative_weights.append(total_weight)

    for weight in cargo_weight:
        total_weight -= weight
        cumulative_weights.append(total_weight)

    # print("weight: ", len(cumulative_weights))
    # print("node: ", delivery_points)
    # print("time: ", len(time_windows))

    return _min_charge(
        delivery_points,
        charging_stations,
        time_windows,
        distance_between_points,
        distance_to_stations,
        cumulative_weights
    )
    