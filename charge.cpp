#include <bits/stdc++.h>
using namespace std;

const double INF = 1e9;
const double time_per_distance = 0.1;
const double distance_per_charge = 3;
const double time_per_charge = 0.05;
const int charge_quantums = 8;
const double weight_penalty_factor = 0.0001;
const double base_consumption_per_dist = 1.0 / distance_per_charge;

// Helper Calculations
static double time_for_distance(double dis) {
    return time_per_distance * dis;
}

static int charge_loss_for_distance(double dis, double cargo_weight) {
    double effective_consumption = base_consumption_per_dist + (cargo_weight * weight_penalty_factor);
    // Charge used is the metric we want to minimize
    double charge_loss = dis * effective_consumption;
    return ceil(charge_loss);
}

// The State for Dijkstra
struct State {
    int u;             
    int bat;
    double time;
    int total_charge;

    bool operator>(const State& other) const {
        return total_charge > other.total_charge;
    }
};

void path_checker_charge(
    int delivery_points,
    int charging_stations,
    vector<pair<double, double>> &times, // Time Windows
    vector<double> &distance,            // Dist A -> B
    vector<vector<double>> &dis_c,       // Dist A -> Station -> B logic needs care
    vector<double> &cargo_weight
) {
    // 1. DP Table for Pruning: Stores MIN TIME for a given state
    // We minimize charge via PQ, but we prune if we are too slow.
    vector<vector<double>> best_time(delivery_points, vector<double>(charge_quantums + 1, INF));

    // 2. Dijkstra Priority Queue
    priority_queue<State, vector<State>, greater<State>> pq;

    // Initial State: Node 0, Max Charge, Time 0, Total Consumed 0
    pq.push({0, charge_quantums, 0.0, 0});
    best_time[0][charge_quantums] = 0.0;

    double min_total_charge = INF, time_taken_for_min_charge = INF;

    while (!pq.empty()) {
        State current = pq.top();
        pq.pop();

        int u = current.u;
        int bat = current.bat;
        double time = current.time;
        int cost = current.total_charge;

        // --- GOAL CHECK ---
        if (u == delivery_points - 1) {
            min_total_charge = cost;
            time_taken_for_min_charge = time;
            break;
        }

        // --- PRUNING ---
        if (time > best_time[u][bat]) continue;

        // Current payload weight
        double cur_w = cargo_weight[u];
        
        // --- OPTION 1: Drive DIRECTLY to Next Node (u -> u+1) ---
        if (u < delivery_points - 1) {
            double dist_next = distance[u];
            int loss = charge_loss_for_distance(dist_next, cur_w);
            
            if (bat >= loss) {
                int next_bat = bat - loss;
                double travel_time = time_for_distance(dist_next);
                double arrival_time = time + travel_time;
                
                // Check Time Window for Next Node
                double window_start = times[u+1].first;
                double window_end = times[u+1].second;

                double final_arrival = max(arrival_time, window_start);

                if (final_arrival <= window_end) {
                    if (final_arrival < best_time[u+1][next_bat]) {
                        best_time[u+1][next_bat] = final_arrival;
                        pq.push({u + 1, next_bat, final_arrival, cost + loss});
                    }
                }
            }
        }

        // --- OPTION 2: Visit Charging Station (u -> Station -> u+1) ---
        if (u < delivery_points - 1) {
            for (int s = 0; s < charging_stations; s++) {
                // Dist: Node -> Station
                double d1 = dis_c[u][s]; 
                double d2 = dis_c[u+1][s]; 

                int loss1 = charge_loss_for_distance(d1, cur_w);
                int loss2 = charge_loss_for_distance(d2, cur_w);

                if (bat >= loss1) {
                    int bat_at_station = bat - loss1;
                    
                    for (int target_bat = bat_at_station; target_bat <= charge_quantums; target_bat++) {
                        int added_charge = target_bat - bat_at_station;
                        
                        // Can we leave station and reach next node?
                        if (target_bat >= loss2) {
                            int final_bat = target_bat - loss2;
                            
                            double t_travel = time_for_distance(d1) + time_for_distance(d2);
                            double t_charge = added_charge * time_per_charge;
                            double arrival_time = time + t_travel + t_charge;

                            double window_start = times[u+1].first;
                            double window_end = times[u+1].second;
                            double final_arrival = max(arrival_time, window_start);

                            if (final_arrival <= window_end) {
                                if (final_arrival < best_time[u+1][final_bat]) {
                                    best_time[u+1][final_bat] = final_arrival;
                                    pq.push({u + 1, final_bat, final_arrival, cost + loss1 + loss2});
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (min_total_charge == INF) cout << "IMPOSSIBLE\n";
    else{
        cout << "Minimum Total Charge Used: " << min_total_charge << "\n";
        cout << "Time Taken For Minimum Total Charge: " << time_taken_for_min_charge << "\n";
    }
}

void path_checker_charge_external(
    int delivery_points,
    int charging_stations,
    vector<pair<double, double>> times,
    vector<double> distance,
    vector<vector<double>> dis_c,
    vector<double> delivery_weights
){
    cout << times.back().first << endl;
    times.insert(times.begin(), {0, INF});
    times.push_back({0, INF});
    delivery_points += 2;

    // add one more dis_c row for final depot return (reuse first row)
    dis_c.push_back(dis_c[0]);

    vector <double> cargo_weight(delivery_points - 1);

    double total_weight = 0;
    for (double w : delivery_weights) {
        total_weight += w;
    }

    cargo_weight[0] = total_weight;
    for (int i = 1; i < delivery_points - 1; i++) {
        cargo_weight[i] = cargo_weight[i-1] - delivery_weights[i-1];
    }

    path_checker_charge(delivery_points, charging_stations, times, distance, dis_c, cargo_weight);
}

int main() {
    int delivery_points, charging_stations;
    cin >> delivery_points >> charging_stations;

    // assuming the time array provided is the delivery sequence the EV guy will be following
    // since it also contains the return node the time for final will be (0,1e9)
    vector<pair<double,double>> times(delivery_points);
    for (int i = 0; i < delivery_points; i++) cin >> times[i].first >> times[i].second;

    vector <double> delivery_weights(delivery_points);
    for (int i = 0; i < delivery_points; i++) cin >> delivery_weights[i];

    // take in two extra distances, depot to first city and last city to depot
    vector<double> distance(delivery_points + 1);
    for (int i = 0; i <= delivery_points; i++) cin >> distance[i];

    // distance of charging stations from node, here first array will be depot to charging station, so one extra
    vector<vector<double>> dis_c(delivery_points + 1, vector<double>(charging_stations));
    for (int i = 0; i <= delivery_points; i++)
        for (int j = 0; j < charging_stations; j++)
            cin >> dis_c[i][j];

    // add depot start + end
    times.insert(times.begin(), {0, INF});
    times.push_back({0, INF});
    delivery_points += 2;

    // add one more dis_c row for final depot return (reuse first row)
    dis_c.push_back(dis_c[0]);

    vector <double> cargo_weight(delivery_points - 1);

    double total_weight = 0;
    for (double w : delivery_weights) {
        total_weight += w;
    }

    cargo_weight[0] = total_weight;
    for (int i = 1; i < delivery_points - 1; i++) {
        cargo_weight[i] = cargo_weight[i-1] - delivery_weights[i-1];
    }

    path_checker_charge(delivery_points, charging_stations, times, distance, dis_c, cargo_weight);
}