#include <bits/stdc++.h>
using namespace std;

const double INF = 1e9;
const double time_per_distance = 0.1;
const double distance_per_charge = 1;
const double time_per_charge = 0.05;
const int charge_quantums = 50;
const double MAX_DIS = charge_quantums * distance_per_charge;
const double base_consumption_per_dist = 1.0 / distance_per_charge;
const double weight_penalty_factor = 0.001;

double time_for_distance(double dis){
    return time_per_distance * dis;
}

double distance_for_charge(int charge){
    return distance_per_charge * charge;
}

int charge_loss_for_distance(double dis, double cargo_weight){
    double effective_consumption = base_consumption_per_dist + (cargo_weight * weight_penalty_factor);
    double charge_loss = dis * effective_consumption;
    return ceil(charge_loss);
}

double charging_time(int del_charge){
    return del_charge * time_per_charge;
}

void path_checker(
    int delivery_points,
    int charging_stations,
    const vector<pair<double, double>> &times,
    const vector<double> &distance,
    const vector<vector<double>> &dis_c,
    const vector<double> &cargo_weight
) {
    // DP tables

    // dp_n[delivery_points][charge]
    vector<vector<double>> dp_n(delivery_points, vector<double>(charge_quantums + 1, INF));

    // dp_c[delivery_points][station][charge]
    vector<vector<vector<double>>> dp_c(
        delivery_points, vector<vector<double>>(charging_stations, vector<double>(charge_quantums + 1, INF))
    );

    // base case: full battery at depot
    dp_n[0][charge_quantums] = 0;

    // DP transitions
    for(int node = 0; node < delivery_points - 1; node++){

        double current_weight = cargo_weight[node];

        for(int charge = 0; charge <= charge_quantums; charge++){
            double arrival_time = dp_n[node][charge];
            if(arrival_time == INF) continue;

            // Respect time window
            if(arrival_time < times[node].first) arrival_time = times[node].first;
            if(arrival_time > times[node].second)continue;

            // Go directly to next city ( A -> B)
            if(charge >= charge_loss_for_distance(distance[node], current_weight)){
                int rem_charge = charge - charge_loss_for_distance(distance[node], current_weight);
                double arrival_time_next_node = arrival_time + time_for_distance(distance[node]);
                if(arrival_time_next_node < dp_n[node+1][rem_charge]) {
                    dp_n[node+1][rem_charge] = arrival_time_next_node;
                }
            }

            // Visit charging stations ( A -> S)
            for (int station = 0; station < charging_stations; station++) {
                double dist_to_station = dis_c[node][station];
                int charge_loss_to_station = charge_loss_for_distance(dist_to_station, current_weight);

                // can't reach station
                if(charge < charge_loss_to_station)continue;

                double time_to_station = arrival_time + time_for_distance(dist_to_station);
                int charge_at_station = charge - charge_loss_to_station;

                if(time_to_station < dp_c[node][station][charge_at_station]){
                    dp_c[node][station][charge_at_station] = time_to_station;
                }
            }
        }

        for (int station = 0; station < charging_stations; station++) {
            for (int c = 1; c <= charge_quantums; c++) {
                dp_c[node][station][c] = min(
                    dp_c[node][station][c],
                    dp_c[node][station][c - 1] + time_per_charge
                );
            }
        }

        // From stations to next city ( S -> B)
        for (int station = 0; station < charging_stations; station++) {
            for (int charge = 0; charge <= charge_quantums; charge++) {
                if (dp_c[node][station][charge] == INF) continue;
                double dist_to_next = dis_c[node+1][station];
                int charge_needed_to_next = charge_loss_for_distance(dist_to_next, current_weight);

                if (charge < charge_needed_to_next) continue;
                int rem_charge = charge - charge_needed_to_next;
                double arrival_time = dp_c[node][station][charge] + time_for_distance(dist_to_next);
                if(arrival_time < dp_n[node+1][rem_charge]){
                    dp_n[node+1][rem_charge] = arrival_time;
                }
            }
        }

        double min_tim e= INF;
        for (int k = charge_quantums; k >= 0; k--) {
            if (dp_n[node+1][k] > min_time) {
                dp_n[node+1][k] = INF; // Prune dominated state
            } else {
                min_time = dp_n[node+1][k];
            }
        }
    }

    // Final answer
    double ans = INF;
    for (int charge = 0; charge <= charge_quantums; charge++) {
        if (dp_n[delivery_points - 1][charge] < ans) {
            ans = dp_n[delivery_points - 1][charge];
        }
    }

    if (ans == INF) {
        cout << "IMPOSSIBLE\n";
        return;
    }

    cout << "Minimum total time: " << ans << "\n";
}

int main() {
    int delivery_points, charging_stations;
    cin >> delivery_points >> charging_stations;

    // assuming the time array provided is the delivery sequence the EV guy will be following
    // assuming one time unit for one kilometer
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

    vector <double> cargo_weight(delivery_points);

    double total_weight = 0;
    for (double w : delivery_weights) {
        total_weight += w;
    }

    cargo_weight[0] = total_weight;
    for (int i = 1; i < delivery_points; i++) {
        cargo_weight[i] = cargo_weight[i-1] - delivery_weights[i];
    }

    path_checker(delivery_points, charging_stations, times, distance, dis_c, cargo_weight);
}

// Time Complexity: O(Cities * Stations * Charge_quantums)
// Space Complexity: O(Cities * Stations * Charge_quantums)

// test case generator
// latex algorithm
// better approximate dp vs. new dp state with weights
// decouple the code (add relations between energy and distance as input)