#include <bits/stdc++.h>
using namespace std;

// ROAD TYPES:
// Type 1 → Bad-condition regular roads (shorter path, high charge consumption, slower)
// Type 2 → Good-condition highway roads (longer path, low charge consumption, faster)

enum RoadType { TYPE1 = 1, TYPE2 = 2 };

const double INF = 1e20;

// SPEED MODELS
const double time_per_distance_type1 = 0.12;   // slower on bad roads
const double time_per_distance_type2 = 0.08;   // faster on highway

// BATTERY MODELS
const double distance_per_charge_type1 = 2.8;   // consumes more charge per km
const double distance_per_charge_type2 = 4.2;   // consumes less charge per km

// CHARGING MODEL
const double time_per_charge_unit = 0.05;
const int charge_quantums = 50;

// CONSUMPTION MODELS (charge/km baseline)
const double base_consumption_type1 = 1.0 / distance_per_charge_type1;
const double base_consumption_type2 = 1.0 / distance_per_charge_type2;

// weight penalty
const double weight_penalty_factor_type1 = 0.00012;
const double weight_penalty_factor_type2 = 0.00006;

double time_for_distance(double dis, RoadType rt) {
    if (rt == TYPE1) return dis * time_per_distance_type1;
    return dis * time_per_distance_type2;
}

int charge_loss_for_distance(double dis, double cargo_weight, RoadType rt) {
    double eff;

    if (rt == TYPE1) {
        eff = base_consumption_type1 +
              cargo_weight * weight_penalty_factor_type1;
    } else {
        eff = base_consumption_type2 +
              cargo_weight * weight_penalty_factor_type2;
    }

    return (int)ceil(dis * eff);
}

double charging_time(int del_charge){
    return del_charge * time_per_charge_unit;
}

// Minimize total time. For every transition, evaluate BOTH variants (time-optimal and charge-optimal)
// and pick the transition that yields best arrival time while being feasible.
void path_checker_min_time(
    int delivery_points,
    int charging_stations,
    const vector<pair<double, double>> &times,
    const vector<double> &distance_type1,   // time-optimal A->B
    const vector<double> &distance_type2, // charge-optimal A->B (still allowed)
    const vector<vector<double>> &dis_c_type1,
    const vector<vector<double>> &dis_c_type2,
    const vector<double> &cargo_weight
) {

    // dp_n[node][rem_charge] = earliest arrival time at node with remaining battery rem_charge
    vector<vector<double>> dp_n(delivery_points, vector<double>(charge_quantums + 1, INF));
    // dp_c[node][station][rem_charge] = earliest arrival time at that station with rem_charge
    vector<vector<vector<double>>> dp_c(
        delivery_points, vector<vector<double>>(charging_stations, vector<double>(charge_quantums+1, INF))
    );

    // base
    dp_n[0][charge_quantums] = 0.0;

    for (int node = 0; node < delivery_points - 1; ++node) {
        double current_weight = cargo_weight[node];

        for (int rem = 0; rem <= charge_quantums; ++rem) {
            double arrival_time = dp_n[node][rem];
            if (arrival_time == INF) continue;

            // respect time window
            if (arrival_time < times[node].first) arrival_time = times[node].first;
            if (arrival_time > times[node].second) continue;

            // --- A -> B : evaluate both variants (time-optimal and charge-optimal) ---
            {
                // --- Variant TYPE 1 ---
                int loss1 = charge_loss_for_distance(distance_type1[node], current_weight, TYPE1);
                if (rem >= loss1) {
                    int new_rem = rem - loss1;
                    double tnew = arrival_time + time_for_distance(distance_type1[node], TYPE1);
                    dp_n[node+1][new_rem] = min(dp_n[node+1][new_rem], tnew);
                }

                // --- Variant TYPE 2 ---
                int loss2 = charge_loss_for_distance(distance_type2[node], current_weight, TYPE2);
                if (rem >= loss2) {
                    int new_rem = rem - loss2;
                    double tnew = arrival_time + time_for_distance(distance_type2[node], TYPE2);
                    dp_n[node+1][new_rem] = min(dp_n[node+1][new_rem], tnew);
                }

            }

            // --- A -> S : evaluate both node->station variants ---
            for (int station = 0; station < charging_stations; ++station) {
                // time-variant
                int loss1 = charge_loss_for_distance(dis_c_type1[node][station], current_weight, TYPE1);
                if (rem >= loss1) {
                    int rem_at_station = rem - loss1;
                    double t_to_station = arrival_time + time_for_distance(dis_c_type1[node][station], TYPE1);
                    if (t_to_station < dp_c[node][station][rem_at_station]) {
                        dp_c[node][station][rem_at_station] = t_to_station;
                    }
                }
                // charge-variant
                int loss2 = charge_loss_for_distance(dis_c_type2[node][station], current_weight, TYPE2);
                if (rem >= loss2) {
                    int rem_at_station = rem - loss2;
                    double t_to_station = arrival_time + time_for_distance(dis_c_type2[node][station], TYPE2);
                    if (t_to_station < dp_c[node][station][rem_at_station]) {
                        dp_c[node][station][rem_at_station] = t_to_station;
                    }
                }
            }
        }

        // charging at stations: same as before (time cost per unit)
        for (int station = 0; station < charging_stations; ++station) {
            for (int c = 1; c <= charge_quantums; ++c) {
                if (dp_c[node][station][c-1] == INF) continue;
                double cand_time = dp_c[node][station][c-1] + time_per_charge_unit;
                if (cand_time < dp_c[node][station][c]) dp_c[node][station][c] = cand_time;
            }
        }

        // S -> B: evaluate both station->node+1 variants
        for (int station = 0; station < charging_stations; ++station) {
            for (int rem = 0; rem <= charge_quantums; ++rem) {
                double at_station_time = dp_c[node][station][rem];
                if (at_station_time == INF) continue;

                // time-variant
                int loss1 = charge_loss_for_distance(dis_c_type1[node+1][station], current_weight, TYPE1);
                if (rem >= loss1) {
                    int new_rem = rem - loss1;
                    double tnew = at_station_time + time_for_distance(dis_c_type1[node+1][station], TYPE1);
                    if (tnew < dp_n[node+1][new_rem]) dp_n[node+1][new_rem] = tnew;
                }
                // charge-variant
                int loss2 = charge_loss_for_distance(dis_c_type2[node+1][station], current_weight, TYPE2);
                if (rem >= loss2) {
                    int new_rem = rem - loss2;
                    double tnew = at_station_time + time_for_distance(dis_c_type2[node+1][station], TYPE2);
                    if (tnew < dp_n[node+1][new_rem]) dp_n[node+1][new_rem] = tnew;
                }
            }
        }

        // prune dominated states by time (same as your original pruning)
        double min_time = INF;
        for (int k = charge_quantums; k >= 0; --k) {
            if (dp_n[node+1][k] > min_time) {
                dp_n[node+1][k] = INF; // prune
            } else {
                min_time = dp_n[node+1][k];
            }
        }
    }

    // Final answer: minimum time among rem charges
    double ans = INF;
    for (int rem = 0; rem <= charge_quantums; ++rem) {
        if (dp_n[delivery_points - 1][rem] < ans) ans = dp_n[delivery_points - 1][rem];
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
    // since it also contains the return node the time for final will be (0,1e9)
    vector<pair<double,double>> times(delivery_points);
    for (int i = 0; i < delivery_points; i++) cin >> times[i].first >> times[i].second;

    vector <double> delivery_weights(delivery_points);
    for (int i = 0; i < delivery_points; i++) cin >> delivery_weights[i];

    // take in two extra distances, depot to first city and last city to depot
    vector<double> distance_type1(delivery_points + 1);
    for (int i = 0; i <= delivery_points; i++) cin >> distance_type1[i];

    vector<double> distance_type2(delivery_points + 1);
    for (int i = 0; i <= delivery_points; i++) cin >> distance_type2[i];

    // distance of charging stations from node, here first array will be depot to charging station, so one extra
    vector<vector<double>> dis_c_type1(delivery_points + 1, vector<double>(charging_stations));
    for (int i = 0; i <= delivery_points; i++)
        for (int j = 0; j < charging_stations; j++)
            cin >> dis_c_type1[i][j];

    vector<vector<double>> dis_c_type2(delivery_points + 1, vector<double>(charging_stations));
    for (int i = 0; i <= delivery_points; i++)
        for (int j = 0; j < charging_stations; j++)
            cin >> dis_c_type2[i][j];

    // add depot start + end
    times.insert(times.begin(), {0, INF});
    times.push_back({0, INF});
    delivery_points += 2;

    // add one more dis_c row for final depot return (reuse first row)
    dis_c_type1.push_back(dis_c_type1[0]);
    dis_c_type1.push_back(dis_c_type2[0]);

    vector <double> cargo_weight(delivery_points);

    double total_weight = 0;
    for (double w : delivery_weights) {
        total_weight += w;
    }

    cargo_weight[0] = total_weight;
    for (int i = 1; i < delivery_points; i++) {
        cargo_weight[i] = cargo_weight[i-1] - delivery_weights[i];
    }

    path_checker_min_time(delivery_points,
        charging_stations,
        times,
        distance_type1,
        distance_type2, 
        dis_c_type1,
        dis_c_type2,
        cargo_weight
    );
}