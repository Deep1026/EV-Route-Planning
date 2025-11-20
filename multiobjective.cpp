#include <bits/stdc++.h>
using namespace std;

// ROAD TYPES:
enum RoadType { TYPE1 = 1, TYPE2 = 2 };

const double INF = 1e20;

// SPEED MODELS
const double time_per_distance_type1 = 0.42;   // slower on bad roads
const double time_per_distance_type2 = 0.08;   // faster on highway

// BATTERY MODELS
const double distance_per_charge_type1 = 2.8;   // consumes more charge per km
const double distance_per_charge_type2 = 3.2;   // consumes less charge per km

// CHARGING MODEL
const double time_per_charge_unit = 0.05;
const int charge_quantums = 50;

// CONSUMPTION MODELS (charge/km baseline)
const double base_consumption_type1 = 1.0 / distance_per_charge_type1;
const double base_consumption_type2 = 1.0 / distance_per_charge_type2;

// weight penalty
const double weight_penalty_factor_type1 = 0.00012;
const double weight_penalty_factor_type2 = 0.00006;

struct State {
    double time;      // arrival time at that state
    int charge_used;  // cumulative charge consumed so far (we minimize this)

};

bool dominates(const State &a, const State &b) {
    return (a.time <= b.time && a.charge_used <= b.charge_used) &&
           (a.time < b.time || a.charge_used < b.charge_used);
}

void try_insert(vector<State> &frontier,const State &s) {
    for (auto it = frontier.begin(); it != frontier.end();) {
        if(it->time == s.time && it->charge_used == s.charge_used) return;
        if (dominates(*it, s)) return;          // existing dominates candidate -> discard candidate
        if (dominates(s, *it)) it = frontier.erase(it); // candidate dominates existing -> erase existing
        else ++it;
    }
    frontier.push_back(s);
}

double time_for_distance(double dis, RoadType rt) {
    if (rt == TYPE1) return dis * time_per_distance_type1;
    return dis * time_per_distance_type2;
}

int charge_loss_for_distance(double dis, double cargo_weight, RoadType rt) {
    double eff;

    if (rt == TYPE1) {
        eff = base_consumption_type1 + cargo_weight * weight_penalty_factor_type1;
    } else {
        eff = base_consumption_type2 + cargo_weight * weight_penalty_factor_type2;
    }

    return (int)ceil(dis * eff);
}

// Multi-objective DP: compute Pareto frontier of (time, total_charge_used)
void path_checker_pareto(
    int delivery_points,
    int charging_stations,
    const vector<pair<double, double>> &times,
    const vector<double> &distance_type1,
    const vector<double> &distance_type2,
    const vector<vector<double>> &dis_c_type1,
    const vector<vector<double>> &dis_c_type2,
    const vector<double> &cargo_weight
) {
    // dp_n[node][rem] = vector<State> (pareto frontier of (time, charge_used) arriving at node with rem charge)
    vector<vector<vector<State>>> dp_n(
        delivery_points, vector<vector<State>>(charge_quantums + 1)
    );

    // dp_c[node][station][rem] = frontier at (node, station) with rem charge
    vector<vector<vector<vector<State>>>> dp_c(
        delivery_points,
        vector<vector<vector<State>>>(charging_stations, vector<vector<State>>(charge_quantums + 1))
    );

    // base: at depot (node 0) with full charge and zero charge_used
    dp_n[0][charge_quantums].push_back({0.0, 0});

    for (int node = 0; node < delivery_points - 1; ++node) {
        double current_weight = cargo_weight[node];

        // 1) From node -> node+1 directly or via stations
        for (int rem = 0; rem <= charge_quantums; ++rem) {
            auto &frontier = dp_n[node][rem];
            if (frontier.empty()) continue;

            for (const State &st : frontier) {
                double arrival_time = st.time;
                if (arrival_time < times[node].first) arrival_time = times[node].first;
                if (arrival_time > times[node].second) continue; // infeasible due to time window

                // --- A -> B : both road variants ---
                // TYPE1
                {
                    int loss1 = charge_loss_for_distance(distance_type1[node], current_weight, TYPE1);
                    if (rem >= loss1) {
                        int new_rem = rem - loss1;
                        double tnew = arrival_time + time_for_distance(distance_type1[node], TYPE1);
                        State s = {tnew, st.charge_used + loss1};
                        try_insert(dp_n[node+1][new_rem], s);
                    }
                }
                // TYPE2
                {
                    int loss2 = charge_loss_for_distance(distance_type2[node], current_weight, TYPE2);
                    if (rem >= loss2) {
                        int new_rem = rem - loss2;
                        double tnew = arrival_time + time_for_distance(distance_type2[node], TYPE2);
                        State s = {tnew, st.charge_used + loss2};
                        try_insert(dp_n[node+1][new_rem], s);
                    }
                }

                // --- A -> S : node -> each station (both variants) ---
                for (int station = 0; station < charging_stations; ++station) {
                    // TYPE1 to station
                    {
                        int loss1 = charge_loss_for_distance(dis_c_type1[node][station], current_weight, TYPE1);
                        if (rem >= loss1) {
                            int rem_at_station = rem - loss1;
                            double t_to_station = arrival_time + time_for_distance(dis_c_type1[node][station], TYPE1);
                            State s = {t_to_station, st.charge_used + loss1};
                            try_insert(dp_c[node][station][rem_at_station], s);
                        }
                    }
                    // TYPE2 to station
                    {
                        int loss2 = charge_loss_for_distance(dis_c_type2[node][station], current_weight, TYPE2);
                        if (rem >= loss2) {
                            int rem_at_station = rem - loss2;
                            double t_to_station = arrival_time + time_for_distance(dis_c_type2[node][station], TYPE2);
                            State s = {t_to_station, st.charge_used + loss2};
                            try_insert(dp_c[node][station][rem_at_station], s);
                        }
                    }
                }
            }
        }

        // 2) Charging at stations: propagate from rem-1 -> rem by adding time_per_charge_unit (charge_used unchanged)
        for (int station = 0; station < charging_stations; ++station) {
            for (int c = 1; c <= charge_quantums; ++c) {
                auto &from = dp_c[node][station][c-1];
                if (from.empty()) continue;
                for (const State &st : from) {
                    State s = {st.time + time_per_charge_unit, st.charge_used};
                    try_insert(dp_c[node][station][c], s);
                }
            }
        }

        // 3) From station -> node+1 (both variants)
        for (int station = 0; station < charging_stations; ++station) {
            for (int rem = 0; rem <= charge_quantums; ++rem) {
                auto &frontier_at_station = dp_c[node][station][rem];
                if (frontier_at_station.empty()) continue;

                for (const State &st : frontier_at_station) {
                    // TYPE1 from station to node+1
                    {
                        int loss1 = charge_loss_for_distance(dis_c_type1[node+1][station], current_weight, TYPE1);
                        if (rem >= loss1) {
                            int new_rem = rem - loss1;
                            double tnew = st.time + time_for_distance(dis_c_type1[node+1][station], TYPE1);
                            State s = {tnew, st.charge_used + loss1};
                            try_insert(dp_n[node+1][new_rem], s);
                        }
                    }
                    // TYPE2 from station to node+1
                    {
                        int loss2 = charge_loss_for_distance(dis_c_type2[node+1][station], current_weight, TYPE2);
                        if (rem >= loss2) {
                            int new_rem = rem - loss2;
                            double tnew = st.time + time_for_distance(dis_c_type2[node+1][station], TYPE2);
                            State s = {tnew, st.charge_used + loss2};
                            try_insert(dp_n[node+1][new_rem], s);
                        }
                    }
                }
            }
        }
    }

    // Merge all dp_n[last_node][rem] into final Pareto frontier
    vector<State> final_frontier;
    int last = delivery_points - 1;
    for (int rem = 0; rem <= charge_quantums; ++rem) {
        for (const State &s : dp_n[last][rem]){
            try_insert(final_frontier, s);
        }
    }

    if (final_frontier.empty()) {
        cout << "IMPOSSIBLE\n";
        return;
    }

    sort(final_frontier.begin(), final_frontier.end(), [](const State &a, const State &b){
        if (a.charge_used != b.charge_used) return a.charge_used < b.charge_used;
        return a.time < b.time;
    });

    cout << "No. of solutions:" << final_frontier.size() << "\n";
    cout << "Pareto-optimal solutions (time, total_charge_used):\n";
    for (const State &s : final_frontier) {
        cout<<fixed<<setprecision(6);
        cout << s.time << " " << s.charge_used << "\n";
    }
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int delivery_points, charging_stations;
    if (!(cin >> delivery_points >> charging_stations)) return 0;

    vector<pair<double,double>> times(delivery_points);
    for (int i = 0; i < delivery_points; i++) cin >> times[i].first >> times[i].second;

    vector<double> delivery_weights(delivery_points);
    for (int i = 0; i < delivery_points; i++) cin >> delivery_weights[i];

    // take in two extra distances, depot to first city and last city to depot
    vector<double> distance_type1(delivery_points + 1);
    for (int i = 0; i <= delivery_points; i++) cin >> distance_type1[i];

    vector<double> distance_type2(delivery_points + 1);
    for (int i = 0; i <= delivery_points; i++) cin >> distance_type2[i];

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
    dis_c_type2.push_back(dis_c_type2[0]);

    vector<double> cargo_weight(delivery_points - 1);

    double total_weight = 0;
    for (double w : delivery_weights) total_weight += w;

    cargo_weight[0] = total_weight;
    for (int i = 1; i < delivery_points-1; i++) {
        cargo_weight[i] = cargo_weight[i-1] - delivery_weights[i-1];
    }

    path_checker_pareto(delivery_points,
        charging_stations,
        times,
        distance_type1,
        distance_type2,
        dis_c_type1,
        dis_c_type2,
        cargo_weight
    );

    return 0;
}
