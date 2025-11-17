#include <bits/stdc++.h>
#include <chrono>
using namespace std;
using namespace std::chrono;

// Constants
const double TIME_PER_DISTANCE = 0.1;
const double DISTANCE_PER_CHARGE = 3.0;
const double TIME_PER_CHARGE = 0.05;
const int CHARGE_QUANTUMS = 50;
const double MAX_DIS = CHARGE_QUANTUMS * DISTANCE_PER_CHARGE;
const double BASE_CONSUMPTION_PER_DIST = 1.0 / DISTANCE_PER_CHARGE;
const double WEIGHT_PENALTY_FACTOR = 0.0001;

struct TestCase {
    int cities;
    int stations;
    vector <pair <double, double>> times;
    vector <double> distance;
    vector <double> delivery_weights;
    vector <vector <double>> dis_c;
};

extern void path_checker_external(int cities, int stations,
    vector<pair<double,double>> &times,
    vector<double> &distance,
    vector<vector<double>> &dis_c,
    vector<double> &delivery_weights);

// Random double in range [a, b]
double rand_double(double a, double b) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_real_distribution<double> dist(a, b);
    return dist(gen);
}

// Euclidean distance between two 2D points
double get_dist(pair<double,double> p1, pair<double,double> p2) {
    return sqrt((p1.first - p2.first) * (p1.first - p2.first) +
                (p1.second - p2.second) * (p1.second - p2.second));
}

// Charge loss function
int get_charge_loss(double dis, double cargo_weight) {
    double effective_consumption = BASE_CONSUMPTION_PER_DIST + (cargo_weight * WEIGHT_PENALTY_FACTOR);
    double charge_loss = dis * effective_consumption;
    return (int)ceil(charge_loss);
}

TestCase generate_test_case(int delivery_points, int charging_stations, string difficulty) {
    int num_nodes = delivery_points + 1; // Depot + N deliveries

    // Generate random coordinates
    vector<pair<double,double>> nodes(num_nodes);
    for (int i = 0; i < num_nodes; i++)
        nodes[i] = {rand_double(0, 50), rand_double(0, 50)};

    vector<pair<double,double>> stations(charging_stations);
    for (int i = 0; i < charging_stations; i++)
        stations[i] = {rand_double(0, 50), rand_double(0, 50)};

    // Delivery weights
    vector<double> delivery_weights(delivery_points);
    for (int i = 0; i < delivery_points; i++)
        delivery_weights[i] = rand_double(5, 50);

    // Distances between delivery nodes (plus back to depot)
    vector<double> distance;
    for (int i = 0; i < num_nodes - 1; i++)
        distance.push_back(get_dist(nodes[i], nodes[i + 1]));
    distance.push_back(get_dist(nodes[num_nodes - 1], nodes[0]));

    // Distance from each node to each charging station
    vector<vector<double>> dis_c(num_nodes, vector<double>(charging_stations));
    for (int i = 0; i < num_nodes; i++) {
        for (int j = 0; j < charging_stations; j++) {
            dis_c[i][j] = get_dist(nodes[i], stations[j]);
        }
    }

    // Time windows
    vector<pair<double,double>> times;
    double current_time = 0.0;
    for (int i = 0; i < delivery_points; i++) {
        double travel_time = distance[i] * TIME_PER_DISTANCE;
        current_time += travel_time;
        double open_time = current_time;
        double close_time = current_time + 1000.0;
        times.push_back({open_time, close_time});
        current_time += 0.5;
    }

    // Hard difficulty: force lookahead trap
    if (difficulty == "hard" && delivery_points >= 4 && charging_stations >= 2) {
        int A = 2; // D2
        int B = 3; // D3
        int good = 0;

        double d_AB = MAX_DIS * 0.10;   // A→B (reachable from depot)
        double d_BC = MAX_DIS * 0.80;   // B→C (requires full or near full charge)

        distance[A] = d_AB;
        distance[B] = d_BC;

        dis_c[A][good] = 5.0;
        for(int j=0; j < charging_stations; j++){
            dis_c[B][j] = MAX_DIS * 2;
        }
    }

    // --- Output Test Case ---
    // cout << fixed << setprecision(2);
    // cout << delivery_points << " " << charging_stations << "\n";

    // for (auto &t : times) cout << t.first << " " << t.second << " ";
    // cout << "\n";

    // for (auto &w : delivery_weights) cout << w << " ";
    // cout << "\n";

    // for (auto &d : distance) cout << d << " ";
    // cout << "\n";

    // for (int i = 0; i < num_nodes; i++) {
    //     for (int j = 0; j < charging_stations; j++)
    //         cout << dis_c[i][j] << " ";
    //     cout << "\n";
    // }

    TestCase tc;
    tc.cities = delivery_points;
    tc.stations = charging_stations;
    tc.times = times;
    tc.distance = distance;
    tc.delivery_weights = delivery_weights;
    tc.dis_c = dis_c;

    return tc;
}

int main() {
    // ios::sync_with_stdio(false);
    // cin.tie(nullptr);

    cout << "Enter: <points> <stations> <difficulty> <simulations>" << endl;

    int points, stations, sims;
    cin >> points >> stations;
    string diff;
    cin >> diff;
    cin >> sims;

    double total_time = 0;
    for (int s = 0; s < sims; s++) {
        auto tc = generate_test_case(points, stations, diff);
        auto start = high_resolution_clock::now();
        cout << "\n--- Simulation #" << (s + 1) << " ---\n";
        path_checker_external(tc.cities, tc.stations, tc.times, tc.distance, tc.dis_c, tc.delivery_weights);
        auto end = high_resolution_clock::now();
        double duration = duration_cast<microseconds>(end - start).count();
        cout << "Simulation time: " << duration << " µs\n";
        total_time += duration;
    }

    cout << "\n\n-----------------Average Time Taken-------------\n";
    cout << total_time / sims << "\n\n\n\n\n";
    return 0;
}
