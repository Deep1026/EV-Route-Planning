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

    // Hard difficulty "trap"
    if (difficulty == "hard" && delivery_points >= 4 && charging_stations >= 2) {
        int trap_node_A = 2; // D2
        int trap_node_B = 3; // D3
        int trap_node_C = 4; // D4
        int charger_good = 0;
        int charger_bad = 1;

        cerr << "// --- TRAP INFO (for debugging) ---\n";
        cerr << "// Trapping leg D" << trap_node_A << " -> D" << trap_node_B << "\n";
        cerr << "// 'Desert' leg is D" << trap_node_B << " -> D" << trap_node_C << "\n";
        cerr << "// 'Good' charger is C" << charger_good << "\n";
        cerr << "// 'Bad' charger is C" << charger_bad << "\n";

        double desert_dist = MAX_DIS * 0.8;
        distance[trap_node_B] = desert_dist;
        cerr << "// Set D" << trap_node_B << "->D" << trap_node_C << " dist (desert): " << desert_dist << "\n";

        double direct_dist = MAX_DIS * 0.4;
        distance[trap_node_A] = direct_dist;
        cerr << "// Set D" << trap_node_A << "->D" << trap_node_B << " dist (direct): " << direct_dist << "\n";
        distance[trap_node_A - 1] = MAX_DIS * 0.5;

        // Charger positioning
        dis_c[trap_node_A][charger_good] = 5.0;
        dis_c[trap_node_B][charger_good] = 5.0;

        dis_c[trap_node_A][charger_bad] = 6.0;
        dis_c[trap_node_B][charger_bad] = 6.0;

        dis_c[trap_node_B][charger_good] = 5.0;
        dis_c[trap_node_C][charger_good] = 5.0;

        dis_c[trap_node_B][charger_bad] = 5.0;
        dis_c[trap_node_C][charger_bad] = 100.0;

        for (auto &t : times)
            t.second = t.first + 1000.0;
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
