#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// =============================================================
// ALGORITHM 1: NEAREST NEIGHBOR FOR CVRP
// Input:  new_data/<group>/<instance>.txt
// Output: code/1.greedy/1_NNSolutions/<group>/<instance>.sol
//         code/1.greedy/1_NNSolutions/summary.csv
//
// new_data format (all node indices are 0-based, depot = 0):
//     n K Q OPT
//     x0 y0 demand0
//     x1 y1 demand1
//     ...
//     x(n-1) y(n-1) demand(n-1)
//
// n = number of nodes INCLUDING depot.
// OPT is optional; use -1 if unknown.
// Distances follow rounded EUC_2D = floor(hypot(dx,dy) + 0.5).
// =============================================================

struct Point {
    double x = 0.0;
    double y = 0.0;
    int demand = 0;
};

using DistanceMatrix = std::vector<std::vector<int>>;

struct CVRPInstance {
    std::string name;
    int n = 0;
    int K = 0;
    int Q = 0;
    long long reference = -1;
    std::vector<Point> points;
    DistanceMatrix D;
};

struct NNSolution {
    bool success = false;
    std::vector<std::vector<int>> routes; // [0, customer1, ..., 0]
    std::vector<int> loads;
    std::vector<int> unserved;
    long long distance = 0;               // Partial distance if FAILURE
    std::string failureReason;
};

std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

bool nextDataLine(std::istream& in, std::string& line) {
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line.rfind("//", 0) == 0) {
            continue;
        }
        return true;
    }
    return false;
}

// 1) Build the EUC_2D distance matrix.
DistanceMatrix buildEuclideanDistanceMatrix(const std::vector<Point>& points) {
    const std::size_t n = points.size();
    DistanceMatrix D(n, std::vector<int>(n, 0));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double dx = points[i].x - points[j].x;
            const double dy = points[i].y - points[j].y;
            const double euclidean = std::hypot(dx, dy);
            if (!std::isfinite(euclidean) ||
                euclidean > static_cast<double>(std::numeric_limits<int>::max()) - 0.5) {
                throw std::runtime_error("Euclidean distance is out of int range");
            }
            const int d = static_cast<int>(std::floor(euclidean + 0.5));
            D[i][j] = d;
            D[j][i] = d;
        }
    }
    return D;
}

// 2) Read one normalized data file.
CVRPInstance readCVRPInstance(const fs::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open input file");

    CVRPInstance inst;
    inst.name = path.stem().string();

    std::string line;
    if (!nextDataLine(in, line)) throw std::runtime_error("Empty data file");

    // Optional UTF-8 BOM.
    if (line.size() >= 3 &&
        static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF) {
        line.erase(0, 3);
    }

    {
        std::istringstream header(line);
        if (!(header >> inst.n >> inst.K >> inst.Q)) {
            throw std::runtime_error("Expected header: n K Q [OPT]");
        }
        if (!(header >> inst.reference)) inst.reference = -1;
    }
    if (inst.n <= 0 || inst.K <= 0 || inst.Q <= 0) {
        throw std::runtime_error("n, K and Q must be positive");
    }

    inst.points.reserve(static_cast<std::size_t>(inst.n));
    while (nextDataLine(in, line)) {
        Point p;
        std::istringstream row(line);
        if (!(row >> p.x >> p.y >> p.demand)) {
            throw std::runtime_error("Bad point row: " + line);
        }
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.demand < 0) {
            throw std::runtime_error("Invalid coordinates or negative demand");
        }
        inst.points.push_back(p);
        if (inst.points.size() > static_cast<std::size_t>(inst.n)) {
            throw std::runtime_error("More than n point rows");
        }
    }
    if (inst.points.size() != static_cast<std::size_t>(inst.n)) {
        throw std::runtime_error("Expected " + std::to_string(inst.n) +
                                 " point rows, found " + std::to_string(inst.points.size()));
    }
    if (inst.points.front().demand != 0) {
        throw std::runtime_error("Depot (node 0) must have demand 0");
    }
    inst.D = buildEuclideanDistanceMatrix(inst.points);
    return inst;
}

// 3) Nearest Neighbor EXACTLY following Algorithm 1.
//    Ties are resolved by taking the customer with the smaller index.
NNSolution nearestNeighborCVRP(const CVRPInstance& inst) {
    NNSolution sol;
    std::vector<unsigned char> unvisited(static_cast<std::size_t>(inst.n), 1);
    unvisited[0] = 0; // Depot is not a customer.
    int remaining = inst.n - 1;

    while (remaining > 0) {                                // Step 3
        if (static_cast<int>(sol.routes.size()) == inst.K) { // Step 4
            sol.failureReason = "Maximum number of vehicles K has been reached";
            break;
        }

        std::vector<int> route{0};                          // Step 5
        int load = 0;                                      // Step 6
        int current = 0;                                   // Step 7

        while (true) {                                     // Step 8
            int next = -1;
            int smallestDistance = std::numeric_limits<int>::max();

            // Steps 9-11: feasible unvisited customer nearest to current.
            for (int j = 1; j < inst.n; ++j) {
                if (!unvisited[j]) continue;
                if (inst.points[j].demand > inst.Q - load) continue;

                const int d = inst.D[current][j];
                if (next == -1 || d < smallestDistance ||
                    (d == smallestDistance && j < next)) {
                    next = j;
                    smallestDistance = d;
                }
            }
            if (next == -1) break;                         // Step 10

            route.push_back(next);                         // Step 12
            load += inst.points[next].demand;              // Step 13
            unvisited[next] = 0;                           // Step 14
            --remaining;
            current = next;                                // Step 15
        }

        // No customer fits even when leaving an empty vehicle from depot.
        // Without this guard the algorithm would create empty [0,0] routes.
        if (route.size() == 1) {
            sol.failureReason = "No remaining customer fits vehicle capacity Q";
            break;
        }

        route.push_back(0);                                // Step 16
        for (std::size_t t = 1; t < route.size(); ++t) {
            sol.distance += inst.D[route[t - 1]][route[t]];
        }
        sol.routes.push_back(std::move(route));             // Step 17
        sol.loads.push_back(load);
    }

    for (int j = 1; j < inst.n; ++j) {
        if (unvisited[j]) sol.unserved.push_back(j);
    }
    sol.success = sol.unserved.empty();                    // Step 18
    return sol;
}

// 4) Find all .txt input files recursively AND mirror all input directories.
std::vector<fs::path> findAllDataFiles(const fs::path& dataRoot,
                                       const fs::path& outputRoot) {
    if (!fs::exists(dataRoot) || !fs::is_directory(dataRoot)) {
        throw std::runtime_error("new_data folder not found: " + dataRoot.string());
    }
    fs::create_directories(outputRoot);
    std::vector<fs::path> files;
    for (const auto& entry : fs::recursive_directory_iterator(
             dataRoot, fs::directory_options::skip_permission_denied)) {
        const fs::path relative = entry.path().lexically_relative(dataRoot);
        if (entry.is_directory()) {
            fs::create_directories(outputRoot / relative);
        } else if (entry.is_regular_file()) {
            std::string extension = entry.path().extension().string();
            std::transform(extension.begin(), extension.end(), extension.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (extension == ".txt") files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

// 5) Save a successful solution in standard CVRPLIB-style .sol format.
void writeSolutionFile(const fs::path& outPath, const NNSolution& sol) {
    std::ofstream out(outPath);
    if (!out) throw std::runtime_error("Cannot write: " + outPath.string());
    for (std::size_t k = 0; k < sol.routes.size(); ++k) {
        out << "Route #" << (k + 1) << ":";
        // Do not print depot 0: it is implicit in CVRPLIB .sol files.
        for (std::size_t t = 1; t + 1 < sol.routes[k].size(); ++t) {
            out << ' ' << sol.routes[k][t];
        }
        out << '\n';
    }
    out << "Cost " << sol.distance << '\n';
}

// 6) Save FAILURE with its partial routes and unserved customers.
void writeFailureFile(const fs::path& outPath, const CVRPInstance& inst,
                      const NNSolution& sol) {
    std::ofstream out(outPath);
    if (!out) throw std::runtime_error("Cannot write: " + outPath.string());
    out << "Status: FAILURE\n"
        << "Instance: " << inst.name << '\n'
        << "Reason: " << sol.failureReason << '\n'
        << "Vehicles used: " << sol.routes.size() << " / " << inst.K << '\n'
        << "Partial distance: " << sol.distance << '\n';
    for (std::size_t k = 0; k < sol.routes.size(); ++k) {
        out << "Route #" << k + 1 << " (load=" << sol.loads[k] << "):";
        for (int node : sol.routes[k]) out << ' ' << node;
        out << '\n';
    }
    out << "Unserved customers (" << sol.unserved.size() << "):";
    for (int j : sol.unserved) out << ' ' << j;
    out << '\n';
}

std::string csvCell(const std::string& s) {
    if (s.find_first_of(",\"\r\n") == std::string::npos) return s;
    std::string result = "\"";
    for (char c : s) {
        if (c == '"') result += '"';
        result += c;
    }
    return result + '"';
}

int main(int argc, char* argv[]) {
    // Default Windows paths. Optional command-line arguments override them:
    //   nearest_neighbor_cvrp.exe <new_data_directory> <1_NNSolutions_directory>
    const fs::path PROJECT_ROOT =
        R"(D:\_2026\1.CNTT\7.metaheuristic\01_VRP_Vehicle_Routing_Problem\01_CVRP_Capacitated_Vehicle_Routing_Problem)";

    const fs::path dataRoot = argc > 1 ? fs::path(argv[1])
                                       : PROJECT_ROOT / "new_data";
    const fs::path outputRoot = argc > 2 ? fs::path(argv[2])
                                         : PROJECT_ROOT / "code" / "1.greedy" / "1_NNSolutions";
    try {
        const auto files = findAllDataFiles(dataRoot, outputRoot);
        const fs::path summaryPath = outputRoot / "summary.csv";
        std::ofstream summary(summaryPath);
        if (!summary) throw std::runtime_error("Cannot create summary.csv");
        summary << "group,instance,n,customers,K,Q,reference,status,vehicles_used,"
                   "distance,gap_percent,time_ms,partial_distance,unserved,reason\n";

        std::size_t ok = 0, failed = 0, errors = 0;
        std::cout << "Input : " << dataRoot.string() << '\n';
        std::cout << "Output: " << outputRoot.string() << '\n';
        std::cout << "Files : " << files.size() << "\n\n";

        for (const fs::path& file : files) {
            const fs::path relative = file.lexically_relative(dataRoot);
            const fs::path outBase = outputRoot / relative;
            fs::create_directories(outBase.parent_path());
            const std::string group = relative.parent_path().generic_string();
            const std::string instanceName = file.stem().string();

            // Remove stale outputs from previous runs (e.g., old .sol after FAILURE).
            for (const char* suffix : {".sol", ".fail.txt", ".error.txt"}) {
                fs::path oldFile = outBase;
                oldFile.replace_extension(suffix);
                std::error_code ec;
                fs::remove(oldFile, ec);
            }

            try {
                CVRPInstance inst = readCVRPInstance(file);
                const auto start = std::chrono::steady_clock::now();
                NNSolution sol = nearestNeighborCVRP(inst);
                const auto stop = std::chrono::steady_clock::now();
                const double timeMs =
                    std::chrono::duration<double, std::milli>(stop - start).count();
                fs::path outputFile = outBase;

                if (sol.success) {
                    outputFile.replace_extension(".sol");
                    writeSolutionFile(outputFile, sol);
                    ++ok;
                } else {
                    outputFile.replace_extension(".fail.txt");
                    writeFailureFile(outputFile, inst, sol);
                    ++failed;
                }

                std::ostringstream gap;
                if (sol.success && inst.reference > 0) {
                    gap << std::fixed << std::setprecision(4)
                        << (100.0 * (static_cast<double>(sol.distance - inst.reference) /
                                     static_cast<double>(inst.reference)));
                }

                summary << csvCell(group) << ',' << csvCell(inst.name) << ','
                        << inst.n << ',' << inst.n - 1 << ',' << inst.K << ','
                        << inst.Q << ',' << inst.reference << ','
                        << (sol.success ? "SUCCESS" : "FAILURE") << ','
                        << sol.routes.size() << ',';
                if (sol.success) summary << sol.distance;
                summary << ',' << gap.str() << ',' << std::fixed
                        << std::setprecision(3) << timeMs << ',' << sol.distance << ','
                        << sol.unserved.size() << ',' << csvCell(sol.failureReason) << '\n';

                std::cout << '[' << (sol.success ? "SUCCESS" : "FAILURE") << "] "
                          << relative.generic_string() << " | routes=" << sol.routes.size()
                          << " | cost=";
                if (sol.success) std::cout << sol.distance;
                else std::cout << "N/A (partial=" << sol.distance << ")";
                std::cout << " | unserved=" << sol.unserved.size()
                          << " | time_ms=" << std::fixed << std::setprecision(3)
                          << timeMs << '\n';

            } catch (const std::exception& e) {
                ++errors;
                fs::path errorFile = outBase;
                errorFile.replace_extension(".error.txt");
                std::ofstream err(errorFile);
                if (err) err << "Status: ERROR\nFile: " << relative.generic_string()
                             << "\nReason: " << e.what() << '\n';
                summary << csvCell(group) << ',' << csvCell(instanceName)
                        << ",,,,,,ERROR,,,,,,," << csvCell(e.what()) << '\n';
                std::cerr << "[ERROR] " << relative.generic_string()
                          << " | " << e.what() << '\n';
            }
        }
        std::cout << "\nDONE. SUCCESS=" << ok << ", FAILURE=" << failed
                  << ", ERROR=" << errors << '\n'
                  << "Summary: " << summaryPath.string() << '\n';
        return errors == 0 ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << '\n';
        return 1;
    }
}
