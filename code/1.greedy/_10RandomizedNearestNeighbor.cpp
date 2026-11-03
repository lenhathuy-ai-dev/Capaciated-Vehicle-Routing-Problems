// Algorithm 10: Randomized Nearest Neighbor (RNN) for CVRP.
// Code::Blocks / MinGW compatible (C++11). NO std::filesystem.
// Input:  <project>/new_data/<group>/<instance>.txt
// Output: <project>/code/1.greedy/_10RNN/<group>/<instance>.sol
//         <project>/code/1.greedy/_10RNN/summary.csv
// Format: n K Q OPT; then n rows: x y demand (depot is row 0).
// Distances: CVRPLIB rounded EUC_2D, as in Algorithms 1, 2, 4, 6, 8.
// Default: alpha=0.30, random_seed=42; configurable via command line.
// Every valid input gets a .sol, including FAILURE with unserved IDs.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

struct Point {
    double x, y;
    int demand;
    Point() : x(0.0), y(0.0), demand(0) {}
};

typedef std::vector< std::vector<int> > DistanceMatrix;

struct CVRPInstance {
    std::string name;
    int n, K, Q;
    long long reference;
    std::vector<Point> points;
    DistanceMatrix D;
    CVRPInstance() : n(0), K(0), Q(0), reference(-1) {}
};

struct RNNSolution {
    bool success;
    std::vector< std::vector<int> > routes;
    std::vector<int> loads;
    std::vector<int> unserved;
    long long distance;
    std::string failureReason;
    RNNSolution() : success(false), distance(0) {}
};

struct DataFile {
    std::string path;
    std::string relative;
};

// ==================== PATH HELPERS ====================
std::string joinPath(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (b.empty()) return a;
#ifdef _WIN32
    const char sep = '\\';
#else
    const char sep = '/';
#endif
    const char last = a[a.size() - 1];
    if (last == '/' || last == '\\') return a + b;
    return a + sep + b;
}

std::string parentPath(const std::string& path) {
    const std::size_t p = path.find_last_of("/\\");
    return (p == std::string::npos) ? "" : path.substr(0, p);
}

std::string fileStem(const std::string& path) {
    const std::size_t p = path.find_last_of("/\\");
    const std::string filename = path.substr(p == std::string::npos ? 0 : p + 1);
    const std::size_t dot = filename.find_last_of('.');
    return dot == std::string::npos ? filename : filename.substr(0, dot);
}

bool hasTxtExtension(const std::string& name) {
    if (name.size() < 4) return false;
    const std::string ext = name.substr(name.size() - 4);
    return ext[0] == '.' &&
           std::tolower(static_cast<unsigned char>(ext[1])) == 't' &&
           std::tolower(static_cast<unsigned char>(ext[2])) == 'x' &&
           std::tolower(static_cast<unsigned char>(ext[3])) == 't';
}

bool directoryExists(const std::string& path) {
#ifdef _WIN32
    const DWORD attributes = GetFileAttributesA(path.c_str());
    return attributes != INVALID_FILE_ATTRIBUTES &&
           (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
#else
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
#endif
}

void makeOneDirectory(const std::string& path) {
    if (path.empty() || directoryExists(path)) return;
#ifdef _WIN32
    if (!CreateDirectoryA(path.c_str(), NULL) && !directoryExists(path)) {
        throw std::runtime_error("Cannot create directory: " + path);
    }
#else
    if (mkdir(path.c_str(), 0755) != 0 && !directoryExists(path)) {
        throw std::runtime_error("Cannot create directory: " + path);
    }
#endif
}

void createDirectories(std::string path) {
#ifdef _WIN32
    std::replace(path.begin(), path.end(), '/', '\\');
    const char sep = '\\';
    const std::size_t start = (path.size() >= 3 && path[1] == ':') ? 3 : 0;
#else
    const char sep = '/';
    const std::size_t start = (!path.empty() && path[0] == '/') ? 1 : 0;
#endif
    for (std::size_t i = start; i <= path.size(); ++i) {
        if (i < path.size() && path[i] != sep) continue;
        if (i == 0) continue;
        const std::string part = path.substr(0, i);
#ifdef _WIN32
        if (part.size() == 2 && part[1] == ':') continue;
#endif
        makeOneDirectory(part);
    }
}

// ==================== RECURSIVE DATA SCAN ====================
void scanDataFiles(const std::string& directory,
                   const std::string& relative,
                   std::vector<DataFile>& files) {
#ifdef _WIN32
    WIN32_FIND_DATAA entry;
    HANDLE h = FindFirstFileA(joinPath(directory, "*").c_str(), &entry);
    if (h == INVALID_HANDLE_VALUE) {
        throw std::runtime_error("Cannot scan directory: " + directory);
    }
    do {
        const std::string name = entry.cFileName;
        if (name == "." || name == "..") continue;
        const std::string full = joinPath(directory, name);
        const std::string rel = relative.empty() ? name : joinPath(relative, name);
        if (entry.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            if (entry.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) continue;
            scanDataFiles(full, rel, files);
        } else if (hasTxtExtension(name)) {
            DataFile f;
            f.path = full;
            f.relative = rel;
            files.push_back(f);
        }
    } while (FindNextFileA(h, &entry));
    FindClose(h);
#else
    DIR* dir = opendir(directory.c_str());
    if (dir == NULL) throw std::runtime_error("Cannot scan directory: " + directory);
    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        const std::string name = entry->d_name;
        if (name == "." || name == "..") continue;
        const std::string full = joinPath(directory, name);
        const std::string rel = relative.empty() ? name : joinPath(relative, name);
        struct stat st;
        if (lstat(full.c_str(), &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            scanDataFiles(full, rel, files);
        } else if (S_ISREG(st.st_mode) && hasTxtExtension(name)) {
            DataFile f;
            f.path = full;
            f.relative = rel;
            files.push_back(f);
        }
    }
    closedir(dir);
#endif
}

bool sortDataFile(const DataFile& a, const DataFile& b) {
    return a.relative < b.relative;
}

std::vector<DataFile> findAllDataFiles(const std::string& dataRoot) {
    if (!directoryExists(dataRoot)) {
        throw std::runtime_error("Input folder does not exist: " + dataRoot);
    }
    std::vector<DataFile> files;
    scanDataFiles(dataRoot, "", files);
    std::sort(files.begin(), files.end(), sortDataFile);
    return files;
}

// ==================== READ DATA / DISTANCE MATRIX ====================
std::string trim(const std::string& s) {
    const std::size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    const std::size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

bool nextDataLine(std::istream& in, std::string& line) {
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' ||
            (line.size() >= 2 && line[0] == '/' && line[1] == '/')) continue;
        return true;
    }
    return false;
}

DistanceMatrix buildEuclideanDistanceMatrix(const std::vector<Point>& points) {
    const std::size_t n = points.size();
    DistanceMatrix D(n, std::vector<int>(n, 0));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double dx = points[i].x - points[j].x;
            const double dy = points[i].y - points[j].y;
            const double value = std::sqrt(dx * dx + dy * dy);
            if (!std::isfinite(value) ||
                value >= static_cast<double>(std::numeric_limits<int>::max()) - 0.5) {
                throw std::runtime_error("Invalid or too large Euclidean distance");
            }
            const int d = static_cast<int>(std::floor(value + 0.5)); // EUC_2D
            D[i][j] = D[j][i] = d;
        }
    }
    return D;
}

CVRPInstance readCVRPInstance(const std::string& path) {
    std::ifstream in(path.c_str());
    if (!in) throw std::runtime_error("Cannot open input file: " + path);

    CVRPInstance inst;
    inst.name = fileStem(path);
    std::string line;
    if (!nextDataLine(in, line)) throw std::runtime_error("Empty file");
    if (line.size() >= 3 &&
        static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF) {
        line.erase(0, 3);
    }

    std::istringstream header(line);
    if (!(header >> inst.n >> inst.K >> inst.Q)) {
        throw std::runtime_error("Expected first line: n K Q [OPT]");
    }
    if (!(header >> inst.reference)) inst.reference = -1;
    if (inst.n < 2 || inst.K < 1 || inst.Q < 1) {
        throw std::runtime_error("Require n>=2, K>=1, Q>=1");
    }

    inst.points.reserve(static_cast<std::size_t>(inst.n));
    while (nextDataLine(in, line)) {
        std::istringstream row(line);
        Point p;
        if (!(row >> p.x >> p.y >> p.demand)) {
            throw std::runtime_error("Invalid point row: " + line);
        }
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.demand < 0) {
            throw std::runtime_error("Invalid coordinate or demand");
        }
        inst.points.push_back(p);
        if (static_cast<int>(inst.points.size()) > inst.n) {
            throw std::runtime_error("Too many point rows");
        }
    }
    if (static_cast<int>(inst.points.size()) != inst.n) {
        throw std::runtime_error("Point row count differs from n");
    }
    if (inst.points[0].demand != 0) {
        throw std::runtime_error("Depot (node 0) must have demand 0");
    }
    inst.D = buildEuclideanDistanceMatrix(inst.points);
    return inst;
}

// ==================== ALGORITHM 10: RANDOMIZED NEAREST NEIGHBOR ====================
// Choose randomly and UNIFORMLY from the Restricted Candidate List (RCL):
// RCL = {j feasible and unvisited | D[current][j] <= d_min+alpha*(d_max-d_min)}
// Return -1 if there is no feasible unvisited customer.
// alpha=0: only nearest feasible candidates (random tie-breaking).
// alpha=1: all feasible candidates (uniform random).
int chooseRandomizedNearestNeighbor(
    const CVRPInstance& inst,
    int current,
    int load,
    const std::vector<unsigned char>& unvisited,
    double alpha,
    std::mt19937& rng
) {
    const int availableCapacity = inst.Q - load;
    std::vector<int> feasible;
    int dMin = std::numeric_limits<int>::max();
    int dMax = std::numeric_limits<int>::min();

    // Steps 10-13: Build feasible set F, find minimum and maximum distance.
    for (int j = 1; j < inst.n; ++j) {
        if (!unvisited[j]) continue;
        if (inst.points[j].demand > availableCapacity) continue;

        feasible.push_back(j);
        const int d = inst.D[current][j];
        if (d < dMin) dMin = d;
        if (d > dMax) dMax = d;
    }
    if (feasible.empty()) return -1; // NONE

    // Step 14: Calculate the threshold.
    const double threshold = static_cast<double>(dMin) + alpha *
        (static_cast<double>(dMax) - static_cast<double>(dMin));

    // Step 15: Build Restricted Candidate List (RCL).
    std::vector<int> rcl;
    for (std::size_t i = 0; i < feasible.size(); ++i) {
        const int j = feasible[i];
        if (static_cast<double>(inst.D[current][j]) <= threshold + 1e-9) {
            rcl.push_back(j);
        }
    }

    // The nearest feasible customer is always in RCL, hence nonempty.
    // Step 16: Uniform random choice from RCL.
    std::uniform_int_distribution<std::size_t> uniformIndex(0, rcl.size() - 1);
    return rcl[uniformIndex(rng)];
}

RNNSolution randomizedNearestNeighborCVRP(
    const CVRPInstance& inst,
    double alpha,
    unsigned int randomSeed
) {
    if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
        throw std::runtime_error("alpha must be in [0, 1]");
    }

    std::mt19937 rng(randomSeed); // Step 1: independent, repeatable per instance
    RNNSolution sol;
    std::vector<unsigned char> unvisited(static_cast<std::size_t>(inst.n), 1);
    unvisited[0] = 0;           // Step 2: U = all customers (except depot)
    int remaining = inst.n - 1;

    while (remaining > 0) {     // Step 4
        if (static_cast<int>(sol.routes.size()) >= inst.K) { // Step 5
            sol.failureReason = "Reached maximum number of vehicles K";
            break;
        }

        std::vector<int> route(1, 0);   // Step 6
        int current = 0;                // Step 7
        int load = 0;                   // Step 8

        while (true) {                  // Step 9
            const int j = chooseRandomizedNearestNeighbor(
                inst, current, load, unvisited, alpha, rng);
            if (j == -1) break;         // Step 11

            route.push_back(j);          // Step 17
            load += inst.points[j].demand; // Step 18
            unvisited[j] = 0;            // Step 19
            --remaining;
            current = j;                 // Step 20
        }

        // Avoid adding empty route 0->0 and infinite loop if demand[j]>Q.
        if (route.size() == 1) {
            sol.failureReason =
                "No unvisited customer fits an empty vehicle (demand > Q)";
            break;
        }

        route.push_back(0);              // Step 21
        for (std::size_t t = 1; t < route.size(); ++t) {
            sol.distance += inst.D[route[t - 1]][route[t]];
        }
        sol.routes.push_back(route);      // Step 22
        sol.loads.push_back(load);
    }

    for (int j = 1; j < inst.n; ++j) {
        if (unvisited[j]) sol.unserved.push_back(j);
    }
    sol.success = sol.unserved.empty();
    if (!sol.success && sol.failureReason.empty()) {
        sol.failureReason = "Not all customers could be served";
    }
    return sol;                          // Step 23 or FAILURE
}

// ==================== SAVE RESULTS ====================
// The depot 0 is implicit in the .sol route format, as in prior algorithms.
// Only a successful solution has a valid "Cost"; failures show "Partial cost".
void writeSolutionFile(const std::string& path,
                       const CVRPInstance& inst,
                       const RNNSolution& sol,
                       double alpha,
                       unsigned int randomSeed) {
    std::ofstream out(path.c_str());
    if (!out) throw std::runtime_error("Cannot write solution: " + path);

    for (std::size_t k = 0; k < sol.routes.size(); ++k) {
        out << "Route #" << (k + 1) << ":";
        for (std::size_t t = 1; t + 1 < sol.routes[k].size(); ++t) {
            out << " " << sol.routes[k][t];
        }
        out << '\n';
    }
    out << std::fixed << std::setprecision(4);
    out << "Status: " << (sol.success ? "SUCCESS" : "FAILURE") << '\n';
    out << "Alpha: " << alpha << '\n';
    out << "Random seed: " << randomSeed << '\n';
    out << "Vehicles used: " << sol.routes.size() << " / " << inst.K << '\n';
    out << "Served customers: "
        << (inst.n - 1 - static_cast<int>(sol.unserved.size()))
        << " / " << (inst.n - 1) << '\n';
    out << "Unserved customers (" << sol.unserved.size() << "):";
    if (sol.unserved.empty()) out << " none";
    for (std::size_t i = 0; i < sol.unserved.size(); ++i) {
        out << " " << sol.unserved[i];
    }
    out << '\n';

    long long unservedDemand = 0;
    for (std::size_t i = 0; i < sol.unserved.size(); ++i) {
        unservedDemand += inst.points[sol.unserved[i]].demand;
    }
    out << "Unserved demand: " << unservedDemand << '\n';
    if (sol.success) {
        out << "Cost " << sol.distance << '\n';
    } else {
        out << "Reason: " << sol.failureReason << '\n';
        out << "Partial cost: " << sol.distance << '\n';
    }
}

std::string csvCell(const std::string& s) {
    if (s.find_first_of(",\"\r\n") == std::string::npos) return s;
    std::string result = "\"";
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '"') result += '"';
        result += s[i];
    }
    return result + '"';
}

std::string toStringDouble(double value, int precision) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

template<typename T>
std::string toString(T value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

void writeCsvRow(std::ostream& out, const std::vector<std::string>& cols) {
    for (std::size_t i = 0; i < cols.size(); ++i) {
        if (i) out << ',';
        out << csvCell(cols[i]);
    }
    out << '\n';
}

// Positional arguments, all optional:
//     program.exe [dataRoot] [outputRoot] [alpha] [random_seed]
// Example: program.exe "D:\\...\\new_data" "D:\\...\\_10RNN" 0.30 42
// If no arguments are provided, the paths and defaults below are used.
int main(int argc, char* argv[]) {
    const std::string PROJECT_ROOT =
        "D:\\_2026\\1.CNTT\\7.metaheuristic\\01_VRP_Vehicle_Routing_Problem\\"
        "01_CVRP_Capacitated_Vehicle_Routing_Problem";

    const double DEFAULT_ALPHA = 0.30;
    const unsigned int DEFAULT_RANDOM_SEED = 42u;

    const std::string dataRoot = (argc > 1)
        ? argv[1] : joinPath(PROJECT_ROOT, "new_data");
    const std::string outputRoot = (argc > 2)
        ? argv[2] : joinPath(joinPath(joinPath(PROJECT_ROOT, "code"), "1.greedy"), "_10RNN");

    try {
        double alpha = DEFAULT_ALPHA;
        unsigned int randomSeed = DEFAULT_RANDOM_SEED;
        if (argc > 3) {
            std::istringstream parser(argv[3]);
            if (!(parser >> alpha) || !(parser >> std::ws).eof() ||
                !std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
                throw std::runtime_error("Invalid alpha: must be within [0,1]");
            }
        }
        if (argc > 4) {
            const std::string raw = argv[4];
            if (raw.empty()) throw std::runtime_error("random_seed cannot be empty");
            for (std::size_t i = 0; i < raw.size(); ++i) {
                if (!std::isdigit(static_cast<unsigned char>(raw[i]))) {
                    throw std::runtime_error("random_seed must be a non-negative integer");
                }
            }
            unsigned long long parsed = 0;
            std::istringstream parser(raw);
            if (!(parser >> parsed) || parsed > std::numeric_limits<unsigned int>::max()) {
                throw std::runtime_error("random_seed is too large");
            }
            randomSeed = static_cast<unsigned int>(parsed);
        }
        if (argc > 5) {
            throw std::runtime_error(
                "Usage: program.exe [dataRoot] [outputRoot] [alpha] [random_seed]");
        }

        const std::vector<DataFile> files = findAllDataFiles(dataRoot);
        createDirectories(outputRoot);
        const std::string summaryFile = joinPath(outputRoot, "summary.csv");
        std::ofstream summary(summaryFile.c_str());
        if (!summary) throw std::runtime_error("Cannot write summary.csv");
        summary << "group,instance,n,customers,K,Q,reference,alpha,random_seed,"
                   "status,vehicles_used,distance,gap_percent,time_ms,"
                   "partial_distance,unserved,reason\n";

        int ok = 0, failed = 0, errors = 0;
        std::cout << "Algorithm 10: Randomized Nearest Neighbor (RNN)\n"
                  << "Input: " << dataRoot << "\n"
                  << "Output: " << outputRoot << "\n"
                  << "Alpha: " << alpha << " | Random seed: " << randomSeed << "\n"
                  << "Files found: " << files.size() << "\n\n";

        for (std::size_t i = 0; i < files.size(); ++i) {
            const DataFile& f = files[i];
            const std::string group = parentPath(f.relative);
            const std::string instanceName = fileStem(f.relative);
            const std::string outputDir = joinPath(outputRoot, group);
            const std::string outBase = joinPath(outputDir, instanceName);
            try {
                createDirectories(outputDir);
                std::remove((outBase + ".sol").c_str());
                std::remove((outBase + ".fail.txt").c_str());
                std::remove((outBase + ".error.txt").c_str());

                const CVRPInstance inst = readCVRPInstance(f.path);
                const std::chrono::steady_clock::time_point start =
                    std::chrono::steady_clock::now();
                const RNNSolution sol =
                    randomizedNearestNeighborCVRP(inst, alpha, randomSeed);
                const std::chrono::steady_clock::time_point stop =
                    std::chrono::steady_clock::now();
                const double milliseconds =
                    std::chrono::duration<double, std::milli>(stop - start).count();
                writeSolutionFile(outBase + ".sol", inst, sol, alpha, randomSeed);

                if (sol.success) ++ok;
                else ++failed;

                std::string gap;
                if (sol.success && inst.reference > 0) {
                    const double percent = 100.0 *
                        (static_cast<double>(sol.distance) - inst.reference) /
                        static_cast<double>(inst.reference);
                    gap = toStringDouble(percent, 4);
                }
                std::vector<std::string> cols;
                cols.push_back(group);
                cols.push_back(inst.name);
                cols.push_back(toString(inst.n));
                cols.push_back(toString(inst.n - 1));
                cols.push_back(toString(inst.K));
                cols.push_back(toString(inst.Q));
                cols.push_back(toString(inst.reference));
                cols.push_back(toStringDouble(alpha, 4));
                cols.push_back(toString(randomSeed));
                cols.push_back(sol.success ? "SUCCESS" : "FAILURE");
                cols.push_back(toString(sol.routes.size()));
                cols.push_back(sol.success ? toString(sol.distance) : "");
                cols.push_back(gap);
                cols.push_back(toStringDouble(milliseconds, 3));
                cols.push_back(toString(sol.distance));
                cols.push_back(toString(sol.unserved.size()));
                cols.push_back(sol.failureReason);
                writeCsvRow(summary, cols);

                std::cout << '[' << (sol.success ? "SUCCESS" : "FAILURE") << "] "
                          << f.relative << " | vehicles=" << sol.routes.size()
                          << " | cost=";
                if (sol.success) std::cout << sol.distance;
                else std::cout << "N/A";
                std::cout << " | unserved=" << sol.unserved.size()
                          << " | time_ms=" << std::fixed << std::setprecision(3)
                          << milliseconds << '\n';
            } catch (const std::exception& e) {
                ++errors;
                std::ofstream err((outBase + ".error.txt").c_str());
                if (err) {
                    err << "Status: ERROR\nFile: " << f.relative
                        << "\nReason: " << e.what() << '\n';
                }
                std::vector<std::string> cols(17, "");
                cols[0] = group;
                cols[1] = instanceName;
                cols[7] = toStringDouble(alpha, 4);
                cols[8] = toString(randomSeed);
                cols[9] = "ERROR";
                cols[16] = e.what();
                writeCsvRow(summary, cols);
                std::cerr << "[ERROR] " << f.relative << " | " << e.what() << '\n';
            }
        }
        std::cout << "\nDONE: SUCCESS=" << ok << ", FAILURE=" << failed
                  << ", ERROR=" << errors << "\nSummary: " << summaryFile << '\n';
        return errors == 0 ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << '\n';
        return 1;
    }
}
