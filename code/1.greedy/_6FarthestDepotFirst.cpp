// Algorithm 6: Farthest Depot First (FDF) for CVRP
// C++11, compatible with Code::Blocks / MinGW / g++ (no std::filesystem).
// Input:  <project>/new_data/<group>/<instance>.txt
// Output: <project>/code/1.greedy/_6FDF/<group>/<instance>.sol
//         <project>/code/1.greedy/_6FDF/summary.csv
// A .sol file is written for BOTH success and failure; remaining customer IDs are included.
// Format: n K Q OPT ; then n lines: x y demand (depot is row 0).
// Distances: rounded Euclidean (CVRPLIB EUC_2D).

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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

struct FDFSolution {
    bool success;
    std::vector< std::vector<int> > routes;
    std::vector<int> loads;
    std::vector<int> unserved;
    long long distance;
    std::string failureReason;
    FDFSolution() : success(false), distance(0) {}
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

// ==================== ALGORITHM 6: FARTHEST DEPOT FIRST ====================
// U: all unvisited customers, excluding depot 0.
// Step 5: start each new route at the farthest feasible customer from depot.
// Steps 10-17: extend route with the nearest feasible unvisited customer.
// Ties are resolved by the smallest customer index to make runs reproducible.

// Return farthest customer from depot with demand <= Q, or -1 (NONE).
int chooseFarthestDepotSeed(const CVRPInstance& inst,
                           const std::vector<unsigned char>& unvisited) {
    int seed = -1;
    int farthestDistance = -1;
    for (int j = 1; j < inst.n; ++j) {
        if (!unvisited[j] || inst.points[j].demand > inst.Q) continue;
        const int distance = inst.D[0][j];
        if (distance > farthestDistance) {
            farthestDistance = distance;
            seed = j;
        }
    }
    return seed;
}

// Return nearest customer satisfying load + demand[j] <= Q, or -1.
int chooseNearestFeasibleCustomer(const CVRPInstance& inst,
                                  int current, int load,
                                  const std::vector<unsigned char>& unvisited) {
    int best = -1;
    int bestDistance = std::numeric_limits<int>::max();
    const int remainingCapacity = inst.Q - load;
    for (int j = 1; j < inst.n; ++j) {
        if (!unvisited[j] || inst.points[j].demand > remainingCapacity) continue;
        const int distance = inst.D[current][j];
        if (best == -1 || distance < bestDistance) {
            best = j;
            bestDistance = distance;
        }
    }
    return best;
}

FDFSolution farthestDepotFirstCVRP(const CVRPInstance& inst) {
    FDFSolution solution;
    std::vector<unsigned char> unvisited(static_cast<std::size_t>(inst.n), 1);
    unvisited[0] = 0;
    int remaining = inst.n - 1;

    while (remaining > 0) {                                  // Step 3
        if (static_cast<int>(solution.routes.size()) == inst.K) { // Step 4
            solution.failureReason = "Reached maximum number of vehicles K";
            break;
        }

        const int seed = chooseFarthestDepotSeed(inst, unvisited); // Step 5
        if (seed == -1) {
            solution.failureReason =
                "No unvisited customer fits an empty vehicle (demand > Q)";
            break;
        }

        std::vector<int> route;
        route.push_back(0);
        route.push_back(seed);                                // Step 6
        int load = inst.points[seed].demand;                    // Step 7
        int current = seed;                                   // Step 8
        unvisited[seed] = 0;                                  // Step 9
        --remaining;

        while (true) {                                        // Step 10
            const int j = chooseNearestFeasibleCustomer(
                inst, current, load, unvisited);               // Steps 11-13
            if (j == -1) break;                                // Step 12

            route.push_back(j);                                // Step 14
            load += inst.points[j].demand;                     // Step 15
            unvisited[j] = 0;                                 // Step 16
            --remaining;
            current = j;                                      // Step 17
        }

        route.push_back(0);                                    // Step 18
        for (std::size_t t = 1; t < route.size(); ++t) {
            solution.distance += inst.D[route[t - 1]][route[t]];
        }
        solution.routes.push_back(route);                      // Step 19
        solution.loads.push_back(load);
    }

    for (int j = 1; j < inst.n; ++j) {
        if (unvisited[j]) solution.unserved.push_back(j);
    }
    solution.success = solution.unserved.empty();              // Step 20
    if (!solution.success && solution.failureReason.empty()) {
        solution.failureReason = "Not all customers could be served";
    }
    return solution;
}

// ==================== SAVE SOLUTION / FAILURE / CSV ====================
// Always save routes and customer IDs in a .sol file, even on FAILURE.
// Keep "Cost <value>" only for feasible solutions. In an incomplete solution,
// "Partial cost" is NOT a feasible CVRP objective value.
void writeSolutionFile(const std::string& output,
                       const CVRPInstance& inst,
                       const FDFSolution& sol) {
    std::ofstream out(output.c_str());
    if (!out) throw std::runtime_error("Cannot write solution: " + output);

    // Route format is the same as Algorithms 1/2/4; depot 0 is implicit.
    for (std::size_t k = 0; k < sol.routes.size(); ++k) {
        out << "Route #" << (k + 1) << ":";
        for (std::size_t t = 1; t + 1 < sol.routes[k].size(); ++t) {
            out << " " << sol.routes[k][t];
        }
        out << '\n';
    }

    out << "Status: " << (sol.success ? "SUCCESS" : "FAILURE") << '\n';
    out << "Vehicles used: " << sol.routes.size() << " / " << inst.K << '\n';
    out << "Served customers: " << (inst.n - 1 - static_cast<int>(sol.unserved.size()))
        << " / " << (inst.n - 1) << '\n';

    out << "Unserved customers (" << sol.unserved.size() << "):";
    if (sol.unserved.empty()) out << " none";
    else {
        for (std::size_t j = 0; j < sol.unserved.size(); ++j) {
            out << " " << sol.unserved[j];
        }
    }
    out << '\n';

    long long unservedDemand = 0;
    for (std::size_t j = 0; j < sol.unserved.size(); ++j) {
        unservedDemand += inst.points[sol.unserved[j]].demand;
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

int main(int argc, char* argv[]) {
    const std::string PROJECT_ROOT =
        "D:\\_2026\\1.CNTT\\7.metaheuristic\\01_VRP_Vehicle_Routing_Problem\\"
        "01_CVRP_Capacitated_Vehicle_Routing_Problem";

    const std::string dataRoot = (argc > 1)
        ? argv[1] : joinPath(PROJECT_ROOT, "new_data");
    const std::string outputRoot = (argc > 2)
        ? argv[2] : joinPath(joinPath(joinPath(PROJECT_ROOT, "code"), "1.greedy"), "_6FDF");

    try {
        const std::vector<DataFile> files = findAllDataFiles(dataRoot);
        createDirectories(outputRoot);
        const std::string summaryFile = joinPath(outputRoot, "summary.csv");
        std::ofstream summary(summaryFile.c_str());
        if (!summary) throw std::runtime_error("Cannot write summary.csv");
        summary << "group,instance,n,customers,K,Q,reference,status,vehicles_used,"
                   "distance,gap_percent,time_ms,partial_distance,unserved,reason\n";

        int ok = 0, failed = 0, errors = 0;
        std::cout << "Algorithm: Farthest Depot First (FDF)\n"
                  << "Input: " << dataRoot << "\n"
                  << "Output: " << outputRoot << "\n"
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
                const FDFSolution sol = farthestDepotFirstCVRP(inst);
                const std::chrono::steady_clock::time_point stop =
                    std::chrono::steady_clock::now();
                const double milliseconds =
                    std::chrono::duration<double, std::milli>(stop - start).count();

                // Always create <instance>.sol, including when some customers
                // remain unserved. No separate .fail.txt is needed.
                writeSolutionFile(outBase + ".sol", inst, sol);
                if (sol.success) ++ok;
                else ++failed;

                std::ostringstream gap;
                if (sol.success && inst.reference > 0) {
                    gap << std::fixed << std::setprecision(4)
                        << 100.0 * (static_cast<double>(sol.distance) -
                                    static_cast<double>(inst.reference)) /
                               static_cast<double>(inst.reference);
                }

                summary << csvCell(group) << ',' << csvCell(inst.name) << ','
                        << inst.n << ',' << inst.n - 1 << ',' << inst.K << ','
                        << inst.Q << ',' << inst.reference << ','
                        << (sol.success ? "SUCCESS" : "FAILURE") << ','
                        << sol.routes.size() << ',';
                if (sol.success) summary << sol.distance;
                summary << ',' << gap.str() << ',' << std::fixed
                        << std::setprecision(3) << milliseconds << ','
                        << sol.distance << ',' << sol.unserved.size() << ','
                        << csvCell(sol.failureReason) << '\n';

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
                if (err) err << "Status: ERROR\nFile: " << f.relative
                             << "\nReason: " << e.what() << '\n';
                // 15 columns, blanks for unavailable fields.
                summary << csvCell(group) << ',' << csvCell(instanceName)
                        << ",,,,,,ERROR,,,,,,," << csvCell(e.what()) << '\n';
                std::cerr << "[ERROR] " << f.relative << " | " << e.what() << '\n';
            }
        }

        std::cout << "\nDONE: SUCCESS=" << ok
                  << ", FAILURE=" << failed
                  << ", ERROR=" << errors
                  << "\nSummary: " << summaryFile << '\n';
        return errors == 0 ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << '\n';
        return 1;
    }
}
