// Nearest Neighbor for CVRP - Code::Blocks / MinGW compatible (C++11).
// No std::filesystem is used.
// Input:  <project>/new_data/<group>/<instance>.txt
// Output: <project>/code/1.greedy/1_NNSolutions/<group>/<instance>.sol
//         <project>/code/1.greedy/1_NNSolutions/summary.csv
// Data format: n K Q OPT ; then n lines: x y demand (depot is row 0).

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
#include <cerrno>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

struct Point {
    double x, y;
    int demand;
    Point() : x(0), y(0), demand(0) {}
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

struct NNSolution {
    bool success;
    std::vector< std::vector<int> > routes;
    std::vector<int> loads;
    std::vector<int> unserved;
    long long distance;
    std::string failureReason;
    NNSolution() : success(false), distance(0) {}
};

struct DataFile {
    std::string path;       // Absolute file path
    std::string relative;   // e.g. A/A-n32-k5.txt
};

// -------------------- PATH HELPERS --------------------
std::string joinPath(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (b.empty()) return a;
#ifdef _WIN32
    const char sep = '\\';
#else
    const char sep = '/';
#endif
    const char tail = a[a.size() - 1];
    if (tail == '/' || tail == '\\') return a + b;
    return a + sep + b;
}

std::string parentPath(const std::string& path) {
    const std::string::size_type p = path.find_last_of("/\\");
    return (p == std::string::npos) ? "" : path.substr(0, p);
}

std::string fileStem(const std::string& path) {
    const std::string::size_type p = path.find_last_of("/\\");
    const std::string filename = path.substr(p == std::string::npos ? 0 : p + 1);
    const std::string::size_type dot = filename.find_last_of('.');
    return dot == std::string::npos ? filename : filename.substr(0, dot);
}

bool hasTxtExtension(const std::string& name) {
    if (name.size() < 4) return false;
    const std::string suffix = name.substr(name.size() - 4);
    return suffix[0] == '.' &&
           std::tolower(static_cast<unsigned char>(suffix[1])) == 't' &&
           std::tolower(static_cast<unsigned char>(suffix[2])) == 'x' &&
           std::tolower(static_cast<unsigned char>(suffix[3])) == 't';
}

bool directoryExists(const std::string& path) {
#ifdef _WIN32
    DWORD a = GetFileAttributesA(path.c_str());
    return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY) != 0;
#else
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
#endif
}

void makeOneDirectory(const std::string& path) {
    if (path.empty()) return;
    if (directoryExists(path)) return;
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

// Create missing ancestors, including 1_NNSolutions and nested group folders.
void createDirectories(std::string path) {
#ifdef _WIN32
    std::replace(path.begin(), path.end(), '/', '\\');
    const char sep = '\\';
    std::size_t start = (path.size() >= 3 && path[1] == ':') ? 3 : 0;
#else
    const char sep = '/';
    std::size_t start = path.size() > 0 && path[0] == '/' ? 1 : 0;
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

// -------------------- RECURSIVE FILE SCAN --------------------
void scanDataFiles(const std::string& directory, const std::string& relative,
                   const std::string& outputRoot, std::vector<DataFile>& files) {
#ifdef _WIN32
    WIN32_FIND_DATAA entry;
    HANDLE h = FindFirstFileA(joinPath(directory, "*").c_str(), &entry);
    if (h == INVALID_HANDLE_VALUE) {
        throw std::runtime_error("Cannot scan directory: " + directory);
    }
    do {
        const std::string name = entry.cFileName;
        if (name == "." || name == "..") continue;
        const std::string abs = joinPath(directory, name);
        const std::string rel = relative.empty() ? name : joinPath(relative, name);
        if (entry.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            if (entry.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) continue;
            createDirectories(joinPath(outputRoot, rel));
            scanDataFiles(abs, rel, outputRoot, files);
        } else if (hasTxtExtension(name)) {
            DataFile f;
            f.path = abs;
            f.relative = rel;
            files.push_back(f);
        }
    } while (FindNextFileA(h, &entry));
    FindClose(h);
#else
    DIR* d = opendir(directory.c_str());
    if (d == NULL) throw std::runtime_error("Cannot scan directory: " + directory);
    struct dirent* e;
    while ((e = readdir(d)) != NULL) {
        const std::string name = e->d_name;
        if (name == "." || name == "..") continue;
        const std::string abs = joinPath(directory, name);
        const std::string rel = relative.empty() ? name : joinPath(relative, name);
        struct stat st;
        if (lstat(abs.c_str(), &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            createDirectories(joinPath(outputRoot, rel));
            scanDataFiles(abs, rel, outputRoot, files);
        } else if (S_ISREG(st.st_mode) && hasTxtExtension(name)) {
            DataFile f;
            f.path = abs;
            f.relative = rel;
            files.push_back(f);
        }
    }
    closedir(d);
#endif
}

bool sortDataFile(const DataFile& a, const DataFile& b) {
    return a.relative < b.relative;
}

std::vector<DataFile> findAllDataFiles(const std::string& dataRoot,
                                       const std::string& outputRoot) {
    if (!directoryExists(dataRoot)) {
        throw std::runtime_error("Input folder does not exist: " + dataRoot);
    }
    createDirectories(outputRoot);
    std::vector<DataFile> files;
    scanDataFiles(dataRoot, "", outputRoot, files);
    std::sort(files.begin(), files.end(), sortDataFile);
    return files;
}

// -------------------- READ CVRP DATA --------------------
std::string trim(const std::string& s) {
    const std::string::size_type a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    const std::string::size_type b = s.find_last_not_of(" \t\r\n");
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

DistanceMatrix buildEuclideanDistanceMatrix(const std::vector<Point>& p) {
    const std::size_t n = p.size();
    DistanceMatrix D(n, std::vector<int>(n, 0));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double dx = p[i].x - p[j].x;
            const double dy = p[i].y - p[j].y;
            const double value = std::sqrt(dx * dx + dy * dy);
            if (!(value >= 0.0) ||
                value >= static_cast<double>(std::numeric_limits<int>::max()) - 0.5) {
                throw std::runtime_error("Invalid or too large Euclidean distance");
            }
            const int dist = static_cast<int>(std::floor(value + 0.5)); // EUC_2D
            D[i][j] = D[j][i] = dist;
        }
    }
    return D;
}

CVRPInstance readCVRPInstance(const std::string& path) {
    std::ifstream in(path.c_str());
    if (!in) throw std::runtime_error("Cannot open input file: " + path);
    CVRPInstance instance;
    instance.name = fileStem(path);
    std::string line;
    if (!nextDataLine(in, line)) throw std::runtime_error("Empty file");
    if (line.size() >= 3 &&
        static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF) line.erase(0, 3);

    std::istringstream header(line);
    if (!(header >> instance.n >> instance.K >> instance.Q)) {
        throw std::runtime_error("Expected first line: n K Q [OPT]");
    }
    if (!(header >> instance.reference)) instance.reference = -1;
    if (instance.n < 2 || instance.K < 1 || instance.Q < 1) {
        throw std::runtime_error("n >= 2, K >= 1 and Q >= 1 required");
    }
    instance.points.reserve(static_cast<std::size_t>(instance.n));
    while (nextDataLine(in, line)) {
        std::istringstream row(line);
        Point p;
        if (!(row >> p.x >> p.y >> p.demand)) {
            throw std::runtime_error("Invalid row: " + line);
        }
        if (!(p.x == p.x) || !(p.y == p.y) || p.demand < 0) {
            throw std::runtime_error("Invalid coordinates or demand");
        }
        instance.points.push_back(p);
        if (static_cast<int>(instance.points.size()) > instance.n) {
            throw std::runtime_error("Too many point rows");
        }
    }
    if (static_cast<int>(instance.points.size()) != instance.n) {
        throw std::runtime_error("Point row count differs from n");
    }
    if (instance.points[0].demand != 0) {
        throw std::runtime_error("Depot (node 0) must have demand 0");
    }
    instance.D = buildEuclideanDistanceMatrix(instance.points);
    return instance;
}

// -------------------- ALGORITHM 1: NEAREST NEIGHBOR --------------------
NNSolution nearestNeighborCVRP(const CVRPInstance& inst) {
    NNSolution solution;
    std::vector<unsigned char> unvisited(static_cast<std::size_t>(inst.n), 1);
    unvisited[0] = 0;
    int remaining = inst.n - 1;

    while (remaining > 0) {
        if (static_cast<int>(solution.routes.size()) == inst.K) {
            solution.failureReason = "Reached maximum number of vehicles K";
            break;
        }
        std::vector<int> route;
        route.push_back(0);
        int load = 0;
        int current = 0;

        while (true) {
            int next = -1;
            int bestDistance = std::numeric_limits<int>::max();
            for (int j = 1; j < inst.n; ++j) {
                if (!unvisited[j]) continue;
                if (inst.points[j].demand > inst.Q - load) continue;
                const int distance = inst.D[current][j];
                if (next == -1 || distance < bestDistance ||
                    (distance == bestDistance && j < next)) {
                    next = j;
                    bestDistance = distance;
                }
            }
            if (next == -1) break;
            route.push_back(next);
            load += inst.points[next].demand;
            unvisited[next] = 0;
            --remaining;
            current = next;
        }
        if (route.size() == 1) {
            solution.failureReason = "No unvisited customer fits capacity Q";
            break;
        }
        route.push_back(0);
        for (std::size_t t = 1; t < route.size(); ++t) {
            solution.distance += inst.D[route[t - 1]][route[t]];
        }
        solution.routes.push_back(route);
        solution.loads.push_back(load);
    }
    for (int j = 1; j < inst.n; ++j) {
        if (unvisited[j]) solution.unserved.push_back(j);
    }
    solution.success = solution.unserved.empty();
    return solution;
}

// -------------------- SAVE RESULTS --------------------
void writeSolutionFile(const std::string& output, const NNSolution& sol) {
    std::ofstream out(output.c_str());
    if (!out) throw std::runtime_error("Cannot write solution: " + output);
    for (std::size_t k = 0; k < sol.routes.size(); ++k) {
        out << "Route #" << (k + 1) << ":";
        for (std::size_t t = 1; t + 1 < sol.routes[k].size(); ++t) {
            out << " " << sol.routes[k][t];
        }
        out << "\n";
    }
    out << "Cost " << sol.distance << "\n";
}

void writeFailureFile(const std::string& output, const CVRPInstance& inst,
                      const NNSolution& sol) {
    std::ofstream out(output.c_str());
    if (!out) throw std::runtime_error("Cannot write failure: " + output);
    out << "Status: FAILURE\n"
        << "Instance: " << inst.name << "\n"
        << "Reason: " << sol.failureReason << "\n"
        << "Vehicles used: " << sol.routes.size() << " / " << inst.K << "\n"
        << "Partial cost: " << sol.distance << "\n";
    for (std::size_t k = 0; k < sol.routes.size(); ++k) {
        out << "Route #" << k + 1 << " (load=" << sol.loads[k] << "):";
        for (std::size_t j = 0; j < sol.routes[k].size(); ++j) {
            out << " " << sol.routes[k][j];
        }
        out << "\n";
    }
    out << "Unserved customers (" << sol.unserved.size() << "):";
    for (std::size_t j = 0; j < sol.unserved.size(); ++j) {
        out << " " << sol.unserved[j];
    }
    out << "\n";
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
    const std::string dataRoot = argc > 1 ? argv[1] : joinPath(PROJECT_ROOT, "new_data");
    const std::string outputRoot = argc > 2 ? argv[2] :
        joinPath(joinPath(joinPath(PROJECT_ROOT, "code"), "1.greedy"), "1_NNSolutions");

    try {
        const std::vector<DataFile> files = findAllDataFiles(dataRoot, outputRoot);
        const std::string summaryFile = joinPath(outputRoot, "summary.csv");
        std::ofstream summary(summaryFile.c_str());
        if (!summary) throw std::runtime_error("Cannot write summary.csv");
        summary << "group,instance,n,customers,K,Q,reference,status,vehicles_used,"
                   "distance,gap_percent,time_ms,partial_distance,unserved,reason\n";
        int ok = 0, failed = 0, errors = 0;
        std::cout << "Input: " << dataRoot << "\nOutput: " << outputRoot
                  << "\nFiles found: " << files.size() << "\n\n";

        for (std::size_t i = 0; i < files.size(); ++i) {
            const DataFile& f = files[i];
            const std::string outputDir = joinPath(outputRoot, parentPath(f.relative));
            const std::string outBase = joinPath(outputDir, fileStem(f.relative));
            const std::string group = parentPath(f.relative);
            const std::string instanceName = fileStem(f.relative);
            createDirectories(outputDir);
            // Clear stale files from previous executions.
            std::remove((outBase + ".sol").c_str());
            std::remove((outBase + ".fail.txt").c_str());
            std::remove((outBase + ".error.txt").c_str());

            try {
                const CVRPInstance inst = readCVRPInstance(f.path);
                const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                const NNSolution sol = nearestNeighborCVRP(inst);
                const std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
                const double milliseconds =
                    std::chrono::duration<double, std::milli>(stop - start).count();

                if (sol.success) {
                    writeSolutionFile(outBase + ".sol", sol);
                    ++ok;
                } else {
                    writeFailureFile(outBase + ".fail.txt", inst, sol);
                    ++failed;
                }
                std::ostringstream gap;
                if (sol.success && inst.reference > 0) {
                    gap << std::fixed << std::setprecision(4)
                        << 100.0 * (static_cast<double>(sol.distance) - inst.reference) /
                           static_cast<double>(inst.reference);
                }
                summary << csvCell(group) << ',' << csvCell(inst.name) << ','
                        << inst.n << ',' << inst.n - 1 << ',' << inst.K << ',' << inst.Q
                        << ',' << inst.reference << ','
                        << (sol.success ? "SUCCESS" : "FAILURE") << ','
                        << sol.routes.size() << ',';
                if (sol.success) summary << sol.distance;
                summary << ',' << gap.str() << ',' << std::fixed << std::setprecision(3)
                        << milliseconds << ',' << sol.distance << ',' << sol.unserved.size()
                        << ',' << csvCell(sol.failureReason) << '\n';

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
                summary << csvCell(group) << ',' << csvCell(instanceName)
                        << ",,,,,,ERROR,,,,,,," << csvCell(e.what()) << '\n';
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
