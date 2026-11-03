// Algorithm 7: Global Cheapest Arc - simplified route-chain construction for CVRP
// C++11, Code::Blocks/MinGW compatible. Does NOT use std::filesystem.
// Reads every new_data/**/*.txt and writes code/1.greedy/_7GCA/**/{name}.sol.
// All .sol files report served/unserved customers, including FAILURE cases.
// Optional arguments: program.exe "inputRoot" "outputRoot"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <dirent.h>
#  include <sys/stat.h>
#endif

using namespace std;

struct Customer {
    double x, y;
    int demand;
};

struct Instance {
    string name;
    int n, K, Q;                 // n INCLUDES depot 0
    long long referenceCost;    // OPT/BKS; -1 means unknown
    vector<Customer> point;
    vector<vector<int> > D;     // rounded EUC_2D distance matrix
};

struct Edge {
    int distance, i, j;
};

struct Chain {
    vector<int> nodes;         // customers only, no depot
    long long load;
};

struct Solution {
    bool allServed;
    int chainsAfterMerging;
    vector<vector<int> > routes; // each route includes depot at both ends
    vector<long long> loads;
    vector<int> unserved;
    long long cost;             // partial cost if FAILURE
    string reason;
    Solution() : allServed(false), chainsAfterMerging(0), cost(0) {}
};

struct DataFile {
    string fullPath;
    string relativePath;
};

// --------------------- Portable path operations ---------------------
string joinPath(const string &a, const string &b) {
    if (a.empty()) return b;
    if (b.empty()) return a;
#ifdef _WIN32
    const char sep = '\\';
#else
    const char sep = '/';
#endif
    if (a[a.size()-1] == '/' || a[a.size()-1] == '\\') return a + b;
    return a + sep + b;
}

string parentPath(const string &path) {
    const size_t p = path.find_last_of("/\\");
    return p == string::npos ? "" : path.substr(0, p);
}

string basenameNoExt(const string &path) {
    const size_t p = path.find_last_of("/\\");
    string name = path.substr(p == string::npos ? 0 : p+1);
    const size_t dot = name.find_last_of('.');
    return dot == string::npos ? name : name.substr(0, dot);
}

bool isTxtFile(const string &name) {
    if (name.size() < 4) return false;
    const size_t p = name.size() - 4;
    return name[p] == '.' &&
           tolower((unsigned char)name[p+1]) == 't' &&
           tolower((unsigned char)name[p+2]) == 'x' &&
           tolower((unsigned char)name[p+3]) == 't';
}

bool directoryExists(const string &path) {
#ifdef _WIN32
    const DWORD attributes = GetFileAttributesA(path.c_str());
    return attributes != INVALID_FILE_ATTRIBUTES &&
           (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
#else
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
#endif
}

void createOneDirectory(const string &path) {
    if (path.empty() || directoryExists(path)) return;
#ifdef _WIN32
    if (!CreateDirectoryA(path.c_str(), NULL) && !directoryExists(path))
#else
    if (mkdir(path.c_str(), 0755) != 0 && !directoryExists(path))
#endif
        throw runtime_error("Cannot create directory: " + path);
}

void ensureDirectory(string path) {
#ifdef _WIN32
    replace(path.begin(), path.end(), '/', '\\');
    const char sep = '\\';
    const size_t start = path.size() >= 3 && path[1] == ':' ? 3 : 0;
#else
    const char sep = '/';
    const size_t start = !path.empty() && path[0] == '/' ? 1 : 0;
#endif
    for (size_t i = start; i <= path.size(); ++i) {
        if (i == path.size() || path[i] == sep) {
            if (i > 0) createOneDirectory(path.substr(0, i));
        }
    }
}

void scanDataFiles(const string &directory, const string &relative,
                   vector<DataFile> &files) {
#ifdef _WIN32
    WIN32_FIND_DATAA entry;
    const HANDLE handle = FindFirstFileA(joinPath(directory,"*").c_str(), &entry);
    if (handle == INVALID_HANDLE_VALUE)
        throw runtime_error("Cannot scan: " + directory);
    do {
        const string name = entry.cFileName;
        if (name == "." || name == "..") continue;
        const string full = joinPath(directory, name);
        const string rel = joinPath(relative, name);
        if (entry.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            if (entry.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) continue;
            scanDataFiles(full, rel, files);
        } else if (isTxtFile(name)) {
            files.push_back(DataFile{full, rel});
        }
    } while (FindNextFileA(handle, &entry));
    FindClose(handle);
#else
    DIR *dir = opendir(directory.c_str());
    if (!dir) throw runtime_error("Cannot scan: " + directory);
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        const string name = entry->d_name;
        if (name == "." || name == "..") continue;
        const string full = joinPath(directory, name);
        const string rel = joinPath(relative, name);
        struct stat info;
        if (lstat(full.c_str(), &info) != 0) continue;
        if (S_ISDIR(info.st_mode)) {
            scanDataFiles(full, rel, files);
        } else if (S_ISREG(info.st_mode) && isTxtFile(name)) {
            files.push_back(DataFile{full, rel});
        }
    }
    closedir(dir);
#endif
}

bool dataFileLess(const DataFile &a, const DataFile &b) {
    return a.relativePath < b.relativePath;
}

// --------------------- Read normalized CVRP instances ---------------------
string trim(const string &s) {
    const size_t first = s.find_first_not_of(" \t\r\n");
    if (first == string::npos) return "";
    const size_t last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last-first+1);
}

bool nextDataLine(istream &input, string &line) {
    while (getline(input, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' ||
            (line.size() >= 2 && line[0] == '/' && line[1] == '/'))
            continue;
        return true;
    }
    return false;
}

Instance readInstance(const string &filename) {
    ifstream input(filename.c_str());
    if (!input) throw runtime_error("Cannot open input file");

    Instance inst;
    inst.name = basenameNoExt(filename);
    inst.referenceCost = -1;
    string line;
    if (!nextDataLine(input, line)) throw runtime_error("Empty data file");
    // Strip UTF-8 BOM, if present.
    if (line.size() >= 3 && (unsigned char)line[0] == 0xEF &&
        (unsigned char)line[1] == 0xBB && (unsigned char)line[2] == 0xBF)
        line.erase(0, 3);

    istringstream header(line);
    if (!(header >> inst.n >> inst.K >> inst.Q))
        throw runtime_error("Expected first line: n K Q [OPT]");
    if (!(header >> inst.referenceCost)) inst.referenceCost = -1;
    if (inst.n < 2 || inst.K < 1 || inst.Q < 1)
        throw runtime_error("Invalid n, K or Q");

    while (nextDataLine(input, line)) {
        istringstream row(line);
        Customer p;
        if (!(row >> p.x >> p.y >> p.demand) ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || p.demand < 0)
            throw runtime_error("Invalid coordinate/demand row: " + line);
        inst.point.push_back(p);
        if ((int)inst.point.size() > inst.n)
            throw runtime_error("More than n coordinate/demand rows");
    }
    if ((int)inst.point.size() != inst.n)
        throw runtime_error("Wrong number of coordinate/demand rows");
    if (inst.point[0].demand != 0)
        throw runtime_error("Depot index 0 must have demand 0");

    // Benchmark EUC_2D distance: floor(sqrt(dx^2+dy^2) + 0.5).
    inst.D.assign(inst.n, vector<int>(inst.n, 0));
    for (int i = 0; i < inst.n; ++i) {
        for (int j = i+1; j < inst.n; ++j) {
            const double dx = inst.point[i].x - inst.point[j].x;
            const double dy = inst.point[i].y - inst.point[j].y;
            const double distance = std::hypot(dx, dy);
            if (!std::isfinite(distance) ||
                distance > (double)numeric_limits<int>::max()-1)
                throw runtime_error("Distance out of integer range");
            const int rounded = (int)floor(distance + 0.5);
            inst.D[i][j] = inst.D[j][i] = rounded;
        }
    }
    return inst;
}

// --------------------- Algorithm 7: Global Cheapest Arc ---------------------
int findRoot(vector<int> &parent, int x) {
    if (parent[x] != x) parent[x] = findRoot(parent, parent[x]);
    return parent[x];
}

bool edgeLess(const Edge &a, const Edge &b) {
    if (a.distance != b.distance) return a.distance < b.distance;
    if (a.i != b.i) return a.i < b.i;
    return a.j < b.j;
}

bool chainChoiceLess(const Chain &a, const Chain &b) {
    // On FAILURE, choose at most K feasible chains covering the most customers.
    if (a.nodes.size() != b.nodes.size())
        return a.nodes.size() > b.nodes.size();
    return *min_element(a.nodes.begin(),a.nodes.end()) <
           *min_element(b.nodes.begin(),b.nodes.end());
}

bool chainDisplayLess(const Chain &a, const Chain &b) {
    return *min_element(a.nodes.begin(),a.nodes.end()) <
           *min_element(b.nodes.begin(),b.nodes.end());
}

Solution globalCheapestArc(const Instance &inst) {
    Solution solution;
    const int n = inst.n;
    vector<int> parent(n);
    vector<long long> load(n, 0);
    vector<vector<int> > routeNodes(n);
    for (int i = 1; i < n; ++i) {
        parent[i] = i;
        load[i] = inst.point[i].demand;
        routeNodes[i].push_back(i); // singleton chain [i]
    }
    int chainCount = n-1;

    vector<Edge> edges;
    edges.reserve((size_t)(n-1)*(n-2)/2);
    for (int i = 1; i < n; ++i)
        for (int j = i+1; j < n; ++j)
            edges.push_back(Edge{inst.D[i][j], i, j});
    sort(edges.begin(), edges.end(), edgeLess);

    // Steps 4-12: connect disjoint chains only at their endpoints.
    for (size_t t = 0; t < edges.size() && chainCount > inst.K; ++t) {
        const Edge &e = edges[t];
        const int rootA = findRoot(parent, e.i);
        const int rootB = findRoot(parent, e.j);
        if (rootA == rootB) continue; // would create a cycle

        vector<int> &A = routeNodes[rootA];
        vector<int> &B = routeNodes[rootB];
        if ((A.front() != e.i && A.back() != e.i) ||
            (B.front() != e.j && B.back() != e.j)) continue;
        if (load[rootA] + load[rootB] > inst.Q) continue;

        // Orient A so i is its last node; B so j is its first node.
        if (A.front() == e.i) reverse(A.begin(), A.end());
        if (B.back() == e.j) reverse(B.begin(), B.end());

        // Join A --(i,j)--> B, without depot in the middle.
        A.insert(A.end(), B.begin(), B.end());
        B.clear();
        load[rootA] += load[rootB];
        parent[rootB] = rootA;
        --chainCount;
    }
    solution.chainsAfterMerging = chainCount;

    vector<Chain> allChains;
    for (int i = 1; i < n; ++i) {
        if (findRoot(parent, i) == i && !routeNodes[i].empty()) {
            Chain c;
            c.nodes.swap(routeNodes[i]);
            c.load = load[i];
            allChains.push_back(c);
        }
    }

    vector<Chain> eligible;
    for (size_t k = 0; k < allChains.size(); ++k) {
        if (allChains[k].load <= inst.Q) eligible.push_back(allChains[k]);
    }
    // If impossible to use every chain, select up to K complete, feasible
    // chains for a PARTIAL answer; leave all other customers unserved.
    sort(eligible.begin(), eligible.end(), chainChoiceLess);
    if ((int)eligible.size() > inst.K) eligible.resize(inst.K);
    sort(eligible.begin(), eligible.end(), chainDisplayLess);

    vector<unsigned char> served(n, 0);
    for (size_t k = 0; k < eligible.size(); ++k) {
        const Chain &chain = eligible[k];
        vector<int> route;
        route.push_back(0);
        for (size_t t = 0; t < chain.nodes.size(); ++t) {
            const int customer = chain.nodes[t];
            route.push_back(customer);
            served[customer] = 1;
        }
        route.push_back(0);
        for (size_t t = 1; t < route.size(); ++t)
            solution.cost += inst.D[route[t-1]][route[t]];
        solution.routes.push_back(route);
        solution.loads.push_back(chain.load);
    }
    for (int i = 1; i < n; ++i)
        if (!served[i]) solution.unserved.push_back(i);
    solution.allServed = solution.unserved.empty();

    if (!solution.allServed) {
        ostringstream ss;
        if (chainCount > inst.K)
            ss << "Remaining " << chainCount << " chains exceed K=" << inst.K;
        bool oversized = false;
        for (int i = 1; i < n; ++i)
            if (inst.point[i].demand > inst.Q) oversized = true;
        if (oversized) {
            if (ss.tellp() > 0) ss << "; ";
            ss << "some customer demands exceed Q";
        }
        solution.reason = ss.str();
        if (solution.reason.empty())
            solution.reason = "Not every customer can be assigned to K feasible routes";
    }
    return solution;
}

// --------------------- Solution and benchmark outputs ---------------------
string csvField(const string &s) {
    if (s.find_first_of(",\"\r\n") == string::npos) return s;
    string result = "\"";
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '"') result += '"';
        result += s[i];
    }
    return result + '"';
}

string idList(const vector<int> &items) {
    ostringstream out;
    for (size_t i = 0; i < items.size(); ++i) {
        if (i) out << " ";
        out << items[i];
    }
    return out.str();
}

void saveSolution(const string &filename, const Instance &inst,
                  const Solution &sol) {
    ofstream out(filename.c_str(), ios::out | ios::trunc);
    if (!out) throw runtime_error("Cannot write solution: " + filename);
    for (size_t k = 0; k < sol.routes.size(); ++k) {
        out << "Route #" << k+1 << ":";
        for (size_t j = 1; j+1 < sol.routes[k].size(); ++j)
            out << " " << sol.routes[k][j];
        out << "\n";
    }
    if (sol.allServed) out << "Cost " << sol.cost << "\n";
    else out << "Partial cost " << sol.cost << "\n";
    out << "Status: " << (sol.allServed ? "SUCCESS" : "FAILURE") << "\n";
    out << "All customers served: " << (sol.allServed ? "YES" : "NO") << "\n";
    out << "Served customers: " << inst.n-1-(int)sol.unserved.size()
        << " / " << inst.n-1 << "\n";
    out << "Unserved customers (" << sol.unserved.size() << "):";
    if (sol.unserved.empty()) out << " None";
    else for (size_t j = 0; j < sol.unserved.size(); ++j)
        out << " " << sol.unserved[j];
    out << "\n";
    out << "Vehicles used: " << sol.routes.size() << " / " << inst.K << "\n";
    out << "Chains after merging: " << sol.chainsAfterMerging << "\n";
    if (!sol.allServed)
        out << "Failure reason: " << sol.reason << "\n";
    out << "Generator: Algorithm 7 - Global Cheapest Arc (GCA)\n";
    if (!out) throw runtime_error("Cannot finish writing: " + filename);
}

int main(int argc, char **argv) {
    const string PROJECT =
        "D:\\_2026\\1.CNTT\\7.metaheuristic\\01_VRP_Vehicle_Routing_Problem\\"
        "01_CVRP_Capacitated_Vehicle_Routing_Problem";
    const string dataRoot = argc > 1 ? argv[1] : joinPath(PROJECT, "new_data");
    const string outputRoot = argc > 2 ? argv[2] :
        joinPath(joinPath(joinPath(PROJECT, "code"), "1.greedy"), "_7GCA");

    try {
        if (!directoryExists(dataRoot))
            throw runtime_error("Data folder not found: " + dataRoot);
        ensureDirectory(outputRoot);
        vector<DataFile> files;
        scanDataFiles(dataRoot, "", files);
        sort(files.begin(), files.end(), dataFileLess);
        if (files.empty()) throw runtime_error("No .txt input files found");

        const string csvPath = joinPath(outputRoot, "summary.csv");
        ofstream csv(csvPath.c_str(), ios::out | ios::trunc);
        if (!csv) throw runtime_error("Cannot open summary.csv for writing");
        csv << "group,instance,status,n,K,Q,OPT,vehicles_used,served,unserved,"
               "unserved_ids,cost,partial_cost,gap_percent,time_ms,"
               "chains_remaining,reason\n";

        int successes = 0, failures = 0, errors = 0;
        cout << "ALGORITHM 7 - GLOBAL CHEAPEST ARC\n";
        cout << "DATA: " << dataRoot << "\n";
        cout << "OUTPUT: " << outputRoot << "\n";
        cout << "FILES: " << files.size() << "\n\n";

        for (size_t f = 0; f < files.size(); ++f) {
            const string relative = files[f].relativePath;
            const string group = parentPath(relative);
            const string name = basenameNoExt(relative);
            const string outputDir = joinPath(outputRoot, group);
            const string solutionFile = joinPath(outputDir, name + ".sol");
            const string errorFile = joinPath(outputDir, name + ".error.txt");
            try {
                ensureDirectory(outputDir);
                Instance inst = readInstance(files[f].fullPath);
                const chrono::steady_clock::time_point t0 = chrono::steady_clock::now();
                Solution sol = globalCheapestArc(inst);
                const chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
                const double ms = chrono::duration<double, milli>(t1-t0).count();
                saveSolution(solutionFile, inst, sol);
                remove(errorFile.c_str()); // remove stale error from an earlier run

                if (sol.allServed) ++successes; else ++failures;
                ostringstream gap;
                if (sol.allServed && inst.referenceCost > 0)
                    gap << fixed << setprecision(4)
                        << 100.0 * (sol.cost - inst.referenceCost) / inst.referenceCost;
                csv << csvField(group) << "," << csvField(inst.name) << ","
                    << (sol.allServed ? "SUCCESS" : "FAILURE") << ","
                    << inst.n << "," << inst.K << "," << inst.Q << ","
                    << inst.referenceCost << "," << sol.routes.size() << ","
                    << inst.n-1-(int)sol.unserved.size() << ","
                    << sol.unserved.size() << "," << csvField(idList(sol.unserved)) << ",";
                if (sol.allServed) csv << sol.cost;
                csv << "," << sol.cost << "," << gap.str() << ","
                    << fixed << setprecision(4) << ms << ","
                    << sol.chainsAfterMerging << "," << csvField(sol.reason) << "\n";
                cout << "[" << (sol.allServed ? "SUCCESS" : "FAILURE") << "] "
                     << relative << " | Served "
                     << inst.n-1-(int)sol.unserved.size() << "/" << inst.n-1
                     << " | Routes " << sol.routes.size() << "/" << inst.K
                     << " | " << (sol.allServed ? "Cost " : "Partial cost ")
                     << sol.cost << "\n";
            } catch (const exception &e) {
                ++errors;
                remove(solutionFile.c_str()); // avoid leaving a stale .sol
                ofstream error(errorFile.c_str());
                if (error) error << "Status: ERROR\nFile: " << relative
                                 << "\nReason: " << e.what() << "\n";
                csv << csvField(group) << "," << csvField(name)
                    << ",ERROR,,,,,,,,,,,,,," << csvField(e.what()) << "\n";
                cerr << "[ERROR] " << relative << ": " << e.what() << "\n";
            }
        }
        cout << "\nDONE: SUCCESS=" << successes << " | FAILURE=" << failures
             << " | ERROR=" << errors << "\n";
        cout << "SUMMARY: " << csvPath << "\n";
        return errors == 0 ? 0 : 1;
    } catch (const exception &e) {
        cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }
}
