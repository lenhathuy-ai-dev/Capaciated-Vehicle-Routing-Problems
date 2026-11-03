// Algorithm 5 - Nearest Depot First for CVRP
// Code::Blocks / MinGW, C++11. No std::filesystem.
// Reads all .txt files inside new_data and saves results to _5NDF.
// IMPORTANT: Every .sol includes service status and unserved customer IDs.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
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
#endif

using namespace std;

struct Point { double x, y; int demand; };
struct Instance {
    string name;
    int n, K, Q;
    long long opt;
    vector<Point> points;
    vector<vector<int> > D;
};
struct Solution {
    bool allServed;
    vector<vector<int> > routes;
    vector<int> loads;
    vector<int> unserved;
    long long cost;
    string reason;
    Solution() : allServed(false), cost(0) {}
};
struct DataFile { string full, relative; };

// -------------------- Paths & directories --------------------
string joinPath(const string &a, const string &b) {
    if (a.empty()) return b;
    if (b.empty()) return a;
#ifdef _WIN32
    const char separator = '\\';
#else
    const char separator = '/';
#endif
    if (a[a.size()-1] == '/' || a[a.size()-1] == '\\') return a + b;
    return a + separator + b;
}
string parentPath(const string &p) {
    size_t pos = p.find_last_of("/\\");
    return pos == string::npos ? "" : p.substr(0,pos);
}
string stem(const string &p) {
    size_t pos = p.find_last_of("/\\");
    string name = p.substr(pos == string::npos ? 0 : pos+1);
    pos = name.find_last_of('.');
    return pos == string::npos ? name : name.substr(0,pos);
}
bool isTxt(const string &s) {
    if (s.size() < 4 || s[s.size()-4] != '.') return false;
    return tolower((unsigned char)s[s.size()-3]) == 't' &&
           tolower((unsigned char)s[s.size()-2]) == 'x' &&
           tolower((unsigned char)s[s.size()-1]) == 't';
}
bool isDirectory(const string &path) {
#ifdef _WIN32
    DWORD attr = GetFileAttributesA(path.c_str());
    return attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY) != 0;
#else
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
#endif
}
void makeDirectory(const string &path) {
    if (path.empty() || isDirectory(path)) return;
#ifdef _WIN32
    if (!CreateDirectoryA(path.c_str(), NULL) && !isDirectory(path))
#else
    if (mkdir(path.c_str(), 0755) != 0 && !isDirectory(path))
#endif
        throw runtime_error("Cannot create directory: " + path);
}
void ensureDirectory(string path) {
#ifdef _WIN32
    replace(path.begin(), path.end(), '/', '\\');
    size_t start = (path.size() >= 3 && path[1] == ':') ? 3 : 0;
    char sep = '\\';
#else
    size_t start = (!path.empty() && path[0] == '/') ? 1 : 0;
    char sep = '/';
#endif
    for (size_t i = start; i <= path.size(); ++i) {
        if (i != path.size() && path[i] != sep) continue;
        if (i > 0) makeDirectory(path.substr(0,i));
    }
}
void scanFiles(const string &directory, const string &relative,
               const string &outRoot, vector<DataFile> &files) {
#ifdef _WIN32
    WIN32_FIND_DATAA info;
    HANDLE h = FindFirstFileA(joinPath(directory,"*").c_str(), &info);
    if (h == INVALID_HANDLE_VALUE) throw runtime_error("Cannot scan: " + directory);
    do {
        string name = info.cFileName;
        if (name == "." || name == "..") continue;
        string full = joinPath(directory,name);
        string rel = joinPath(relative,name);
        if (info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            if (info.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) continue;
            ensureDirectory(joinPath(outRoot,rel));
            scanFiles(full,rel,outRoot,files);
        } else if (isTxt(name)) {
            files.push_back(DataFile{full,rel});
        }
    } while (FindNextFileA(h,&info));
    FindClose(h);
#else
    DIR *d = opendir(directory.c_str());
    if (!d) throw runtime_error("Cannot scan: " + directory);
    struct dirent *entry;
    while ((entry = readdir(d)) != NULL) {
        string name = entry->d_name;
        if (name == "." || name == "..") continue;
        string full = joinPath(directory,name);
        string rel = joinPath(relative,name);
        struct stat st;
        if (lstat(full.c_str(), &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            ensureDirectory(joinPath(outRoot,rel));
            scanFiles(full,rel,outRoot,files);
        } else if (S_ISREG(st.st_mode) && isTxt(name)) {
            files.push_back(DataFile{full,rel});
        }
    }
    closedir(d);
#endif
}
bool byName(const DataFile &a, const DataFile &b) {
    return a.relative < b.relative;
}

// -------------------- Input data, rounded EUC_2D --------------------
string trim(const string &s) {
    size_t begin = s.find_first_not_of(" \t\r\n");
    if (begin == string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin,end-begin+1);
}
bool dataLine(istream &in, string &line) {
    while (getline(in,line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' ||
            (line.size() > 1 && line[0] == '/' && line[1] == '/')) continue;
        return true;
    }
    return false;
}
Instance readInstance(const string &filename) {
    ifstream in(filename.c_str());
    if (!in) throw runtime_error("Cannot open input file");
    Instance inst;
    inst.name = stem(filename);
    inst.opt = -1;
    string line;
    if (!dataLine(in,line)) throw runtime_error("Empty file");
    // Optional UTF-8 BOM.
    if (line.size() >= 3 && (unsigned char)line[0] == 0xEF &&
        (unsigned char)line[1] == 0xBB && (unsigned char)line[2] == 0xBF)
        line.erase(0,3);
    istringstream header(line);
    if (!(header >> inst.n >> inst.K >> inst.Q))
        throw runtime_error("First line must be: n K Q [OPT]");
    if (!(header >> inst.opt)) inst.opt = -1;
    if (inst.n < 2 || inst.K < 1 || inst.Q < 1)
        throw runtime_error("Invalid n, K or Q");
    while (dataLine(in,line)) {
        istringstream row(line);
        Point p;
        if (!(row >> p.x >> p.y >> p.demand) ||
            !isfinite(p.x) || !isfinite(p.y) || p.demand < 0)
            throw runtime_error("Invalid point row: " + line);
        inst.points.push_back(p);
        if ((int)inst.points.size() > inst.n)
            throw runtime_error("Too many points");
    }
    if ((int)inst.points.size() != inst.n)
        throw runtime_error("Expected n coordinate/demand rows");
    if (inst.points[0].demand != 0)
        throw runtime_error("Depot must be node 0 with demand 0");
    inst.D.assign(inst.n, vector<int>(inst.n,0));
    for (int i = 0; i < inst.n; ++i) {
        for (int j = i+1; j < inst.n; ++j) {
            double dx = inst.points[i].x - inst.points[j].x;
            double dy = inst.points[i].y - inst.points[j].y;
            double d = sqrt(dx*dx + dy*dy);
            if (!isfinite(d) || d > (double)numeric_limits<int>::max() - 1)
                throw runtime_error("Distance out of integer range");
            inst.D[i][j] = inst.D[j][i] = (int)floor(d+0.5);
        }
    }
    return inst;
}

// -------------------- Algorithm 5: Nearest Depot First --------------------
// Each new route begins with the nearest unserved customer to depot 0
// whose demand fits Q. After this seed, append the nearest feasible
// unserved customer to the route endpoint until none fits.
// Distances ties use the smaller customer ID for reproducible results.
Solution nearestDepotFirst(const Instance &inst) {
    Solution sol;
    vector<unsigned char> unvisited(inst.n, 1);
    unvisited[0] = 0;  // Depot is not a customer.
    int remaining = inst.n - 1;

    while (remaining > 0) {
        if (static_cast<int>(sol.routes.size()) >= inst.K) {
            sol.reason = "Maximum K vehicles reached";
            break;
        }

        // Step 5: choose the closest feasible seed to the depot.
        int seed = -1;
        int seedDistance = numeric_limits<int>::max();
        for (int j = 1; j < inst.n; ++j) {
            if (!unvisited[j] || inst.points[j].demand > inst.Q) continue;
            const int distance = inst.D[0][j];
            if (seed == -1 || distance < seedDistance ||
                (distance == seedDistance && j < seed)) {
                seed = j;
                seedDistance = distance;
            }
        }
        if (seed == -1) {
            sol.reason = "No unserved customer can start a route (demand exceeds Q)";
            break;
        }

        // Steps 6-9: initialize route from this seed.
        vector<int> route;
        route.push_back(0);
        route.push_back(seed);
        int load = inst.points[seed].demand;
        int current = seed;
        unvisited[seed] = 0;
        --remaining;

        // Steps 10-15: append nearest feasible unserved customer.
        while (remaining > 0) {
            int next = -1;
            int bestDistance = numeric_limits<int>::max();
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

        // Steps 16-17: close the current route at the depot.
        route.push_back(0);
        for (size_t t = 1; t < route.size(); ++t)
            sol.cost += inst.D[route[t - 1]][route[t]];
        sol.routes.push_back(route);
        sol.loads.push_back(load);
    }

    for (int j = 1; j < inst.n; ++j)
        if (unvisited[j]) sol.unserved.push_back(j);
    sol.allServed = sol.unserved.empty();
    if (!sol.allServed && sol.reason.empty())
        sol.reason = "Some customers remain unserved";
    return sol;
}

// -------------------- Save EVERY result to .sol --------------------
void writeSolution(const string &file, const Instance &inst, const Solution &sol) {
    ofstream out(file.c_str(),ios::out | ios::trunc);
    if (!out) throw runtime_error("Cannot write: " + file);
    for (size_t k = 0; k < sol.routes.size(); ++k) {
        out << "Route #" << k+1 << ":";
        for (size_t p = 1; p+1 < sol.routes[k].size(); ++p)
            out << " " << sol.routes[k][p];
        out << "\n";
    }
    // Cost is valid only when every customer has been served.
    if (sol.allServed) out << "Cost " << sol.cost << "\n";
    else out << "Partial cost " << sol.cost << "\n";

    // ---- NEW: mandatory service status lines ----
    out << "Status: " << (sol.allServed ? "SUCCESS" : "FAILURE") << "\n";
    out << "All customers served: " << (sol.allServed ? "YES" : "NO") << "\n";
    out << "Served customers: " << (inst.n - 1 - (int)sol.unserved.size())
        << " / " << (inst.n-1) << "\n";
    out << "Unserved customers (" << sol.unserved.size() << "):";
    if (sol.unserved.empty()) out << " None";
    else for (size_t i=0; i<sol.unserved.size(); ++i)
        out << " " << sol.unserved[i];
    out << "\n";
    out << "Vehicles used: " << sol.routes.size() << " / " << inst.K << "\n";
    if (!sol.allServed) out << "Failure reason: " << sol.reason << "\n";
    out << "Generator: Algorithm 5 - Nearest Depot First (NDF)\n";
    if (!out) throw runtime_error("Write failed: " + file);
}
string toCSV(const string &s) {
    if (s.find_first_of(",\"\r\n") == string::npos) return s;
    string t = "\"";
    for (size_t i=0; i<s.size(); ++i) {
        if (s[i] == '"') t += '"';
        t += s[i];
    }
    return t + '"';
}
string joinUnserved(const vector<int> &ids) {
    ostringstream ss;
    for (size_t i=0; i<ids.size(); ++i) {
        if (i) ss << " ";
        ss << ids[i];
    }
    return ss.str();
}

int main(int argc, char **argv) {
    // Fixed paths requested by the teacher.
    const string PROJECT =
        "D:\\_2026\\1.CNTT\\7.metaheuristic\\01_VRP_Vehicle_Routing_Problem\\"
        "01_CVRP_Capacitated_Vehicle_Routing_Problem";
    const string inputRoot = argc > 1 ? argv[1] : joinPath(PROJECT,"new_data");
    const string outputRoot = argc > 2 ? argv[2] :
        joinPath(joinPath(joinPath(PROJECT,"code"),"1.greedy"),"_5NDF");
    try {
        if (!isDirectory(inputRoot))
            throw runtime_error("Data directory does not exist: " + inputRoot);
        ensureDirectory(outputRoot);
        vector<DataFile> files;
        scanFiles(inputRoot,"",outputRoot,files);
        sort(files.begin(),files.end(),byName);
        if (files.empty()) throw runtime_error("No .txt data files found");
        const string summaryPath = joinPath(outputRoot,"summary.csv");
        ofstream csv(summaryPath.c_str(),ios::out | ios::trunc);
        if (!csv) throw runtime_error("Cannot write summary.csv");
        csv << "group,instance,status,n,K,Q,OPT,vehicles_used,served,unserved,"
               "unserved_ids,cost,partial_cost,gap_percent,time_ms,reason\n";
        int success = 0, failure = 0, errors = 0;
        cout << "ALGORITHM 5 - NEAREST DEPOT FIRST (NDF)\n";
        cout << "DATA:   " << inputRoot << "\n";
        cout << "OUTPUT: " << outputRoot << "\n";
        cout << "FILES:  " << files.size() << "\n\n";
        for (size_t f=0; f<files.size(); ++f) {
            string relative = files[f].relative;
            string group = parentPath(relative);
            string name = stem(relative);
            string destination = joinPath(outputRoot,relative);
            string solFile = joinPath(parentPath(destination),name + ".sol");
            string errFile = joinPath(parentPath(destination),name + ".error.txt");
            ensureDirectory(parentPath(solFile));
            try {
                Instance inst = readInstance(files[f].full);
                auto t0 = chrono::steady_clock::now();
                Solution sol = nearestDepotFirst(inst);
                auto t1 = chrono::steady_clock::now();
                double ms = chrono::duration<double,milli>(t1-t0).count();
                writeSolution(solFile,inst,sol);
                // Avoid stale error from a previous failed run.
                remove(errFile.c_str());
                if (sol.allServed) ++success; else ++failure;
                ostringstream gap;
                if (sol.allServed && inst.opt > 0)
                    gap << fixed << setprecision(4) <<
                        100.0 * (sol.cost - inst.opt) / inst.opt;
                csv << toCSV(group) << "," << toCSV(inst.name) << ","
                    << (sol.allServed ? "SUCCESS" : "FAILURE") << ","
                    << inst.n << "," << inst.K << "," << inst.Q << "," << inst.opt
                    << "," << sol.routes.size() << ","
                    << (inst.n-1-(int)sol.unserved.size()) << ","
                    << sol.unserved.size() << "," << toCSV(joinUnserved(sol.unserved)) << ",";
                if (sol.allServed) csv << sol.cost;
                csv << "," << sol.cost << "," << gap.str() << ","
                    << fixed << setprecision(4) << ms << "," << toCSV(sol.reason) << "\n";
                cout << "[" << (sol.allServed ? "SUCCESS" : "FAILURE") << "] "
                     << relative << " | Served: " << (inst.n-1-(int)sol.unserved.size())
                     << "/" << (inst.n-1) << " | Unserved: " << sol.unserved.size() << "\n";
                cout << "  SAVED: " << solFile << "\n";
            } catch (const exception &e) {
                ++errors;
                remove(solFile.c_str()); // No stale .sol for an invalid input file.
                ofstream err(errFile.c_str());
                if (err) err << "Status: ERROR\nFile: " << relative << "\nReason: " << e.what() << "\n";
                csv << toCSV(group) << "," << toCSV(name)
                    << ",ERROR,,,,,,,,,,,,," << toCSV(e.what()) << "\n";
                cerr << "[ERROR] " << relative << ": " << e.what() << "\n";
            }
        }
        cout << "\nDONE: SUCCESS=" << success << " | FAILURE=" << failure
             << " | ERROR=" << errors << "\n";
        cout << "SUMMARY: " << summaryPath << "\n";
        return errors == 0 ? 0 : 1;
    } catch (const exception &e) {
        cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }
}
