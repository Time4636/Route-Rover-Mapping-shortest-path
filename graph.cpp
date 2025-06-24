#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <set>
#include <limits>
#include <algorithm>
#include <string>
#include <unordered_map>

using namespace std;

struct Edge {
    int to;
    double weight;
};

struct Path {
    vector<int> nodes;
    double cost;

    // Comparison operator needed for using Path inside std::set
    bool operator<(const Path& other) const {
        if (cost != other.cost) return cost < other.cost;
        return nodes < other.nodes;
    }
};

class Graph {
public:
    Graph(int n) : adj(n) {}

    void addEdge(int u, int v, double w) {
        adj[u].push_back((Edge){v, w});
        adj[v].push_back((Edge){u, w});  // Undirected graph
    }

    Path dijkstra(int src, int dest) const {
        int n = adj.size();
        vector<double> dist(n, numeric_limits<double>::max());
        vector<int> parent(n, -1);
        dist[src] = 0;
        typedef pair<double, int> P; // dist, node
        priority_queue<P, vector<P>, greater<P> > pq;
        pq.push(make_pair(0, src));

        while(!pq.empty()) {
            double curDist = pq.top().first;
            int u = pq.top().second;
            pq.pop();
            if (curDist > dist[u]) continue;
            if (u == dest) break;

            for (size_t i = 0; i < adj[u].size(); i++) {
                Edge e = adj[u][i];
                int v = e.to;
                double w = e.weight;
                if (dist[u] + w < dist[v]) {
                    dist[v] = dist[u] + w;
                    parent[v] = u;
                    pq.push(make_pair(dist[v], v));
                }
            }
        }

        Path path;
        if (dist[dest] == numeric_limits<double>::max()) {
            path.cost = -1; // no path
            return path;
        }
        path.cost = dist[dest];

        // reconstruct path
        for (int cur = dest; cur != -1; cur = parent[cur]) {
            path.nodes.push_back(cur);
        }
        reverse(path.nodes.begin(), path.nodes.end());
        return path;
    }

    vector<Path> yenKShortestPaths(int src, int dest, int K) const {
        vector<Path> result;
        Path shortest = dijkstra(src, dest);
        if (shortest.cost == -1) return result;

        result.push_back(shortest);

        // set for candidate paths
        set<pair<double, Path> > candidates;

        for (int k = 1; k < K; ++k) {
            const Path& lastPath = result[k-1];
            for (size_t i = 0; i < lastPath.nodes.size() - 1; ++i) {
                int spurNode = lastPath.nodes[i];
                vector<int> rootPath(lastPath.nodes.begin(), lastPath.nodes.begin() + i + 1);

                Graph tempGraph = *this;

                for (size_t j = 0; j < result.size(); ++j) {
                    const Path& p = result[j];
                    if (p.nodes.size() > i && equal(rootPath.begin(), rootPath.end(), p.nodes.begin())) {
                        int u = p.nodes[i];
                        int v = p.nodes[i+1];
                        tempGraph.removeEdge(u, v);
                    }
                }
                for (size_t j = 0; j < rootPath.size(); ++j) {
                    int node = rootPath[j];
                    if (node != spurNode)
                        tempGraph.removeNode(node);
                }

                Path spurPath = tempGraph.dijkstra(spurNode, dest);
                if (spurPath.cost != -1) {
                    vector<int> totalPath = rootPath;
                    totalPath.pop_back();
                    totalPath.insert(totalPath.end(), spurPath.nodes.begin(), spurPath.nodes.end());
                    double totalCost = 0;
                    for (size_t x = 0; x < totalPath.size() - 1; ++x) {
                        totalCost += getWeight(totalPath[x], totalPath[x+1]);
                    }
                    Path newPath = {totalPath, totalCost};
                    candidates.insert(make_pair(totalCost, newPath));
                }
            }
            if (candidates.empty()) break;

            auto it = candidates.begin();
            result.push_back(it->second);
            candidates.erase(it);
        }
        return result;
    }

private:
    vector<vector<Edge> > adj;

    void removeEdge(int u, int v) {
        vector<Edge>& edgesU = adj[u];
        edgesU.erase(remove_if(edgesU.begin(), edgesU.end(),
            [v](const Edge& e){ return e.to == v; }), edgesU.end());
        vector<Edge>& edgesV = adj[v];
        edgesV.erase(remove_if(edgesV.begin(), edgesV.end(),
            [u](const Edge& e){ return e.to == u; }), edgesV.end());
    }

    void removeNode(int u) {
        adj[u].clear();
        for (size_t i = 0; i < adj.size(); i++) {
            vector<Edge>& edges = adj[i];
            edges.erase(remove_if(edges.begin(), edges.end(),
                [u](const Edge& e){ return e.to == u; }), edges.end());
        }
    }

    double getWeight(int u, int v) const {
        for (size_t i = 0; i < adj[u].size(); i++) {
            if (adj[u][i].to == v) return adj[u][i].weight;
        }
        return numeric_limits<double>::max();
    }
};

int main() {
    // Step 1: Read CSV file "uttarakhand_road_graph.csv"
    ifstream fin("uttarakhand_road_graph.csv");
    if (!fin) {
        cerr << "Cannot open uttarakhand_road_graph.csv\n";
        return 1;
    }

    unordered_map<string, int> cityToId;
    vector<string> idToCity;
    vector< tuple<int,int,double> > edgesRaw;

    string line;
    // Read header
    getline(fin, line);

    while (getline(fin, line)) {
        if (line.empty()) continue;
        stringstream ss(line);
        string fromCity, toCity, distStr;
        getline(ss, fromCity, ',');
        getline(ss, toCity, ',');
        getline(ss, distStr, ',');

        if (cityToId.find(fromCity) == cityToId.end()) {
            cityToId[fromCity] = idToCity.size();
            idToCity.push_back(fromCity);
        }
        if (cityToId.find(toCity) == cityToId.end()) {
            cityToId[toCity] = idToCity.size();
            idToCity.push_back(toCity);
        }

        int u = cityToId[fromCity];
        int v = cityToId[toCity];
        double w = stod(distStr);

        edgesRaw.push_back(make_tuple(u,v,w));
    }
    fin.close();

    Graph graph(idToCity.size());
    for (size_t i = 0; i < edgesRaw.size(); ++i) {
        graph.addEdge(get<0>(edgesRaw[i]), get<1>(edgesRaw[i]), get<2>(edgesRaw[i]));
    }

    // Step 2: Read input.txt (source and dest city names)
    ifstream fin2("input.txt");
    if (!fin2) {
        cerr << "Cannot open input.txt\n";
        return 1;
    }
    string srcCity, destCity;
    fin2 >> srcCity >> destCity;
    fin2.close();

    if (cityToId.find(srcCity) == cityToId.end() || cityToId.find(destCity) == cityToId.end()) {
        cerr << "Source or destination city not found\n";
        return 1;
    }

    int src = cityToId[srcCity];
    int dest = cityToId[destCity];

    // Step 3: Find top 3 shortest paths
    int K = 3;
    vector<Path> paths = graph.yenKShortestPaths(src, dest, K);

    // Step 4: Write output.txt (paths with city names)
    ofstream fout("output.txt");
    if (!fout) {
        cerr << "Cannot open output.txt\n";
        return 1;
    }

    if (paths.empty()) {
        fout << "No paths found from " << srcCity << " to " << destCity << "\n";
    } else {
        for (size_t i = 0; i < paths.size(); ++i) {
            fout << "Path " << (i+1) << " (cost: " << paths[i].cost << " km): ";
            for (size_t j = 0; j < paths[i].nodes.size(); ++j) {
                fout << idToCity[paths[i].nodes[j]];
                if (j + 1 < paths[i].nodes.size()) fout << " -> ";
            }
            fout << "\n";
        }
    }
    fout.close();

    return 0;
}
